#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include "common/types.hpp"
#include "io/seqlock_ring.hpp"
#include "io/sensor_task.hpp"

using ptc::SensorSample;
using ptc::io::SeqlockRing;

namespace {
SensorSample make(std::uint64_t seq) {
  SensorSample s;
  s.seq = seq;
  s.t_ns = static_cast<std::int64_t>(seq) * 200'000;
  s.temperature_c = static_cast<double>(seq);
  s.pressure_torr = 2.0 * static_cast<double>(seq);
  s.valve_position = 0.5;
  return s;
}
}  // namespace

TEST(SeqlockRing, EmptyDrainDeliversNothing) {
  SeqlockRing<SensorSample, 8> ring;
  int calls = 0;
  const auto r = ring.drain([&](const SensorSample&) { ++calls; });
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(r.delivered, 0u);
  EXPECT_EQ(r.lost, 0u);
}

TEST(SeqlockRing, DeliversInOrderWithFullPayload) {
  SeqlockRing<SensorSample, 8> ring;
  for (std::uint64_t i = 0; i < 5; ++i) ring.push(make(i));
  std::vector<SensorSample> got;
  const auto r = ring.drain([&](const SensorSample& s) { got.push_back(s); });
  EXPECT_EQ(r.delivered, 5u);
  EXPECT_EQ(r.lost, 0u);
  ASSERT_EQ(got.size(), 5u);
  for (std::uint64_t i = 0; i < 5; ++i) {
    EXPECT_EQ(got[i].seq, i);
    EXPECT_DOUBLE_EQ(got[i].temperature_c, static_cast<double>(i));
    EXPECT_DOUBLE_EQ(got[i].pressure_torr, 2.0 * static_cast<double>(i));
    EXPECT_EQ(got[i].t_ns, static_cast<std::int64_t>(i) * 200'000);
  }
  // A second drain sees nothing new.
  EXPECT_EQ(ring.drain([](const SensorSample&) {}).delivered, 0u);
}

TEST(SeqlockRing, ExactlyFullIsNotAnOverrun) {
  SeqlockRing<SensorSample, 8> ring;
  for (std::uint64_t i = 0; i < 8; ++i) ring.push(make(i));
  const auto r = ring.drain([](const SensorSample&) {});
  EXPECT_EQ(r.delivered, 8u);
  EXPECT_EQ(r.lost, 0u);
}

TEST(SeqlockRing, OverrunIsCountedExactlyAndNewestDataSurvives) {
  SeqlockRing<SensorSample, 8> ring;
  for (std::uint64_t i = 0; i < 20; ++i) ring.push(make(i));  // 12 more than capacity
  std::vector<std::uint64_t> seqs;
  const auto r = ring.drain([&](const SensorSample& s) { seqs.push_back(s.seq); });
  EXPECT_EQ(r.lost, 12u) << "every overwritten sample must be counted";
  EXPECT_EQ(r.delivered, 8u);
  ASSERT_EQ(seqs.size(), 8u);
  EXPECT_EQ(seqs.front(), 12u);  // oldest surviving
  EXPECT_EQ(seqs.back(), 19u);   // newest is never lost
}

TEST(SeqlockRing, OverrunAcrossMultipleDrainsAccumulates) {
  SeqlockRing<SensorSample, 4> ring;
  std::uint64_t lost = 0, delivered = 0;
  std::uint64_t seq = 0;
  for (int round = 0; round < 10; ++round) {
    for (int i = 0; i < 7; ++i) ring.push(make(seq++));  // 3 too many each round
    const auto r = ring.drain([](const SensorSample&) {});
    lost += r.lost;
    delivered += r.delivered;
  }
  EXPECT_EQ(lost, 30u);
  EXPECT_EQ(delivered + lost, seq) << "no sample may disappear uncounted";
}

// Real concurrency: a producer hammering the ring while a consumer drains.
// Invariants: sequence numbers strictly increase, payload is never torn, and
// delivered + lost == pushed. Run this under TSan to check for data races.
TEST(SeqlockRing, ConcurrentProducerConsumerNeverTearsOrLosesCount) {
  auto ring = std::make_unique<SeqlockRing<SensorSample, 16>>();
  constexpr std::uint64_t kCount = 300'000;
  std::atomic<bool> done{false};

  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kCount; ++i) ring->push(make(i));
    done.store(true, std::memory_order_release);
  });

  std::uint64_t delivered = 0, lost = 0, last_seq = 0, torn = 0, out_of_order = 0;
  bool first = true;
  auto check = [&](const SensorSample& s) {
    if (s.temperature_c != static_cast<double>(s.seq) || s.pressure_torr != 2.0 * static_cast<double>(s.seq)) ++torn;
    if (!first && s.seq <= last_seq) ++out_of_order;
    first = false;
    last_seq = s.seq;
  };
  while (!done.load(std::memory_order_acquire)) {
    const auto r = ring->drain(check);
    delivered += r.delivered;
    lost += r.lost;
  }
  const auto r = ring->drain(check);
  delivered += r.delivered;
  lost += r.lost;
  producer.join();

  EXPECT_EQ(torn, 0u);
  EXPECT_EQ(out_of_order, 0u);
  EXPECT_EQ(delivered + lost, kCount);
  EXPECT_EQ(last_seq, kCount - 1) << "the newest sample must always be delivered";
}

TEST(SeqlockRing, ProductionRingHoldsAbout13msOfSamples) {
  // 64 slots at 5 kHz: the control thread may be late by ~12 ticks before any loss.
  EXPECT_EQ(ptc::io::SensorRing::kCapacity, 64u);
}
