#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>

#include "telemetry/spsc_queue.hpp"

using ptc::telemetry::SpscQueue;

TEST(SpscQueue, FifoOrder) {
  SpscQueue<int, 8> q;
  for (int i = 0; i < 5; ++i) EXPECT_TRUE(q.try_push(i));
  int v = -1;
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(q.try_pop(v));
    EXPECT_EQ(v, i);
  }
  EXPECT_FALSE(q.try_pop(v));
}

TEST(SpscQueue, FullDropsNewestAndCounts) {
  SpscQueue<int, 4> q;
  for (int i = 0; i < 4; ++i) EXPECT_TRUE(q.try_push(i));
  EXPECT_FALSE(q.try_push(99));  // never blocks, never overwrites
  EXPECT_FALSE(q.try_push(100));
  EXPECT_EQ(q.dropped(), 2u);
  int v;
  ASSERT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 0) << "oldest entries are preserved: a log must not rewrite history";
  EXPECT_TRUE(q.try_push(4));  // space again after a pop
}

TEST(SpscQueue, WrapsAroundManyTimes) {
  SpscQueue<int, 4> q;
  int v;
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(q.try_push(i));
    ASSERT_TRUE(q.try_pop(v));
    ASSERT_EQ(v, i);
  }
}

TEST(SpscQueue, ConcurrentTransferIsLosslessAndOrdered) {
  auto q = std::make_unique<SpscQueue<std::uint64_t, 64>>();
  constexpr std::uint64_t kCount = 500'000;
  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kCount; ++i) {
      while (!q->try_push(i)) std::this_thread::yield();  // test only: retry instead of drop
    }
  });
  std::uint64_t expected = 0, v = 0;
  while (expected < kCount) {
    if (q->try_pop(v)) {
      ASSERT_EQ(v, expected);
      ++expected;
    }
  }
  producer.join();
}
