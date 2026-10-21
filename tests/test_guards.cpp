#include <gtest/gtest.h>

#include "recipe/guards.hpp"

using namespace ptc;
using namespace ptc::recipe::guards;

namespace {
SensorSample s(double t, double p, double v = 0.0) {
  SensorSample x;
  x.temperature_c = t;
  x.pressure_torr = p;
  x.valve_position = v;
  return x;
}
}  // namespace

// FR-2: PumpDown -> RampUp only when pressure < 1e-3 Torr.
TEST(Guards, PumpdownRequiresStrictlyBelowTarget) {
  EXPECT_FALSE(pumpdown_complete(s(25, 760), 1e-3));
  EXPECT_FALSE(pumpdown_complete(s(25, 1e-3), 1e-3)) << "equal is not below";
  EXPECT_TRUE(pumpdown_complete(s(25, 0.99e-3), 1e-3));
}

TEST(Guards, InBandIsInclusiveOnBothSides) {
  EXPECT_TRUE(in_band(s(352.0, 0), 350, 2));
  EXPECT_TRUE(in_band(s(348.0, 0), 350, 2));
  EXPECT_FALSE(in_band(s(352.01, 0), 350, 2));
  EXPECT_FALSE(in_band(s(347.99, 0), 350, 2));
}

// FR-2: RampUp -> Process only after 500 CONSECUTIVE ms within +/-2 C.
TEST(Guards, SettleRequires500ConsecutiveMs) {
  std::int64_t acc = 0;
  for (int ms = 0; ms < 499; ++ms) acc = update_in_band(acc, s(350.5, 0), 350, 2, kNsPerMs);
  EXPECT_FALSE(settled(acc, 500 * kNsPerMs));
  acc = update_in_band(acc, s(350.5, 0), 350, 2, kNsPerMs);
  EXPECT_TRUE(settled(acc, 500 * kNsPerMs));
}

TEST(Guards, ExcursionResetsTheSettleTimer) {
  std::int64_t acc = 0;
  for (int ms = 0; ms < 450; ++ms) acc = update_in_band(acc, s(350, 0), 350, 2, kNsPerMs);
  acc = update_in_band(acc, s(353, 0), 350, 2, kNsPerMs);  // one sample out of band
  EXPECT_EQ(acc, 0);
  for (int ms = 0; ms < 499; ++ms) acc = update_in_band(acc, s(350, 0), 350, 2, kNsPerMs);
  EXPECT_FALSE(settled(acc, 500 * kNsPerMs)) << "the 450 ms before the excursion must not count";
}

TEST(Guards, CooledVentedAndValvePositions) {
  EXPECT_TRUE(cooled(s(149.9, 0), 150));
  EXPECT_FALSE(cooled(s(150.0, 0), 150));
  EXPECT_TRUE(vented(s(25, 750), 750));
  EXPECT_FALSE(vented(s(25, 749), 750));
  EXPECT_TRUE(valve_fully_open(s(0, 0, 0.99)));
  EXPECT_FALSE(valve_fully_open(s(0, 0, 0.9)));
  EXPECT_TRUE(valve_fully_closed(s(0, 0, 0.01)));
  EXPECT_FALSE(valve_fully_closed(s(0, 0, 0.1)));
}

// Guards are constexpr pure functions: they can even be checked at compile time.
static_assert(pumpdown_complete(SensorSample{0, 0, 25.0, 5e-4, 0.0}, 1e-3));
static_assert(!in_band(SensorSample{0, 0, 360.0, 0.0, 0.0}, 350.0, 2.0));
