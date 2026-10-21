#include <gtest/gtest.h>

#include "control/pid.hpp"
#include "sim/plant.hpp"

using ptc::control::Pid;
using ptc::control::PidGains;

TEST(Pid, OutputIsClamped) {
  Pid pid;
  EXPECT_DOUBLE_EQ(pid.update(1000.0, 0.0, 1e-3), 1.0);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.update(0.0, 1000.0, 1e-3), 0.0);
}

TEST(Pid, IntegratorDoesNotWindUpWhileSaturated) {
  PidGains g;
  g.kd = 0.0;
  Pid pid(g);
  // 10 s pinned at 100 % with a large positive error (a long ramp).
  for (int i = 0; i < 10'000; ++i) pid.update(350.0, 25.0, 1e-3);
  EXPECT_DOUBLE_EQ(pid.integral(), 0.0) << "conditional integration must freeze the integrator";
}

TEST(Pid, NaiveIntegratorWouldHaveWoundUp) {
  // Same scenario, showing what the anti-windup prevents: ki * error * time.
  const double naive = PidGains{}.ki * (350.0 - 25.0) * 10.0;
  EXPECT_GT(naive, 60.0) << "60x full-scale output would have to unwind";
}

TEST(Pid, IntegratorStillWorksInsideTheLinearRange) {
  PidGains g;
  g.kd = 0.0;
  Pid pid(g);
  for (int i = 0; i < 1000; ++i) pid.update(100.0, 99.0, 1e-3);  // small steady error
  EXPECT_NEAR(pid.integral(), g.ki * 1.0 * 1.0, 1e-9);
}

TEST(Pid, FeedforwardAloneHoldsAPerfectModel) {
  Pid pid;
  const double ff = 0.42;
  EXPECT_NEAR(pid.update(300.0, 300.0, 1e-3, ff), ff, 1e-12);
}

TEST(Pid, ClosedLoopOnPlantSettlesWithoutOvershoot) {
  ptc::sim::PlantParams pp;
  pp.temp_noise_c = 0.0;
  ptc::sim::Plant plant(pp);
  Pid pid;
  ptc::ActuatorCommand c;
  c.heater_contactor = true;
  double max_t = 0.0;
  for (int i = 0; i < 8000; ++i) {  // 8 s at 1 kHz
    const double ff = (350.0 - 25.0) / 570.0;  // deliberately mismatched model
    c.heater = pid.update(350.0, plant.state().temperature_c, 1e-3, ff);
    plant.step(1e-3, c);
    max_t = std::max(max_t, plant.state().temperature_c);
  }
  EXPECT_NEAR(plant.state().temperature_c, 350.0, 0.5);
  EXPECT_LT(max_t, 352.0) << "overshoot must stay inside the +/-2 C process band";
}
