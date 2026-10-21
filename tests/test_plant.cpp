#include <gtest/gtest.h>

#include <cmath>

#include "sim/plant.hpp"

using ptc::ActuatorCommand;
using ptc::FaultType;
using ptc::sim::Plant;
using ptc::sim::PlantParams;

namespace {
void run(Plant& p, double seconds, const ActuatorCommand& c, double dt = 200e-6) {
  const int n = static_cast<int>(seconds / dt);
  for (int i = 0; i < n; ++i) p.step(dt, c);
}
ActuatorCommand heater(double duty) {
  ActuatorCommand c;
  c.heater = duty;
  c.heater_contactor = true;
  return c;
}
}  // namespace

TEST(Plant, StartsAtAmbientAndAtmosphere) {
  Plant p;
  EXPECT_DOUBLE_EQ(p.state().temperature_c, 25.0);
  EXPECT_DOUBLE_EQ(p.state().pressure_torr, 760.0);
  EXPECT_DOUBLE_EQ(p.state().valve_position, 0.0);
}

TEST(Plant, ThermalFirstOrderResponse) {
  Plant p;
  run(p, 2.5, heater(0.5));  // one time constant at half power
  const double expected = 25.0 + 300.0 * (1.0 - std::exp(-1.0));
  EXPECT_NEAR(p.state().temperature_c, expected, 0.05);
}

TEST(Plant, ExactIntegrationIsStepSizeIndependent) {
  Plant a, b;
  run(a, 1.0, heater(1.0), 200e-6);
  run(b, 1.0, heater(1.0), 50e-3);  // a stalled thread integrating big steps
  EXPECT_NEAR(a.state().temperature_c, b.state().temperature_c, 1e-6);
}

TEST(Plant, PumpDownReachesTargetInAboutTwoSeconds) {
  Plant p;
  ActuatorCommand c;
  c.pump = true;
  run(p, 1.5, c);
  EXPECT_GT(p.state().pressure_torr, 1e-3);
  run(p, 1.0, c);
  EXPECT_LT(p.state().pressure_torr, 1e-3);
}

TEST(Plant, VentReturnsToAtmosphere) {
  Plant p;
  ActuatorCommand c;
  c.pump = true;
  run(p, 3.0, c);
  c.pump = false;
  c.vent = true;
  run(p, 2.0, c);
  EXPECT_GT(p.state().pressure_torr, 750.0);
}

TEST(Plant, ValveTravelsAtFiniteSpeed) {
  Plant p;
  ActuatorCommand c;
  c.valve_open = true;
  run(p, 0.15, c);
  EXPECT_NEAR(p.state().valve_position, 0.5, 0.01);
  run(p, 0.2, c);
  EXPECT_DOUBLE_EQ(p.state().valve_position, 1.0);
}

TEST(Plant, RunawayIgnoresDutyButNotContactor) {
  Plant p;
  p.set_fault(FaultType::TemperatureRunaway);
  run(p, 1.0, heater(0.0));  // duty 0, but SSR is welded on
  EXPECT_GT(p.state().temperature_c, 200.0);
  ActuatorCommand off;  // contactor open
  const double t = p.state().temperature_c;
  run(p, 1.0, off);
  EXPECT_LT(p.state().temperature_c, t) << "opening the contactor must cut heater power";
}

TEST(Plant, StuckValveOpensRegardlessOfCommand) {
  Plant p;
  p.set_fault(FaultType::StuckValve);
  run(p, 0.5, ActuatorCommand{});
  EXPECT_DOUBLE_EQ(p.state().valve_position, 1.0);
}

TEST(Plant, VacuumLeakOverwhelmsPump) {
  Plant p;
  ActuatorCommand c;
  c.pump = true;
  run(p, 3.0, c);
  p.set_fault(FaultType::VacuumLeak);
  run(p, 2.0, c);
  EXPECT_GT(p.state().pressure_torr, 10.0);
}

TEST(Plant, NoiseIsSeededAndReproducible) {
  PlantParams pp;
  pp.seed = 7;
  Plant a(pp), b(pp);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(a.sample(i, 0).temperature_c, b.sample(i, 0).temperature_c);
  }
}
