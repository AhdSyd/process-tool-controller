#include <gtest/gtest.h>

#include "recipe/recipe_machine.hpp"

using namespace ptc;
using ptc::recipe::RecipeMachine;
using ptc::recipe::RecipeParams;
using ptc::recipe::Transition;

namespace {

// Drives the state machine with hand-written sensor values: no plant, no
// threads. Lets each transition be tested exactly at its boundary.
struct Driver {
  RecipeMachine m{RecipeParams{}};
  std::int64_t now = 1'000 * kNsPerSec;
  SensorSample s{0, 0, 25.0, 760.0, 0.0};
  bool trips = false;

  Transition tick(std::int64_t ms = 1) {
    Transition last;
    for (std::int64_t i = 0; i < ms; ++i) {
      now += kNsPerMs;
      s.t_ns = now;
      const Transition t = m.tick({s, now, kNsPerMs, trips});
      if (t.happened) last = t;
    }
    return last;
  }
  bool tick_until(State st, std::int64_t max_ms) {
    for (std::int64_t i = 0; i < max_ms && m.state() != st; ++i) tick();
    return m.state() == st;
  }
  // Mimic the slit valve following its command instantly.
  void follow_valve() { s.valve_position = m.outputs().valve_open ? 1.0 : 0.0; }
  void run_transfer() {
    for (int i = 0; i < 2000 && (m.state() == State::Load || m.state() == State::Unload); ++i) {
      follow_valve();
      tick();
    }
  }
};

}  // namespace

TEST(RecipeMachine, IdleWaitsForWafers) {
  Driver d;
  d.tick(100);
  EXPECT_EQ(d.m.state(), State::Idle);
  d.m.request_wafers(2);
  const Transition t = d.tick();
  EXPECT_TRUE(t.happened);
  EXPECT_EQ(t.to, State::Load);
  EXPECT_EQ(d.m.wafers_pending(), 1u);
}

TEST(RecipeMachine, IdleDoesNotStartWhileAnInterlockIsActive) {
  Driver d;
  d.trips = true;
  d.m.request_wafers(1);
  d.tick(10);
  EXPECT_EQ(d.m.state(), State::Idle);
}

TEST(RecipeMachine, FullNominalSequence) {
  Driver d;
  d.m.request_wafers(1);
  d.tick();
  ASSERT_EQ(d.m.state(), State::Load);
  EXPECT_TRUE(d.m.outputs().valve_open);

  d.run_transfer();
  ASSERT_EQ(d.m.state(), State::PumpDown);
  EXPECT_TRUE(d.m.outputs().pump);
  EXPECT_FALSE(d.m.outputs().valve_open);

  d.s.pressure_torr = 5e-4;
  d.tick();
  ASSERT_EQ(d.m.state(), State::RampUp);
  EXPECT_TRUE(d.m.outputs().pid_enabled);
  EXPECT_DOUBLE_EQ(d.m.outputs().setpoint_c, 350.0);

  d.s.temperature_c = 350.5;
  d.tick(499);
  EXPECT_EQ(d.m.state(), State::RampUp) << "499 ms in band is not enough";
  d.tick();
  ASSERT_EQ(d.m.state(), State::Process);

  d.tick(2999);
  EXPECT_EQ(d.m.state(), State::Process);
  d.tick();
  ASSERT_EQ(d.m.state(), State::RampDown);
  EXPECT_FALSE(d.m.outputs().pid_enabled) << "heater off while cooling";

  d.s.temperature_c = 149.0;
  d.tick();
  ASSERT_EQ(d.m.state(), State::Vent);
  EXPECT_TRUE(d.m.outputs().vent);
  EXPECT_FALSE(d.m.outputs().pump);

  d.s.pressure_torr = 755.0;
  d.tick();
  ASSERT_EQ(d.m.state(), State::Unload);

  d.run_transfer();
  EXPECT_EQ(d.m.state(), State::Idle);
  EXPECT_EQ(d.m.wafers_done(), 1u);
  EXPECT_GT(d.m.last_cycle_ns(), 0);
}

TEST(RecipeMachine, LoadWaitsForValveToReallyClose) {
  Driver d;
  d.m.request_wafers(1);
  d.tick();
  d.s.valve_position = 1.0;
  d.tick(600);  // dwell elapsed, valve commanded closed...
  EXPECT_FALSE(d.m.outputs().valve_open);
  d.tick(10);   // ...but the sensor still reads open
  EXPECT_EQ(d.m.state(), State::Load);
  d.s.valve_position = 0.0;
  d.tick();
  EXPECT_EQ(d.m.state(), State::PumpDown);
}

// FR-3: exceeding a state's maximum duration -> Fault with a reason code.
TEST(RecipeMachine, EveryTimedStateFaultsOnTimeout) {
  const RecipeParams p;
  for (State st : {State::Load, State::PumpDown, State::RampUp, State::Process, State::RampDown, State::Vent,
                   State::Unload}) {
    const std::int64_t limit = RecipeMachine::timeout_ns(st, p);
    ASSERT_GT(limit, 0) << state_name(st);
  }
  Driver d;  // PumpDown that never reaches vacuum
  d.m.request_wafers(1);
  d.tick();
  d.run_transfer();
  ASSERT_EQ(d.m.state(), State::PumpDown);
  d.tick(static_cast<std::int64_t>(p.pumpdown_timeout_ns / kNsPerMs));
  EXPECT_EQ(d.m.state(), State::PumpDown) << "exactly at the limit is still allowed";
  const Transition t = d.tick();
  EXPECT_EQ(t.to, State::Fault);
  EXPECT_EQ(t.from, State::PumpDown);
  EXPECT_EQ(t.reason, Reason::StateTimeout);
  EXPECT_FALSE(d.m.outputs().heater_contactor);
}

TEST(RecipeMachine, IdleAndFaultNeverTimeOut) {
  EXPECT_EQ(RecipeMachine::timeout_ns(State::Idle, RecipeParams{}), 0);
  EXPECT_EQ(RecipeMachine::timeout_ns(State::Fault, RecipeParams{}), 0);
}

// FR-5: abort turns the heater off in the same call, then vents, then Idle.
TEST(RecipeMachine, AbortIsImmediateThenVentsThenIdle) {
  Driver d;
  d.m.request_wafers(3);
  d.tick();
  d.run_transfer();
  d.s.pressure_torr = 5e-4;
  d.tick();
  ASSERT_EQ(d.m.state(), State::RampUp);
  ASSERT_TRUE(d.m.outputs().pid_enabled);

  const Transition t = d.m.force(State::Aborting, Reason::OverTemperature, d.now);
  EXPECT_TRUE(t.happened);
  EXPECT_FALSE(d.m.outputs().pid_enabled) << "heater off immediately";
  EXPECT_TRUE(d.m.outputs().heater_contactor) << "abort is the controlled path; contactor stays";
  EXPECT_TRUE(d.m.outputs().vent);
  EXPECT_FALSE(d.m.outputs().pump);
  EXPECT_FALSE(d.m.outputs().valve_open);
  EXPECT_EQ(d.m.wafers_pending(), 0u) << "abort cancels the remaining run";
  EXPECT_EQ(d.m.reason(), Reason::OverTemperature);

  d.tick(100);
  EXPECT_EQ(d.m.state(), State::Aborting) << "not vented yet";
  d.s.pressure_torr = 760.0;
  d.trips = true;
  d.tick(10);
  EXPECT_EQ(d.m.state(), State::Aborting) << "vented, but an interlock is still active";
  d.trips = false;
  d.tick();
  EXPECT_EQ(d.m.state(), State::Idle);
}

TEST(RecipeMachine, AbortThatCannotCompleteEscalatesToFault) {
  Driver d;
  d.s.pressure_torr = 1e-4;  // vent never works
  d.m.force(State::Aborting, Reason::OperatorAbort, d.now);
  const Transition t = d.tick(static_cast<std::int64_t>(RecipeParams{}.abort_timeout_ns / kNsPerMs) + 1);
  EXPECT_EQ(t.to, State::Fault);
  EXPECT_EQ(t.reason, Reason::AbortTimeout);
}

TEST(RecipeMachine, FaultIsLatchedUntilResetThenVentsToIdle) {
  Driver d;
  d.s.pressure_torr = 1e-4;
  d.m.force(State::Fault, Reason::StaleSensorData, d.now);
  d.tick(1000);
  EXPECT_EQ(d.m.state(), State::Fault) << "latched";

  d.trips = true;
  d.m.request_reset();
  d.tick();
  EXPECT_EQ(d.m.state(), State::Fault) << "reset refused while the cause is still present";

  d.trips = false;
  d.m.request_reset();
  d.tick();
  EXPECT_EQ(d.m.state(), State::Aborting) << "reset re-runs the vent sequence";
  EXPECT_EQ(d.m.reason(), Reason::StaleSensorData) << "original reason kept";
  d.s.pressure_torr = 760.0;
  d.tick();
  EXPECT_EQ(d.m.state(), State::Idle);
}

TEST(RecipeMachine, ResetOutsideFaultIsIgnored) {
  Driver d;
  d.m.request_reset();
  d.tick();
  d.m.force(State::Fault, Reason::StateTimeout, d.now);
  d.tick();
  EXPECT_EQ(d.m.state(), State::Fault) << "a stale reset request must not clear a new fault";
}

TEST(RecipeMachine, ForceToCurrentStateIsANoOp) {
  Driver d;
  d.m.force(State::Aborting, Reason::OperatorAbort, d.now);
  EXPECT_FALSE(d.m.force(State::Aborting, Reason::OverTemperature, d.now).happened);
  EXPECT_EQ(d.m.reason(), Reason::OperatorAbort);
}
