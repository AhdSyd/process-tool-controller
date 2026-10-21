// Closed-loop system tests: real Controller + Plant + ring buffer, simulated
// time. These are the fault-injection proofs that every interlock fires and
// every abort path ends in a safe state (acceptance criteria 3 and 6).

#include <gtest/gtest.h>

#include "safety/interlocks.hpp"
#include "sim_harness.hpp"

using namespace ptc;
using ptc::safety::Interlock;
using ptc::telemetry::LogEvent;
using ptc::test::make_harness;

namespace {

std::uint32_t trips(const test::SimHarness& h, Interlock i) {
  return h.ctl.metrics().interlock_trips[static_cast<std::size_t>(i)];
}

// Start one wafer and inject `f` 500 ms into Process.
std::unique_ptr<test::SimHarness> run_into_fault(FaultType f) {
  auto h = make_harness();
  h->start_wafers(1);
  EXPECT_TRUE(h->run_until_state(State::Process, 15'000));
  h->run_ms(500);
  if (f == FaultType::SensorDropout) h->sensor_dropout = true;
  else h->plant.set_fault(f);
  return h;
}

}  // namespace

TEST(System, NominalRunProcessesWafersWithinSpec) {
  auto h = make_harness();
  h->start_wafers(3);
  ASSERT_TRUE(h->run_until([&] { return h->ctl.machine().wafers_done() == 3 && h->state() == State::Idle; },
                           60'000));
  const auto& m = h->ctl.metrics();
  EXPECT_EQ(m.wafers_processed, 3u);
  EXPECT_EQ(m.aborts, 0u);
  EXPECT_EQ(m.faults, 0u);
  EXPECT_EQ(m.buffer_overruns, 0u);
  EXPECT_LT(m.process_max_dev_c, 2.0) << "temperature held inside +/-2 C during Process";
  EXPECT_LT(m.rampup_overshoot_c, 2.0);
  EXPECT_NEAR(static_cast<double>(m.last_cycle_ns) / 1e9, 13.0, 1.5);
  for (std::size_t i = 0; i < safety::kInterlockCount; ++i) EXPECT_EQ(m.interlock_trips[i], 0u);
}

// ---- One test per interlock, each driven by a physical fault ---------------

TEST(System, TemperatureRunawayAbortsThenEscalatesToFault) {
  auto h = run_into_fault(FaultType::TemperatureRunaway);
  ASSERT_TRUE(h->run_until_state(State::Aborting, 3'000));
  EXPECT_EQ(h->ctl.machine().reason(), Reason::OverTemperature);
  EXPECT_DOUBLE_EQ(h->port.read().heater, 0.0) << "abort commands the heater off...";
  // ...but the welded SSR ignores it and temperature keeps climbing:
  ASSERT_TRUE(h->run_until_state(State::Fault, 3'000));
  EXPECT_EQ(h->ctl.machine().reason(), Reason::CriticalOverTemperature);
  EXPECT_FALSE(h->port.read().heater_contactor) << "Fault opens the contactor: the layer that works";

  const double t_at_fault = h->plant.state().temperature_c;
  h->run_ms(3'000);
  EXPECT_LT(h->plant.state().temperature_c, t_at_fault - 50.0) << "chamber cooling after contactor opened";
  EXPECT_LT(h->plant.state().temperature_c, 550.0);

  const auto& m = h->ctl.metrics();
  EXPECT_EQ(trips(*h, Interlock::OverTemperature), 1u);
  EXPECT_EQ(trips(*h, Interlock::CriticalOverTemperature), 1u);
  ASSERT_EQ(m.safety_event_count, 1u);
  EXPECT_TRUE(m.safety_events[0].escalated) << "the abort path was interrupted by a more severe fault";
  EXPECT_GE(m.safety_events[0].safe_ns, 0);
}

TEST(System, VacuumLeakWithHeaterOnAbortsAndVentsToIdle) {
  auto h = run_into_fault(FaultType::VacuumLeak);
  bool heater_off_same_tick = false;
  h->on_log = [&](const telemetry::LogRecord& r) {
    if (r.event == LogEvent::InterlockTrip && r.a == static_cast<std::uint8_t>(Interlock::OverPressureHeaterOn))
      heater_off_same_tick = h->port.read().heater == 0.0;
  };
  ASSERT_TRUE(h->run_until_state(State::Aborting, 3'000));
  EXPECT_EQ(h->ctl.machine().reason(), Reason::OverPressureHeaterOn);
  EXPECT_TRUE(heater_off_same_tick) << "heater must be off in the tick the trip is detected";
  ASSERT_TRUE(h->run_until_state(State::Idle, 6'000));
  // Guard is on the (noisy, +/-1 %) gauge reading; true pressure is near atmosphere.
  EXPECT_GT(h->plant.state().pressure_torr, 720.0);
  EXPECT_EQ(trips(*h, Interlock::OverPressureHeaterOn), 1u);
  EXPECT_EQ(h->ctl.metrics().safety_events[0].from, State::Process);
  EXPECT_GT(h->ctl.metrics().safety_events[0].safe_ns, 0);
}

TEST(System, SlitValveFailingOpenUnderVacuumAborts) {
  auto h = run_into_fault(FaultType::StuckValve);
  ASSERT_TRUE(h->run_until_state(State::Aborting, 1'000));
  EXPECT_EQ(h->ctl.machine().reason(), Reason::SlitValveOpenUnderVacuum);
  EXPECT_FALSE(h->port.read().valve_open) << "controller commands it closed (actuator ignores it)";
  ASSERT_TRUE(h->run_until_state(State::Idle, 6'000));
  EXPECT_EQ(trips(*h, Interlock::SlitValveOpenUnderVacuum), 1u);
}

TEST(System, SensorDropoutFaultsWithinElevenMs) {
  auto h = run_into_fault(FaultType::SensorDropout);
  const std::int64_t t_drop = h->now;
  ASSERT_TRUE(h->run_until_state(State::Fault, 100));
  const std::int64_t elapsed_ms = (h->now - t_drop) / kNsPerMs;
  EXPECT_GE(elapsed_ms, 10) << "not before the 10 ms limit";
  EXPECT_LE(elapsed_ms, 12) << "10 ms limit + 1 tick debounce + 1 tick";
  EXPECT_EQ(h->ctl.machine().reason(), Reason::StaleSensorData);
  EXPECT_FALSE(h->port.read().heater_contactor);
  EXPECT_FALSE(h->port.read().pump);
  EXPECT_EQ(trips(*h, Interlock::StaleSensorData), 1u);

  h->mailbox.reset.store(true);
  h->run_ms(100);
  EXPECT_EQ(h->state(), State::Fault) << "reset refused while the sensor is still dead";
  h->sensor_dropout = false;
  h->mailbox.reset.store(true);
  ASSERT_TRUE(h->run_until_state(State::Idle, 6'000)) << "sensor back + reset: vent, then Idle";
}

// ---- Timeouts, operator abort, overruns ------------------------------------

TEST(System, PumpThatCannotReachVacuumTimesOutToFault) {
  auto h = make_harness();
  h->plant.set_fault(FaultType::VacuumLeak);  // leak from the start: pump-down stalls ~57 Torr
  h->start_wafers(1);
  ASSERT_TRUE(h->run_until_state(State::Fault, 15'000));
  EXPECT_EQ(h->ctl.machine().reason(), Reason::StateTimeout);
  EXPECT_EQ(h->ctl.metrics().safety_events[0].from, State::PumpDown);
}

TEST(System, OperatorAbortMidProcess) {
  auto h = make_harness();
  h->start_wafers(5);
  ASSERT_TRUE(h->run_until_state(State::Process, 15'000));
  h->mailbox.abort.store(true);
  h->run_ms(1);
  EXPECT_EQ(h->state(), State::Aborting);
  EXPECT_EQ(h->ctl.machine().reason(), Reason::OperatorAbort);
  ASSERT_TRUE(h->run_until_state(State::Idle, 6'000));
  h->run_ms(500);
  EXPECT_EQ(h->state(), State::Idle) << "remaining 4 wafers cancelled";
}

TEST(System, StalledControlThreadOverrunsAreCountedExactly) {
  auto h = make_harness();
  h->control_stall_ticks = 20;  // 20 ms without a control tick: 100 samples arrive
  h->run_ms(21);                // 20 stalled ticks + 1 that drains (5 more samples)
  // 105 samples published, ring holds 64: exactly 41 overwritten and counted.
  EXPECT_EQ(h->ctl.metrics().buffer_overruns, 41u);
  EXPECT_FALSE(h->ctl.active_trips().any()) << "data is fresh on resume: no stale trip";
}

TEST(System, InterlocksFireRegardlessOfRecipeState) {
  // Over-temperature while Idle (no recipe running) still trips and aborts.
  auto h = make_harness();
  h->plant.set_state({460.0, 760.0, 0.0, 0.0});
  h->run_ms(2);
  EXPECT_EQ(trips(*h, Interlock::OverTemperature), 1u);
  EXPECT_EQ(h->state(), State::Aborting);
}
