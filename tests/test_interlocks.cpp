#include <gtest/gtest.h>

#include "safety/interlocks.hpp"

using namespace ptc;
using namespace ptc::safety;

namespace {
const InterlockLimits L{};
SensorSample s(double t, double p, double v = 0.0, std::int64_t t_ns = 0) {
  SensorSample x;
  x.t_ns = t_ns;
  x.temperature_c = t;
  x.pressure_torr = p;
  x.valve_position = v;
  return x;
}
}  // namespace

// ---- Each predicate, at its boundary --------------------------------------
TEST(Interlocks, OverTemperatureAbove450) {
  EXPECT_FALSE(over_temperature(s(450.0, 1e-4), L));
  EXPECT_TRUE(over_temperature(s(450.01, 1e-4), L));
}

TEST(Interlocks, CriticalOverTemperatureAbove500) {
  EXPECT_FALSE(critical_over_temperature(s(500.0, 1e-4), L));
  EXPECT_TRUE(critical_over_temperature(s(500.01, 1e-4), L));
}

TEST(Interlocks, OverPressureOnlyMattersWithHeaterOn) {
  EXPECT_TRUE(over_pressure_heater_on(s(300, 10.5), true, L));
  EXPECT_FALSE(over_pressure_heater_on(s(300, 10.5), false, L)) << "venting with heater off is normal";
  EXPECT_FALSE(over_pressure_heater_on(s(300, 10.0), true, L));
}

TEST(Interlocks, SlitValveOpenUnderVacuum) {
  EXPECT_TRUE(slit_valve_open_under_vacuum(s(25, 0.5, 0.2), L));
  EXPECT_FALSE(slit_valve_open_under_vacuum(s(25, 760, 1.0), L)) << "open at atmosphere is how wafers load";
  EXPECT_FALSE(slit_valve_open_under_vacuum(s(25, 0.5, 0.04), L)) << "below the 5 % open threshold";
  EXPECT_FALSE(slit_valve_open_under_vacuum(s(25, 1.0, 1.0), L)) << "exactly 1 Torr is not below 1 Torr";
}

TEST(Interlocks, StaleAfterTenMilliseconds) {
  const SensorSample x = s(25, 760, 0, 0);
  EXPECT_FALSE(sensor_data_stale(&x, 10 * kNsPerMs, L));
  EXPECT_TRUE(sensor_data_stale(&x, 10 * kNsPerMs + 1, L));
  EXPECT_TRUE(sensor_data_stale(nullptr, 0, L)) << "no data at all is the stalest data";
}

// ---- evaluate(): combinations, and independence from recipe state ----------
TEST(Interlocks, EvaluateHealthyIsEmpty) {
  const SensorSample x = s(350, 1e-4, 0.0, 0);
  EXPECT_FALSE(evaluate({&x, kNsPerMs, true}, L).any());
}

TEST(Interlocks, EvaluateReportsMultipleTrips) {
  const SensorSample x = s(510, 20, 0.0, 0);
  const TripSet t = evaluate({&x, kNsPerMs, true}, L);
  EXPECT_TRUE(t.test(Interlock::OverTemperature));
  EXPECT_TRUE(t.test(Interlock::CriticalOverTemperature));
  EXPECT_TRUE(t.test(Interlock::OverPressureHeaterOn));
  EXPECT_FALSE(t.test(Interlock::StaleSensorData));
  EXPECT_EQ(t.most_severe(), Interlock::CriticalOverTemperature);
}

// ---- Stale debounce: one late tick is not a dead sensor --------------------
TEST(Interlocks, StaleIsDebouncedByOneTick) {
  InterlockMonitor mon;
  const SensorSample x = s(25, 760, 0, 0);
  EXPECT_FALSE(mon.update({&x, 5 * kNsPerMs, false}).any());
  EXPECT_FALSE(mon.update({&x, 11 * kNsPerMs, false}).test(Interlock::StaleSensorData)) << "first stale tick";
  EXPECT_TRUE(mon.update({&x, 12 * kNsPerMs, false}).test(Interlock::StaleSensorData)) << "second: trip";
}

TEST(Interlocks, FreshDataResetsStaleDebounce) {
  InterlockMonitor mon;
  SensorSample x = s(25, 760, 0, 0);
  mon.update({&x, 11 * kNsPerMs, false});  // stale once (scheduling hiccup)
  x.t_ns = 11 * kNsPerMs;                  // data resumes
  EXPECT_FALSE(mon.update({&x, 12 * kNsPerMs, false}).any());
  EXPECT_FALSE(mon.update({&x, 22 * kNsPerMs, false}).any()) << "counter restarted from zero";
}

TEST(Interlocks, NonStaleTripsAreNotDebounced) {
  InterlockMonitor mon;
  const SensorSample x = s(451, 1e-4, 0, 0);
  EXPECT_TRUE(mon.update({&x, kNsPerMs, true}).test(Interlock::OverTemperature)) << "same tick";
}

TEST(Interlocks, HysteresisPreventsChatterAroundTheLimit) {
  InterlockMonitor mon;
  SensorSample x = s(450.5, 1e-4, 0, 0);
  EXPECT_TRUE(mon.update({&x, kNsPerMs, false}).test(Interlock::OverTemperature));
  x.temperature_c = 449.5;  // noise dips just under the limit
  EXPECT_TRUE(mon.update({&x, kNsPerMs, false}).test(Interlock::OverTemperature)) << "held: within 5 C";
  x.temperature_c = 444.9;  // genuinely back below limit - hysteresis
  EXPECT_FALSE(mon.update({&x, kNsPerMs, false}).test(Interlock::OverTemperature)) << "cleared";
  x.temperature_c = 449.5;  // below the TRIP limit: must not re-trip
  EXPECT_FALSE(mon.update({&x, kNsPerMs, false}).test(Interlock::OverTemperature));
}

TEST(Interlocks, NoisySignalAtTheLimitTripsExactlyOnce) {
  InterlockMonitor mon;
  int rising_edges = 0;
  bool prev = false;
  for (int i = 0; i < 1000; ++i) {
    const SensorSample x = s(450.0 + ((i % 2) ? 0.3 : -0.3), 1e-4, 0, 0);  // alternating +/-0.3 C
    const bool now = mon.update({&x, kNsPerMs, false}).test(Interlock::OverTemperature);
    if (now && !prev) ++rising_edges;
    prev = now;
  }
  EXPECT_EQ(rising_edges, 1);
}

// ---- decide(): the abort-sequencing policy ---------------------------------
TEST(AbortPolicy, NoTripsNoAction) {
  EXPECT_EQ(decide(State::Process, TripSet{}).kind, Action::Kind::None);
}

TEST(AbortPolicy, AbortSeverityAbortsFromEveryNormalState) {
  TripSet t;
  t.set(Interlock::OverTemperature);
  for (State st : {State::Idle, State::Load, State::PumpDown, State::RampUp, State::Process, State::RampDown,
                   State::Vent, State::Unload}) {
    const Action a = decide(st, t);
    EXPECT_EQ(a.kind, Action::Kind::Abort) << state_name(st);
    EXPECT_EQ(a.reason, Reason::OverTemperature);
  }
}

TEST(AbortPolicy, AbortSeverityDoesNotRestartAnAbortInProgress) {
  TripSet t;
  t.set(Interlock::OverPressureHeaterOn);
  EXPECT_EQ(decide(State::Aborting, t).kind, Action::Kind::None);
}

TEST(AbortPolicy, FaultSeverityInterruptsTheAbortPath) {
  TripSet t;
  t.set(Interlock::OverTemperature);
  t.set(Interlock::CriticalOverTemperature);
  const Action a = decide(State::Aborting, t);
  EXPECT_EQ(a.kind, Action::Kind::Fault);
  EXPECT_EQ(a.reason, Reason::CriticalOverTemperature);
}

TEST(AbortPolicy, StaleDataGoesStraightToFault) {
  TripSet t;
  t.set(Interlock::StaleSensorData);
  EXPECT_EQ(decide(State::Process, t).kind, Action::Kind::Fault);
}

TEST(AbortPolicy, NothingFurtherOnceFaulted) {
  TripSet t;
  t.set(Interlock::CriticalOverTemperature);
  EXPECT_EQ(decide(State::Fault, t).kind, Action::Kind::None);
}
