#include "safety/interlocks.hpp"

namespace ptc::safety {

TripSet evaluate(const InterlockInput& in, const InterlockLimits& l) noexcept {
  TripSet t;
  if (sensor_data_stale(in.latest, in.now_ns, l)) t.set(Interlock::StaleSensorData);
  if (in.latest == nullptr) return t;  // nothing else can be judged without data

  const SensorSample& s = *in.latest;
  if (over_temperature(s, l)) t.set(Interlock::OverTemperature);
  if (critical_over_temperature(s, l)) t.set(Interlock::CriticalOverTemperature);
  if (over_pressure_heater_on(s, in.heater_on, l)) t.set(Interlock::OverPressureHeaterOn);
  if (slit_valve_open_under_vacuum(s, l)) t.set(Interlock::SlitValveOpenUnderVacuum);
  return t;
}

TripSet InterlockMonitor::update(const InterlockInput& in) noexcept {
  const TripSet trip = evaluate(in, limits_);   // crosses the trip limit now
  const TripSet hold = evaluate(in, clear_);    // still outside the clear limit

  if (trip.test(Interlock::StaleSensorData)) {
    if (stale_count_ < limits_.stale_debounce_ticks) ++stale_count_;
  } else {
    stale_count_ = 0;
  }
  const bool stale_confirmed = stale_count_ >= limits_.stale_debounce_ticks || in.latest == nullptr;

  TripSet next;
  for (std::size_t i = 0; i < kInterlockCount; ++i) {
    const auto il = static_cast<Interlock>(i);
    if (il == Interlock::StaleSensorData) {
      if (stale_confirmed) next.set(il);  // staleness is binary: no hysteresis
    } else if (trip.test(il) || (active_.test(il) && hold.test(il))) {
      next.set(il);
    }
  }
  active_ = next;
  return next;
}

Action decide(State current, TripSet trips) noexcept {
  if (!trips.any() || current == State::Fault) return {};
  const Interlock worst = trips.most_severe();
  if (severity(worst) == Severity::Fault) return {Action::Kind::Fault, reason_for(worst)};
  if (current == State::Aborting) return {};
  return {Action::Kind::Abort, reason_for(worst)};
}

}  // namespace ptc::safety
