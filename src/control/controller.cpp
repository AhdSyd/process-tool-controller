#include "control/controller.hpp"

#include <algorithm>
#include <cmath>

namespace ptc::control {

using safety::Action;
using safety::Interlock;
using safety::TripSet;
using telemetry::LogEvent;
using telemetry::LogRecord;

namespace {
bool recipe_active(State s) noexcept {
  return s != State::Idle && s != State::Aborting && s != State::Fault;
}
}  // namespace

Controller::Controller(const ControllerConfig& cfg, io::SensorRing& ring, io::ActuatorPort& port,
                       CommandMailbox& mailbox, StatusBoard& status, telemetry::LogQueue& log)
    : cfg_(cfg),
      ring_(ring),
      port_(port),
      mailbox_(mailbox),
      status_(status),
      log_(log),
      machine_(cfg.recipe),
      monitor_(cfg.limits),
      pid_(cfg.pid) {}

LogRecord Controller::record(LogEvent ev, std::int64_t now) const noexcept {
  LogRecord r;
  r.t_ns = now;
  r.event = ev;
  r.x = latest_.temperature_c;
  r.y = latest_.pressure_torr;
  r.z = latest_.valve_position;
  return r;
}

void Controller::step(std::int64_t now) noexcept {
  const std::int64_t dt_ns = prev_now_ ? std::clamp<std::int64_t>(now - prev_now_, 0, 50 * kNsPerMs)
                                       : kNsPerMs;
  prev_now_ = now;

  // ---- 1. Sensors ----------------------------------------------------------
  // Drain everything the sensor thread published since last tick (~5 samples).
  // Interlocks and guards use the newest sample; the PID uses the mean of the
  // batch, a cheap 5-tap filter that cuts sensor noise by ~sqrt(5).
  double t_sum = 0.0;
  std::uint32_t n = 0;
  const auto dr = ring_.drain([&](const SensorSample& s) {
    latest_ = s;
    have_sample_ = true;
    t_sum += s.temperature_c;
    ++n;
  });
  m_.samples_consumed += dr.delivered;
  if (dr.lost > 0) {
    m_.buffer_overruns += dr.lost;
    LogRecord r = record(LogEvent::BufferOverrun, now);
    r.u = static_cast<std::uint32_t>(std::min<std::uint64_t>(dr.lost, 0xFFFFFFFFu));
    log(r);
  }
  if (dr.delivered == 0) ++m_.ticks_without_new_sample;
  const double t_meas = n ? t_sum / n : latest_.temperature_c;

  // ---- 2. Operator commands ------------------------------------------------
  if (const std::uint32_t w = mailbox_.wafers.exchange(0, std::memory_order_relaxed)) {
    machine_.request_wafers(w);
    accepted_ += w;
  }
  const bool abort_requested = mailbox_.abort.exchange(false, std::memory_order_relaxed);
  if (mailbox_.reset.exchange(false, std::memory_order_relaxed)) machine_.request_reset();

  // ---- 3. Interlocks: every tick, independent of recipe state --------------
  const bool heater_on = last_cmd_.heater > 0.0 && last_cmd_.heater_contactor;
  const TripSet trips = monitor_.update({have_sample_ ? &latest_ : nullptr, now, heater_on});
  log_trip_edges(trips, now);

  // ---- 4. Arbitration: safety > operator abort > recipe --------------------
  recipe::Transition tr;
  const State before = machine_.state();
  const Action act = safety::decide(before, trips);
  if (act.kind == Action::Kind::Fault) {
    tr = machine_.force(State::Fault, act.reason, now);
  } else if (act.kind == Action::Kind::Abort) {
    tr = machine_.force(State::Aborting, act.reason, now);
  } else if (abort_requested && recipe_active(before)) {
    tr = machine_.force(State::Aborting, Reason::OperatorAbort, now);
  } else {
    tr = machine_.tick({latest_, now, dt_ns, trips.any()});
  }

  // ---- 5. Outputs ----------------------------------------------------------
  const recipe::Outputs& o = machine_.outputs();
  if (tr.happened && tr.to == State::RampUp) pid_.reset();  // fresh start for each wafer
  double heater = 0.0;
  if (o.pid_enabled && o.heater_contactor) {
    const double ff = std::clamp((o.setpoint_c - cfg_.ff_ambient_c) / cfg_.ff_gain_c, 0.0, 1.0);
    heater = pid_.update(o.setpoint_c, t_meas, static_cast<double>(dt_ns) * 1e-9, ff);
  }
  const ActuatorCommand cmd{heater, o.heater_contactor, o.pump, o.vent, o.valve_open};
  port_.write(cmd);
  last_cmd_ = cmd;
  const std::int64_t write_ns = clock_ ? clock_() : now;

  // ---- 6. Bookkeeping ------------------------------------------------------
  if (tr.happened) on_transition(tr, now, write_ns);
  update_quality(t_meas);
  publish_status(t_meas);
}

void Controller::log_trip_edges(TripSet trips, std::int64_t now) noexcept {
  const TripSet rising = trips.newly_set_vs(prev_trips_);
  const TripSet falling = prev_trips_.newly_set_vs(trips);
  for (std::size_t i = 0; i < safety::kInterlockCount; ++i) {
    const auto il = static_cast<Interlock>(i);
    if (rising.test(il)) {
      ++m_.interlock_trips[i];
      LogRecord r = record(LogEvent::InterlockTrip, now);
      r.a = static_cast<std::uint8_t>(i);
      r.c = static_cast<std::uint8_t>(safety::reason_for(il));
      log(r);
    }
    if (falling.test(il)) {
      LogRecord r = record(LogEvent::InterlockClear, now);
      r.a = static_cast<std::uint8_t>(i);
      r.c = static_cast<std::uint8_t>(safety::reason_for(il));
      log(r);
    }
  }
  prev_trips_ = trips;
}

void Controller::on_transition(const recipe::Transition& tr, std::int64_t now,
                               std::int64_t write_ns) noexcept {
  LogRecord r = record(LogEvent::StateChange, now);
  r.a = static_cast<std::uint8_t>(tr.from);
  r.b = static_cast<std::uint8_t>(tr.to);
  r.c = static_cast<std::uint8_t>(tr.reason);
  log(r);

  auto open_new_event = [&](bool fault) {
    if (m_.safety_event_count >= telemetry::ControllerMetrics::kMaxSafetyEvents) {
      open_event_ = -1;
      return;
    }
    telemetry::SafetyEvent& e = m_.safety_events[m_.safety_event_count];
    e = telemetry::SafetyEvent{};
    e.reason = tr.reason;
    e.from = tr.from;
    e.fault = fault;
    e.trip_ns = now;
    e.detect_latency_ns = have_sample_ ? now - latest_.t_ns : 0;
    e.heater_off_latency_ns = write_ns - now;
    open_event_ = static_cast<int>(m_.safety_event_count++);
  };
  auto close_event = [&](std::int64_t safe_at) {
    if (open_event_ < 0) return;
    telemetry::SafetyEvent& e = m_.safety_events[static_cast<std::size_t>(open_event_)];
    e.safe_ns = safe_at - e.trip_ns;
    LogRecord s = record(LogEvent::SafeReached, now);
    s.c = static_cast<std::uint8_t>(e.reason);
    s.x = static_cast<double>(e.safe_ns) / 1e6;
    log(s);
    open_event_ = -1;
  };

  if (tr.to == State::Aborting && tr.from != State::Fault) {
    ++m_.aborts;
    open_new_event(false);
  } else if (tr.to == State::Fault) {
    ++m_.faults;
    if (tr.from == State::Aborting && open_event_ >= 0) {
      // The abort path was interrupted by a more severe fault.
      telemetry::SafetyEvent& e = m_.safety_events[static_cast<std::size_t>(open_event_)];
      e.escalated = true;
      e.fault = true;
      e.reason = tr.reason;
    } else {
      open_new_event(true);
    }
    close_event(write_ns);  // Fault is safe the moment everything is de-energised
  } else if (tr.from == State::Aborting && tr.to == State::Idle) {
    close_event(now);  // vented and nothing tripped
  }

  if (tr.from == State::Unload && tr.to == State::Idle) {
    m_.wafers_processed = machine_.wafers_done();
    m_.last_cycle_ns = machine_.last_cycle_ns();
    m_.total_cycle_ns += machine_.last_cycle_ns();
    LogRecord w = record(LogEvent::WaferComplete, now);
    w.u = m_.wafers_processed;
    w.x = static_cast<double>(m_.last_cycle_ns) / 1e9;
    log(w);
  }
}

void Controller::update_quality(double t_meas) noexcept {
  const State s = machine_.state();
  const double sp = cfg_.recipe.process_temp_c;
  if (s == State::Process) m_.process_max_dev_c = std::max(m_.process_max_dev_c, std::abs(t_meas - sp));
  if (s == State::RampUp || s == State::Process) {
    m_.rampup_overshoot_c = std::max(m_.rampup_overshoot_c, t_meas - sp);
  }
}

void Controller::publish_status(double t_meas) noexcept {
  constexpr auto rlx = std::memory_order_relaxed;
  status_.state.store(static_cast<std::uint8_t>(machine_.state()), rlx);
  status_.reason.store(static_cast<std::uint8_t>(machine_.reason()), rlx);
  status_.temperature_c.store(t_meas, rlx);
  status_.pressure_torr.store(latest_.pressure_torr, rlx);
  status_.valve_position.store(latest_.valve_position, rlx);
  status_.heater.store(last_cmd_.heater, rlx);
  status_.wafers_done.store(machine_.wafers_done(), rlx);
  status_.wafers_pending.store(machine_.wafers_pending(), rlx);
  status_.overruns.store(m_.buffer_overruns, rlx);
  status_.wafers_accepted.store(accepted_, std::memory_order_release);  // publish last
}

}  // namespace ptc::control
