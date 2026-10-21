#include "recipe/recipe_machine.hpp"

#include <array>

#include "recipe/guards.hpp"

namespace ptc::recipe {

RecipeMachine::RecipeMachine(const RecipeParams& p) : p_(p) { enter_idle(*this, 0); }

std::int64_t RecipeMachine::timeout_ns(State s, const RecipeParams& p) noexcept {
  switch (s) {
    case State::Load: return p.load_timeout_ns;
    case State::PumpDown: return p.pumpdown_timeout_ns;
    case State::RampUp: return p.rampup_timeout_ns;
    case State::Process: return p.process_time_ns + p.process_timeout_margin_ns;
    case State::RampDown: return p.rampdown_timeout_ns;
    case State::Vent: return p.vent_timeout_ns;
    case State::Unload: return p.unload_timeout_ns;
    case State::Aborting: return p.abort_timeout_ns;
    case State::Idle:   // waits for the operator indefinitely
    case State::Fault:  // latched until reset
    case State::kCount: return 0;
  }
  return 0;
}

const RecipeMachine::StateDef& RecipeMachine::def(State s) noexcept {
  // The whole recipe at a glance: one row per state.
  //                      entry             per-tick          exit condition
  static const std::array<StateDef, static_cast<std::size_t>(State::kCount)> table = {{
      /* Idle     */ {&enter_idle,      &during_nothing,  &exit_idle},
      /* Load     */ {&enter_transfer,  &during_transfer, &exit_load},
      /* PumpDown */ {&enter_pumpdown,  &during_nothing,  &exit_pumpdown},
      /* RampUp   */ {&enter_rampup,    &during_rampup,   &exit_rampup},
      /* Process  */ {&enter_process,   &during_nothing,  &exit_process},
      /* RampDown */ {&enter_rampdown,  &during_nothing,  &exit_rampdown},
      /* Vent     */ {&enter_vent,      &during_nothing,  &exit_vent},
      /* Unload   */ {&enter_transfer,  &during_transfer, &exit_unload},
      /* Aborting */ {&enter_aborting,  &during_nothing,  &exit_aborting},
      /* Fault    */ {&enter_fault,     &during_nothing,  &exit_fault},
  }};
  return table[static_cast<std::size_t>(s)];
}

Transition RecipeMachine::tick(const Input& in) noexcept {
  if (state_ != State::Fault) reset_requested_ = false;  // reset only means something in Fault

  // FR-3: timeout check first, so a state that is "almost done" cannot keep
  // the machine alive past its limit.
  const std::int64_t limit = timeout_ns(state_, p_);
  if (limit > 0 && in.now_ns - entered_ns_ > limit) {
    return go(State::Fault, state_ == State::Aborting ? Reason::AbortTimeout : Reason::StateTimeout,
              in.now_ns);
  }

  const StateDef& d = def(state_);
  d.during(*this, in);
  const State next = d.exit(*this, in);
  if (next != state_) return go(next, Reason::None, in.now_ns);
  return {};
}

Transition RecipeMachine::force(State target, Reason reason, std::int64_t now_ns) noexcept {
  if (state_ == target) return {};
  return go(target, reason, now_ns);
}

Transition RecipeMachine::go(State next, Reason reason, std::int64_t now) noexcept {
  const Transition t{true, state_, next, reason};

  if (state_ == State::Idle && next == State::Load) {
    --pending_;
    cycle_start_ns_ = now;
  }
  if (state_ == State::Unload && next == State::Idle) {
    ++done_;
    last_cycle_ns_ = now - cycle_start_ns_;
  }
  if (state_ == State::Fault) reset_requested_ = false;

  state_ = next;
  entered_ns_ = now;
  if (next == State::Aborting || next == State::Fault) {
    if (reason != Reason::None) reason_ = reason;  // Fault->Aborting on reset keeps the original
  } else {
    reason_ = Reason::None;
  }
  def(next).entry(*this, now);
  return t;
}

// ---------------------------------------------------------------------------
// Entry actions: set the complete output picture for the state.
// ---------------------------------------------------------------------------
void RecipeMachine::enter_idle(RecipeMachine& m, std::int64_t) { m.out_ = Outputs{}; }

void RecipeMachine::enter_transfer(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.valve_open = true;  // robot needs the slit valve open
  m.dwell_start_ns_ = 0;
}

void RecipeMachine::enter_pumpdown(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.pump = true;
}

void RecipeMachine::enter_rampup(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.pump = true;
  m.out_.pid_enabled = true;
  m.out_.setpoint_c = m.p_.process_temp_c;
  m.in_band_ns_ = 0;
}

void RecipeMachine::enter_process(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.pump = true;
  m.out_.pid_enabled = true;
  m.out_.setpoint_c = m.p_.process_temp_c;
}

void RecipeMachine::enter_rampdown(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.pump = true;  // heater off (pid disabled), keep vacuum while cooling
}

void RecipeMachine::enter_vent(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.vent = true;
}

// FR-5: heater off IMMEDIATELY (this runs in the same tick as the trip), close
// the slit valve, stop pumping, vent to atmosphere. The heater contactor stays
// closed: abort is the controlled path; Fault is the one that cuts power.
void RecipeMachine::enter_aborting(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.vent = true;
  m.pending_ = 0;  // an abort cancels the rest of the run
}

// Fault: de-energise everything, including the heater contactor, and latch
// until an operator reset. Valve de-energised = closed (spring-return).
void RecipeMachine::enter_fault(RecipeMachine& m, std::int64_t) {
  m.out_ = Outputs{};
  m.out_.heater_contactor = false;
  m.pending_ = 0;
}

// ---------------------------------------------------------------------------
// Per-tick actions
// ---------------------------------------------------------------------------
void RecipeMachine::during_transfer(RecipeMachine& m, const Input& in) {
  if (m.dwell_start_ns_ == 0 && guards::valve_fully_open(in.s)) m.dwell_start_ns_ = in.now_ns;
  if (m.dwell_start_ns_ != 0 && in.now_ns - m.dwell_start_ns_ >= m.p_.transfer_dwell_ns) {
    m.out_.valve_open = false;  // hand-off done: close the slit valve
  }
}

void RecipeMachine::during_rampup(RecipeMachine& m, const Input& in) {
  m.in_band_ns_ = guards::update_in_band(m.in_band_ns_, in.s, m.p_.process_temp_c, m.p_.band_c, in.dt_ns);
}

// ---------------------------------------------------------------------------
// Exit conditions (all built from the pure guards)
// ---------------------------------------------------------------------------
bool RecipeMachine::transfer_done(const Input& in) const noexcept {
  return dwell_start_ns_ != 0 && !out_.valve_open && guards::valve_fully_closed(in.s);
}

State RecipeMachine::exit_idle(const RecipeMachine& m, const Input& in) {
  return (m.pending_ > 0 && !in.trips_active) ? State::Load : State::Idle;
}
State RecipeMachine::exit_load(const RecipeMachine& m, const Input& in) {
  return m.transfer_done(in) ? State::PumpDown : State::Load;
}
State RecipeMachine::exit_pumpdown(const RecipeMachine& m, const Input& in) {
  return guards::pumpdown_complete(in.s, m.p_.pumpdown_target_torr) ? State::RampUp : State::PumpDown;
}
State RecipeMachine::exit_rampup(const RecipeMachine& m, const Input&) {
  return guards::settled(m.in_band_ns_, m.p_.settle_ns) ? State::Process : State::RampUp;
}
State RecipeMachine::exit_process(const RecipeMachine& m, const Input& in) {
  return (in.now_ns - m.entered_ns_ >= m.p_.process_time_ns) ? State::RampDown : State::Process;
}
State RecipeMachine::exit_rampdown(const RecipeMachine& m, const Input& in) {
  return guards::cooled(in.s, m.p_.unload_temp_c) ? State::Vent : State::RampDown;
}
State RecipeMachine::exit_vent(const RecipeMachine& m, const Input& in) {
  return guards::vented(in.s, m.p_.vent_target_torr) ? State::Unload : State::Vent;
}
State RecipeMachine::exit_unload(const RecipeMachine& m, const Input& in) {
  return m.transfer_done(in) ? State::Idle : State::Unload;
}
State RecipeMachine::exit_aborting(const RecipeMachine& m, const Input& in) {
  // Safe = at atmosphere AND nothing still tripped.
  return (guards::vented(in.s, m.p_.vent_target_torr) && !in.trips_active) ? State::Idle
                                                                          : State::Aborting;
}
State RecipeMachine::exit_fault(const RecipeMachine& m, const Input& in) {
  // Reset re-runs the vent sequence so the chamber returns to a known state
  // (atmosphere) before Idle, instead of jumping to Idle under vacuum.
  return (m.reset_requested_ && !in.trips_active) ? State::Aborting : State::Fault;
}

}  // namespace ptc::recipe
