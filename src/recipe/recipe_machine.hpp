#pragma once

#include <cstdint>

#include "common/types.hpp"

namespace ptc::recipe {

struct RecipeParams {
  double process_temp_c = 350.0;
  double band_c = 2.0;                          // FR-2: +/- 2 C
  std::int64_t settle_ns = 500 * kNsPerMs;      // FR-2: 500 consecutive ms
  double pumpdown_target_torr = 1e-3;           // FR-2: < 1e-3 Torr
  double unload_temp_c = 150.0;
  double vent_target_torr = 750.0;
  std::int64_t transfer_dwell_ns = 500 * kNsPerMs;  // robot hand-off time with valve open
  std::int64_t process_time_ns = 3 * kNsPerSec;

  // FR-3: maximum time in each state. Roughly 2-3x the nominal duration:
  // tight enough to catch a stuck mechanism, loose enough not to false-trip.
  std::int64_t load_timeout_ns = 3 * kNsPerSec;
  std::int64_t pumpdown_timeout_ns = 6 * kNsPerSec;
  std::int64_t rampup_timeout_ns = 8 * kNsPerSec;
  std::int64_t process_timeout_margin_ns = 1 * kNsPerSec;  // added to process_time
  std::int64_t rampdown_timeout_ns = 8 * kNsPerSec;
  std::int64_t vent_timeout_ns = 4 * kNsPerSec;
  std::int64_t unload_timeout_ns = 3 * kNsPerSec;
  std::int64_t abort_timeout_ns = 5 * kNsPerSec;  // FR-5: abort completes in bounded time
};

// What the current state wants the hardware to do. The controller turns this
// into an ActuatorCommand (running the PID if pid_enabled).
struct Outputs {
  bool heater_contactor = true;
  bool pid_enabled = false;
  double setpoint_c = 0.0;
  bool pump = false;
  bool vent = false;
  bool valve_open = false;
};

struct Transition {
  bool happened = false;
  State from = State::Idle;
  State to = State::Idle;
  Reason reason = Reason::None;
};

// Recipe state machine (FR-1, FR-3).
//
// Table-driven: each state is one row of {entry action, per-tick action, exit
// condition}, plus a timeout from RecipeParams. Adding a state means adding a
// row, not threading another `if` through nested conditionals, and every
// transition goes through one function (go) so logging, counting and entry
// actions can never be skipped.
//
// The machine knows nothing about interlocks. Safety lives in safety/ and
// forces transitions from outside via force(); the recipe cannot veto it.
//
// At most one transition per tick: behaviour is deterministic and every
// state's entry action is observed for at least one tick.
class RecipeMachine {
 public:
  struct Input {
    const SensorSample& s;   // latest sensor sample
    std::int64_t now_ns;
    std::int64_t dt_ns;      // time since previous tick
    bool trips_active;       // any interlock currently tripped
  };

  explicit RecipeMachine(const RecipeParams& p = {});

  void request_wafers(std::uint32_t n) noexcept { pending_ += n; }
  void request_reset() noexcept { reset_requested_ = true; }

  // Normal progression: timeout check, then per-tick action, then exit check.
  Transition tick(const Input& in) noexcept;

  // Used by the safety supervisor: unconditional jump to Aborting or Fault.
  Transition force(State target, Reason reason, std::int64_t now_ns) noexcept;

  State state() const noexcept { return state_; }
  Reason reason() const noexcept { return reason_; }
  const Outputs& outputs() const noexcept { return out_; }
  std::int64_t entered_ns() const noexcept { return entered_ns_; }
  std::uint32_t wafers_pending() const noexcept { return pending_; }
  std::uint32_t wafers_done() const noexcept { return done_; }
  std::int64_t last_cycle_ns() const noexcept { return last_cycle_ns_; }
  const RecipeParams& params() const noexcept { return p_; }

  static std::int64_t timeout_ns(State s, const RecipeParams& p) noexcept;  // 0 = none

 private:
  struct StateDef {
    void (*entry)(RecipeMachine&, std::int64_t now);
    void (*during)(RecipeMachine&, const Input&);
    State (*exit)(const RecipeMachine&, const Input&);  // returns next state, or own state to stay
  };
  static const StateDef& def(State s) noexcept;

  Transition go(State next, Reason reason, std::int64_t now) noexcept;

  // State actions (static so they fit in the table; they act on `m`).
  static void enter_idle(RecipeMachine& m, std::int64_t);
  static void enter_transfer(RecipeMachine& m, std::int64_t);
  static void enter_pumpdown(RecipeMachine& m, std::int64_t);
  static void enter_rampup(RecipeMachine& m, std::int64_t);
  static void enter_process(RecipeMachine& m, std::int64_t);
  static void enter_rampdown(RecipeMachine& m, std::int64_t);
  static void enter_vent(RecipeMachine& m, std::int64_t);
  static void enter_aborting(RecipeMachine& m, std::int64_t);
  static void enter_fault(RecipeMachine& m, std::int64_t);

  static void during_nothing(RecipeMachine&, const Input&) {}
  static void during_transfer(RecipeMachine& m, const Input& in);
  static void during_rampup(RecipeMachine& m, const Input& in);

  static State exit_idle(const RecipeMachine& m, const Input& in);
  static State exit_load(const RecipeMachine& m, const Input& in);
  static State exit_pumpdown(const RecipeMachine& m, const Input& in);
  static State exit_rampup(const RecipeMachine& m, const Input& in);
  static State exit_process(const RecipeMachine& m, const Input& in);
  static State exit_rampdown(const RecipeMachine& m, const Input& in);
  static State exit_vent(const RecipeMachine& m, const Input& in);
  static State exit_unload(const RecipeMachine& m, const Input& in);
  static State exit_aborting(const RecipeMachine& m, const Input& in);
  static State exit_fault(const RecipeMachine& m, const Input& in);

  bool transfer_done(const Input& in) const noexcept;

  RecipeParams p_;
  State state_ = State::Idle;
  Reason reason_ = Reason::None;
  Outputs out_{};
  std::int64_t entered_ns_ = 0;

  std::uint32_t pending_ = 0;
  std::uint32_t done_ = 0;
  bool reset_requested_ = false;

  std::int64_t in_band_ns_ = 0;      // RampUp settle accumulator
  std::int64_t dwell_start_ns_ = 0;  // Load/Unload: when valve reached fully open (0 = not yet)
  std::int64_t cycle_start_ns_ = 0;
  std::int64_t last_cycle_ns_ = 0;
};

}  // namespace ptc::recipe
