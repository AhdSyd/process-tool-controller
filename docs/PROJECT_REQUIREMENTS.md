# Process Tool Controller — Project Requirements

**Target:** a C++ control system for simulated semiconductor process equipment.
**Purpose:** demonstrable evidence of multithreading, soft real-time control, state-machine
design, RAII memory management, and measured performance — the skills named in ASM, KLA,
Applied Materials, and NVIDIA SoC job descriptions.

**Hand this document to an AI coding tool as the spec. Build it in milestones, not all at once.**

---

## 1. What the system does

A single process chamber runs wafers through a recipe. The controller:

- drives the chamber through a fixed sequence of process states,
- reads simulated sensors (temperature, pressure, slit-valve position) continuously,
- commands simulated actuators (heater, pump, valve) on a deterministic cycle,
- enforces safety interlocks and aborts safely on fault,
- logs everything without ever stalling the control loop,
- reports timing and throughput metrics at the end of each run.

The hardware is simulated in software. The control software is real.

---

## 2. Scope

**In scope**
- Fixed-tick control loop with measured timing behaviour
- Multi-threaded sensor acquisition with a bounded shared buffer
- Recipe state machine with guards, timeouts, and fault transitions
- Interlock system that can abort a run mid-recipe
- Non-blocking logging
- Metrics collection and an end-of-run report
- Unit tests for state machine, interlocks, and buffer
- A fault-injection mode for testing abort paths

**Out of scope** (say so in the README — scoping deliberately is a positive signal)
- GUI. A text console is fine.
- Real hardware I/O.
- Networking or multi-chamber coordination.
- Persistence beyond log files.

---

## 3. Architecture

Five modules. Keep them in separate translation units with clean headers.

| Module | Responsibility |
|---|---|
| `sim/` | Plant model: thermal mass, pump-down curve, valve travel, sensor noise |
| `io/` | Sensor sampling threads and actuator command interface |
| `control/` | Fixed-tick control loop, PID or bang-bang control, setpoint tracking |
| `recipe/` | State machine, step definitions, guards, timeouts |
| `safety/` | Interlock evaluation and abort sequencing |
| `telemetry/` | Lock-free log queue, logging thread, metrics aggregation |

**Thread layout (exactly four threads):**

1. **Control thread** — fixed 1 ms tick, highest priority, never blocks, never allocates, never does I/O
2. **Sensor thread** — samples the plant model at 5 kHz, publishes into a ring buffer
3. **Logging thread** — drains the log queue and writes to disk
4. **Main/UI thread** — accepts console commands (start, abort, status, quit)

---

## 4. Functional requirements

**FR-1 — Recipe state machine**
States: `Idle → Load → PumpDown → RampUp → Process → RampDown → Vent → Unload → Idle`.
Plus `Fault` and `Aborting`, reachable from any state.
Each state has: an entry action, a per-tick action, an exit condition, and a timeout.

**FR-2 — Transition guards**
`PumpDown → RampUp` only when chamber pressure < 1e-3 Torr.
`RampUp → Process` only when temperature is within ±2 °C of setpoint for 500 consecutive ms.
Every guard is a pure function over the latest sensor snapshot — unit-testable in isolation.

**FR-3 — State timeouts**
Each state declares a maximum duration. Exceeding it transitions to `Fault` with a reason code.

**FR-4 — Interlocks**
Evaluated every tick, independent of current state. Minimum set:
- Temperature > 450 °C → abort
- Pressure > 10 Torr while heater is on → abort
- Slit valve open while pressure < 1 Torr → abort
- No sensor update within 10 ms → abort (stale data)

**FR-5 — Abort sequence**
On abort: heater off immediately, then vent, then `Idle`. The abort path itself must be
interruptible by a more severe fault and must complete within a bounded time.

**FR-6 — Sensor acquisition**
Sensor thread samples at 5 kHz and writes to a fixed-size ring buffer. The control thread
reads the most recent sample each tick. Buffer overrun must be detected and counted, never
silently dropped.

**FR-7 — Control law**
Heater control is a PID loop tracking the recipe temperature setpoint. Output is clamped.
Integral windup must be handled — be ready to explain how.

**FR-8 — Logging**
Every state transition, interlock trip, and fault is logged with a monotonic timestamp.
The control thread may only enqueue; it must never format strings or touch the filesystem.

**FR-9 — Metrics report**
At run end, print: tick count, missed deadlines, jitter p50/p99/p99.9/max, buffer overruns,
wafers processed, total runtime.

**FR-10 — Fault injection**
A command-line flag injects faults (sensor dropout, temperature runaway, stuck valve) so the
abort paths are exercised and testable.

---

## 5. Non-functional requirements

**NFR-1 — Tick determinism**
Control loop target 1 ms. Use `std::chrono::steady_clock` with `sleep_until` against an
absolute next-tick deadline, **not** `sleep_for` — explain the difference in the README.

**NFR-2 — Honest real-time framing**
This is **soft real-time on a general-purpose OS.** Do not claim hard real-time guarantees.
State this explicitly in the README and report measured jitter rather than asserting bounds.
*(This honesty is itself a credibility signal to embedded engineers.)*

**NFR-3 — No allocation in the control thread**
All buffers preallocated. No `new`, no `std::string` construction, no container growth inside
the tick. Be ready to explain why allocation is forbidden in a real-time path.

**NFR-4 — Memory discipline**
RAII throughout. `std::unique_ptr` for ownership, references for non-owning access. No raw
`new`/`delete`. Clean under ASan/UBSan and clean under TSan.

**NFR-5 — Shutdown**
Ctrl-C and `quit` both shut down every thread cleanly, flush the log queue, and print the
metrics report. No detached threads, no leaked resources.

---

## 6. Technology constraints

- **C++17 or C++20.** Pick one, state it, don't mix.
- **Standard library only.** `std::thread`, `std::mutex`, `std::condition_variable`,
  `std::atomic`, `std::chrono`. No Boost, no external concurrency libraries.
- **CMake** build, out-of-source, Debug and Release configurations.
- **GoogleTest** for unit tests.
- **Sanitizers** wired into CMake: a build target that runs under ASan/UBSan and one under TSan.
- Cross-platform (Linux and Windows) if cheap; Linux-only is acceptable if you say so.

---

## 7. Acceptance criteria

The project is done when all of these hold:

1. A full recipe runs end to end and produces a metrics report.
2. Jitter p99 is reported with a real measured number, whatever it is.
3. Every interlock is proven to fire by a fault-injection test.
4. TSan reports no data races across a 60-second run.
5. ASan/UBSan report no leaks or undefined behaviour.
6. Unit tests cover every guard, every interlock, and ring-buffer overrun.
7. README states the architecture, the thread model, the real-time caveat, and the measured numbers.

---

## 8. Milestones

**M1 — Skeleton that ticks.** Control thread, fixed-tick loop, jitter measurement, clean
shutdown. *Nothing else.* Get the timing right before adding features.

**M2 — Plant and sensors.** Simulation model, sensor thread, ring buffer, overrun detection.

**M3 — Recipe and safety.** State machine, guards, timeouts, interlocks, abort sequence.

**M4 — Telemetry and hardening.** Logging thread, metrics report, fault injection, tests,
sanitizer runs, README.

Each milestone should build, run, and be committable on its own.

---

## 9. The nuance checklist

**This is the part that determines whether the project helps or hurts you.** You must be able
to answer each of these without notes. Work through them as the code is written, not after.

**Timing**
- Why `sleep_until` with an absolute deadline rather than `sleep_for`?
- What causes jitter on a general-purpose OS, and why isn't this hard real-time?
- What does your p99 number actually mean, and why report percentiles rather than an average?

**Concurrency**
- Exactly which data is shared between threads, and what protects each piece?
- Why a ring buffer rather than a mutex-protected queue for sensor data? What's the trade-off?
- What happens when the sensor thread outpaces the control thread? Why is that the right behaviour?
- Where did you use `std::atomic` and what memory ordering did you specify? Why that ordering?
- What's a data race versus a race condition? Does TSan catch both?
- Why must the control thread never take a lock that the logging thread can hold?

**Memory**
- Why is allocation forbidden in the control thread?
- Where does every heap allocation in the program happen, and when?
- Which objects own which, and how is that expressed in the types?
- What does RAII buy you here specifically?

**Design**
- Why a state machine rather than nested conditionals?
- How do interlocks stay independent of recipe state, and why does that matter for safety?
- What happens if a fault occurs during the abort sequence?
- What would change if this ran on a real RTOS instead of Linux?

**If you can't answer one of these, stop and learn it before moving on.** An ASM engineer will
ask two or three of them, and the whole value of the project rests on those answers.

---

## 10. Resume bullets this produces

Draft these from real measured numbers once M4 is done — do not write them in advance:

> Built a multi-threaded process-tool controller in C++17 driving a simulated semiconductor
> chamber, holding a 1 kHz control loop with p99 jitter under [X] µs across [Y]-minute runs.

> Designed a recipe state machine with guard conditions, state timeouts, and independent
> safety interlocks that abort and vent the chamber within [Z] ms of a fault.

> Implemented lock-free sensor acquisition between a 5 kHz sampling thread and the control
> loop, with overrun detection; verified race-free under ThreadSanitizer and leak-free under
> AddressSanitizer.

Each cites a number you measured and a technique you can explain.
