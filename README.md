# Process Tool Controller

A multithreaded C++17 control system for a simulated semiconductor process chamber.
A 1 kHz control loop runs wafers through a recipe (pump down, heat, process, cool,
vent), reads 5 kHz sensor data through a lock-free ring buffer, enforces safety
interlocks every tick, and aborts the chamber safely when something goes wrong.

**The hardware is simulated; the control software is real.** The controller only sees
sensor numbers and sends actuator commands, exactly as it would against real equipment.

> **Soft real-time on a general-purpose OS.** This project does not claim hard real-time
> guarantees. Every timing number below is a measurement of a specific run on a specific
> machine, reported as percentiles, not a bound.

## Measured results

Release build, 2-vCPU Intel Xeon cloud VM (Firecracker), Linux 6.18, GCC 13.3. Each run is
10 wafers back to back (~131 s, ~130 000 control ticks).

| | p50 | p99 | p99.9 | max | missed deadlines | sensor overruns |
|---|---|---|---|---|---|---|
| Control loop, SCHED_FIFO 80 | 26 µs | 129 µs | 718 µs | 109 ms | 0.28 % | 385 |
| Control loop, normal scheduling | 11 µs | 67 µs | 723 µs | 9.3 ms | 0.14 % | 0 |
| Sensor loop (5 kHz), SCHED_FIFO 70 | 11 µs | 25 µs | 72 µs | 36 ms | 0.16 % | — |

Tick execution time ≤ 4 µs at p99.9. Process quality: **10/10 wafers, temperature held
within ±1.06 °C** of the 350 °C setpoint (spec ±2 °C), 13.1 s cycle.

**Reading these honestly:**
- On a quiet run, SCHED_FIFO roughly halves p99 (an earlier 60 s comparison: 264 → 134
  µs). In the FIFO run above, the VM was descheduled for **109 ms** by the hypervisor
  (`steal` time), which no in-guest priority can prevent. That one stall dominates the
  tail, which is why the run without FIFO looks better. On a shared VM, run-to-run
  variation in the tail is larger than the effect of the scheduling policy.
- The 109 ms stall is also the best demonstration of the design. The sensor ring
  overflowed, **385 samples were overwritten, every one was counted and logged**, the
  stale-data debounce didn't false-trip, and the recipe carried on. Nothing failed
  silently.
- Expect much better tails on bare metal with `PREEMPT_RT` and an isolated core. Re-run
  `ptc --wafers 10` on your own machine and quote those numbers.

Fault response (real binary, 4 threads, fault injected 500 ms into Process):

| Fault | Interlock | Detect¹ | Heater off² | To safe state |
|---|---|---|---|---|
| `vacuum-leak` | OverPressureHeaterOn → abort | 0.17 ms | 1.0 µs | 900 ms (vented) |
| `stuck-valve` | SlitValveOpenUnderVacuum → abort | 0.15 ms | 1.4 µs | 948 ms (vented) |
| `temp-runaway` | OverTemperature → abort → **CriticalOverTemperature → Fault** | 0.18 ms | 0.8 µs | 840 ms (contactor open) |
| `sensor-dropout` | StaleSensorData → Fault | 11.2 ms³ | 0.5 µs | same tick (de-energised) |

¹ Age of the sample that showed the problem when the control tick acted on it.
² From detection to the actuator command being written, inside the same tick.
³ The limit is 10 ms with a 1-tick debounce, so ~11 ms is the design value.

---

## Build and run

Linux (primary) or macOS. CMake ≥ 3.20, a C++17 compiler. GoogleTest is fetched
automatically.

```bash
cmake -S . -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
./build/release/ptc_tests                      # 87 unit + system tests
(cd build/release && ctest -L e2e)             # 5 end-to-end fault-injection runs

sudo ./build/release/ptc --wafers 3            # headless: 3 wafers, then report
sudo ./build/release/ptc                       # interactive: start [n] | abort | reset | status | quit
sudo ./build/release/ptc --wafers 1 --fault temp-runaway
```

`sudo` (or `CAP_SYS_NICE`) lets the control thread get SCHED_FIFO priority. Without it
the program still runs and the report states that realtime was *not* granted. On macOS
the realtime request uses the Mach time-constraint policy and needs no `sudo`.

Sanitizer builds:

```bash
cmake -S . -B build/address -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPTC_SANITIZE=address   # ASan + UBSan
cmake -S . -B build/thread  -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPTC_SANITIZE=thread    # TSan
```

On a Mac without a native toolchain, Docker works:
`docker run --rm -it --cap-add=SYS_NICE -v "$PWD":/ptc -w /ptc ubuntu:24.04 bash`, then
`apt update && apt install -y build-essential cmake ninja-build git` and the commands above.

| Flag | Meaning |
|---|---|
| `--wafers <n>` | Headless: process n wafers, print report, exit |
| `--duration <s>` | Headless: stop after s seconds |
| `--fault <name>` | `sensor-dropout`, `temp-runaway`, `stuck-valve`, `vacuum-leak` |
| `--fault-state <S>` / `--fault-delay-ms <ms>` | When to inject (default 500 ms into `Process`) |
| `--process-time <s>` | Length of the Process step (default 3) |
| `--log <path>` | Log file (default `ptc_run.log`) |
| `--quiet` | Don't echo events to the console |
| `--cpu <n>`, `--priority <1-99>`, `--no-rt`, `--no-mlock`, `--period-us <us>` | Scheduling knobs |

---

## Architecture

```
                 ┌─────────────── sensor thread (5 kHz, FIFO 70) ───────────────┐
                 │  ActuatorPort.read() → Plant.step(dt) → Plant.sample()       │
                 └───────────────────────────────┬──────────────────────────────┘
                                                 │ SeqlockRing<SensorSample,64>
                                                 │ (lock-free, overwrite-oldest,
                                                 ▼  overruns counted)
 ┌──────────────────────────── control thread (1 kHz, FIFO 80) ────────────────────────────┐
 │ 1 drain ring → latest sample   2 operator commands   3 interlocks (every tick)          │
 │ 4 arbitrate: safety > operator abort > recipe tick   5 PID → ActuatorPort.write()       │
 │ 6 enqueue log records, update metrics, publish status                                   │
 └──────────────┬──────────────────────────────────────┬────────────────────────────────────┘
                │ SpscQueue<LogRecord> (lock-free)      │ StatusBoard / CommandMailbox (atomics)
                ▼                                       ▼
   logging thread: format + write file        main/UI thread: console, signals, report
```

| Module | Responsibility | Key files |
|---|---|---|
| `sim/` | Plant model: thermal lag, pump-down/vent curves, valve travel, sensor noise, injectable faults | `plant.*` |
| `io/` | Sensor sampling task, seqlock ring buffer, actuator command port | `sensor_task.*`, `seqlock_ring.hpp`, `actuator_port.hpp` |
| `control/` | Fixed-period thread runner, controller tick, PID | `periodic_loop.*`, `controller.*`, `pid.*` |
| `recipe/` | Table-driven state machine, pure guards, timeouts | `recipe_machine.*`, `guards.hpp` |
| `safety/` | Interlocks, hysteresis, stale-data debounce, abort/fault decision | `interlocks.*` |
| `telemetry/` | SPSC log queues, logging thread, latency histograms, metrics, report | `logger.*`, `spsc_queue.hpp`, `report.*` |
| `platform/` | SCHED_FIFO / Mach RT policy, CPU pinning, timer slack, `mlockall` | `rt_thread.*` |

### Threads (exactly four)

1. **Control:** 1 ms tick, SCHED_FIFO 80. Never blocks, never allocates, never does I/O.
2. **Sensor:** 200 µs tick, SCHED_FIFO 70. Steps the plant physics by the *real* elapsed
   time, samples the sensors, and publishes into the ring.
3. **Logging:** polls its queues every 5 ms, formats, and writes to disk.
4. **Main/UI:** console commands, signal handling, graceful shutdown, final report.

The control and sensor threads are the same class, `PeriodicLoop`, running a different
`TickTask`, so the absolute-deadline timing logic exists exactly once.

### Recipe

`Idle → Load → PumpDown → RampUp → Process → RampDown → Vent → Unload → Idle`, plus
`Aborting` and `Fault`, which are reachable from any state.

| State | Outputs | Exit condition (guard) | Timeout |
|---|---|---|---|
| Load / Unload | slit valve open, dwell 500 ms, close | valve confirmed closed | 3 s |
| PumpDown | pump on | P < 1e-3 Torr | 6 s |
| RampUp | pump, PID → 350 °C | ±2 °C for 500 consecutive ms | 8 s |
| Process | pump, PID | 3 s elapsed | 4 s |
| RampDown | pump, heater off | T < 150 °C | 8 s |
| Vent | vent valve open | P ≥ 750 Torr | 4 s |
| **Aborting** | heater off *immediately*, valve closed, pump off, vent | vented **and** no active trip → Idle | 5 s → Fault |
| **Fault** | everything de-energised **including the heater contactor**, latched | operator `reset` (and cause cleared) → Aborting → Idle | none |

### Interlocks

| Interlock | Condition | Severity |
|---|---|---|
| OverTemperature | T > 450 °C | Abort |
| CriticalOverTemperature | T > 500 °C | **Fault** |
| OverPressureHeaterOn | P > 10 Torr with heater on | Abort |
| SlitValveOpenUnderVacuum | valve > 5 % open with P < 1 Torr | Abort |
| StaleSensorData | no new sample for > 10 ms (2 consecutive ticks) | **Fault** |

Two deliberate deviations from the original spec, both towards safety:
- **CriticalOverTemperature** was added so the abort path has something more severe to
  be interrupted by (FR-5).
- **Stale data is a Fault, not an Abort.** The vent sequence's exit condition is a
  pressure reading; with no trustworthy readings it cannot be verified, so the controller
  de-energises instead of running a sequence it cannot observe.

### Fault injection

| `--fault` | Simulated hardware failure | Expected outcome (verified by tests) |
|---|---|---|
| `temp-runaway` | heater SSR welded on, ignores duty command | OverTemperature → Aborting; abort can't stop it → CriticalOverTemperature → **Fault**, contactor opens, chamber cools |
| `vacuum-leak` | chamber seal fails | OverPressureHeaterOn → Aborting → vented → Idle |
| `stuck-valve` | slit valve actuator fails open | SlitValveOpenUnderVacuum → Aborting → Idle |
| `sensor-dropout` | sensor thread stops publishing | StaleSensorData → **Fault** ~11 ms later |

---

## Design notes: the questions an interviewer will ask

### Timing

**Why `sleep_until` with an absolute deadline rather than `sleep_for`?**
Deadlines are fixed in advance: `deadline[n] = t0 + n × period`. With `sleep_for(1ms)` the
sleep starts whenever the previous tick *finished*, so wakeup latency and execution time
add to every period and are never paid back. The loop drifts slower than 1 kHz and the
error accumulates. With an absolute deadline a late tick is late by itself only, and the
next deadline doesn't move. `DeadlinesStayOnTheAbsoluteGrid` checks that every executed
deadline is an exact multiple of the period from t0.

**What happens on a missed deadline?** The loop counts every deadline that has passed and
skips to the next future grid point. Running the missed ticks back-to-back would execute
several control updates within microseconds on the same stale data, which is worse than
one honest, counted gap.

**What causes jitter, and why isn't this hard real-time?** Between "timer expired" and "our
thread runs" sit interrupt handling, other runnable threads, non-preemptible kernel
sections, timer slack, page faults, cache/TLB misses, SMIs, frequency/C-state changes,
and on a VM, the hypervisor descheduling the virtual CPU. None of these has a bound the
application can prove. Hard real-time needs an analysable worst case: an RTOS, or at
least `PREEMPT_RT` with isolated cores. Here jitter is minimised (SCHED_FIFO, 1 ns timer
slack, `mlockall`, optional pinning) and **measured**.

**Why percentiles, not an average?** The mean hides the rare late tick, and the rare late
tick is what matters to a controller. p99 = 100 µs means 1 tick in 100 was later than
that, which at 1 kHz is ten times a second. Percentiles come from an allocation-free 1 µs
histogram, reported as the bucket's upper edge (never under-reported), with the exact max.

### Concurrency

**What data is shared between threads, and what protects it?**

| Data | Writer → Reader | Protection |
|---|---|---|
| Sensor samples | sensor → control | `SeqlockRing`: per-slot seqlock, payload in atomic words |
| Actuator command | control → sensor/plant | one packed `std::atomic<uint64_t>`, relaxed |
| Log records | each thread → logger | one `SpscQueue` per producer, acquire/release indices |
| Operator commands | UI → control | `CommandMailbox` atomics, consumed with `exchange` |
| Live status | control → UI (+ sensor reads `state`) | `StatusBoard` atomics; `wafers_accepted` is a release/acquire publication point |
| Stop flags | UI → loops | `std::atomic<bool>`, relaxed |
| Histograms, metrics | loop thread → UI | **no atomics**: read only after `join()`, which is a happens-before edge |
| Signal flag | handler → UI | lock-free `std::atomic<int>` + self-pipe |

There are no mutexes anywhere on the control path.

**Why a ring buffer rather than a mutex-protected queue for sensor data?** A mutex lets a
low-priority thread holding the lock delay the control thread (priority inversion), and
`lock()` can enter the kernel. The seqlock ring never blocks either side and never makes
a system call. The trade-off is that it is lossy by design and harder to reason about,
hence the per-slot sequence numbers and an explicit loss count.

**What happens when the sensor thread outpaces the control thread?** Normally it does,
5:1, and the controller drains about 5 samples per tick (the PID uses their mean as a
filter). If the control thread is descheduled for more than 64 samples (12.8 ms), the
producer **overwrites the oldest** slots and the consumer counts exactly how many it
lost. That's the right behaviour for a sensor: the hardware can't wait for software, and
the newest reading is the most valuable one for control. Contrast the log queue, which
drops the *newest* record when full, because a log must not rewrite history.
`StalledControlThreadOverrunsAreCountedExactly` stalls control for 20 ms and checks the
count is exactly 41.

**Where is `std::atomic` used, and with what memory ordering?**
- Seqlock ring: `seq=odd` relaxed → payload words **release** → `seq=even` release; reader
  `seq` acquire → payload **acquire** → `seq` relaxed re-check. This is Boehm's fence-free
  formulation: if the reader sees any word of a newer write, it synchronises with that
  write and must then see the odd marker, so torn reads are always caught. No standalone
  fences, so ThreadSanitizer can verify every edge.
- SPSC queue: producer writes the slot then `head.store(release)`; consumer
  `head.load(acquire)` then reads the slot. Mirror image for `tail`.
- Actuator port, counters, stop flags, commands: **relaxed**. Each is self-contained data,
  not a flag guarding other data, so atomicity is enough.
- `StatusBoard::wafers_accepted`: **release** store last, **acquire** load first, so the UI
  can't see "all wafers accepted" together with a stale "Idle, nothing pending" from
  before they were picked up.

**Data race vs race condition? Does TSan catch both?** A *data race* is two unsynchronised
accesses to the same memory, at least one a write, at least one non-atomic. It's undefined
behaviour, and TSan detects it. A *race condition* is a logic bug where the outcome depends
on timing, even with every access properly atomic (for example, the UI reading `state` and
`wafers_pending` from different ticks). TSan cannot see those; they're prevented by design
(the publication point above) and by tests.

**Why must the control thread never take a lock the logging thread can hold?** The logger
does slow things (formatting, disk writes) at normal priority. If it held a lock the
control thread needed, the control thread would wait on a low-priority thread that the OS
may not even schedule: unbounded priority inversion. That's also why the logger isn't woken
with a condition variable (notifying requires the mutex); it polls every 5 ms instead.

### Memory

**Why is allocation forbidden in the control thread?** `malloc` can take a lock shared with
other threads, can call `mmap`/`brk` (system calls), can page-fault on fresh memory, and
has no bounded execution time. Any of those can blow a 1 ms deadline.

**Where does every heap allocation happen?** In `main`, before any real-time thread starts:
the `Logger` (three 160 KB queues), `Plant`, `SensorRing`, `ActuatorPort`, mailbox,
status board, `SensorTask`, `Controller`, both `PeriodicLoop`s (each with two 80 KB
histograms), the `std::promise` startup handshakes and the `std::thread`s. After that only
the UI and logging threads allocate (console strings, formatting). `mlockall(MCL_CURRENT)`
then pins everything that exists.

**Who owns what?** `main` owns everything through `std::unique_ptr`. Components hold
references to their collaborators (`Controller&` → `SensorRing&`, `ActuatorPort&`, ...);
references can't be null and don't imply ownership. Declaration order is construction
order, so destruction is reverse order: the threads are always joined before the objects
they reference are destroyed.

**What does RAII buy here?** `PeriodicLoop` and `Logger` join their threads in their
destructors, so no exit path (including exceptions) can leave a running thread or call
`std::terminate` on a joinable `std::thread`. The log file is an `std::ofstream` that
flushes and closes itself. There is no raw `new`/`delete` in the codebase.

### Design

**Why a state machine rather than nested conditionals?** Each state is one table row
(entry action, per-tick action, exit guard) plus a timeout. Every transition goes through a
single `go()` function, so logging, counting and entry actions can't be skipped, and the
current behaviour is always "the row for the current state", which you can test row by row.

**How do interlocks stay independent of recipe state, and why does that matter?** They live
in `safety/`, take only the sensor sample and the heater command, and run before the recipe
every tick. Their decision overrides the recipe via `force()`, and the recipe can't veto it.
If a protection were written inside the recipe states, a recipe bug (wrong state, missed
transition) could silently switch it off. `InterlocksFireRegardlessOfRecipeState` trips one
while Idle.

**What happens if a fault occurs during the abort sequence?** Abort-severity trips don't
restart an abort already in progress. A Fault-severity trip (critical temperature, stale
data) or the abort's own 5 s timeout *interrupts* it and drops to Fault, which opens the
heater contactor. `temp-runaway` demonstrates exactly this: abort commands the heater off,
the welded relay ignores it, temperature reaches 500 °C, and the Fault layer cuts the power.

**Why hysteresis and debounce?** Without hysteresis, sensor noise at 450 °C made the
over-temperature interlock trip and clear three times in one run (a test caught it). A
tripped interlock now clears only 5 °C below its limit. The stale-data check needs two
consecutive stale ticks, so one late *control* tick (the OS's fault, not the sensor's)
doesn't fault the tool, while a dead sensor still trips within 12 ms.

**What would change on a real RTOS?** Fixed-priority preemptive scheduling with bounded
interrupt and kernel latency, so worst-case response could be *analysed* rather than
measured. Priority-inheritance mutexes as standard, a hardware timer interrupt driving the
tick instead of `sleep_until`, static allocation everywhere (no `std::thread` or
`std::promise`), and real I/O: an ADC interrupt or DMA filling the ring buffer and
actuator writes going to memory-mapped registers. Safety-rated interlocks would also be
duplicated in hardware (a PLC or safety relay), not only in software.

---

## Verification

| Check | Result |
|---|---|
| Unit + system tests (GoogleTest) | 87 / 87 pass in Release, ASan/UBSan and TSan builds |
| End-to-end fault injection (`ctest -L e2e`, real binary, 4 threads) | 5 / 5 pass: every interlock fires, every run ends safe |
| ThreadSanitizer, 60 s run: 10 wafers queued, 29 live `status` reads, `quit` mid-Process | no reports |
| ASan + UBSan: nominal, all four faults, Ctrl-C | no errors, no leaks |
| Compiler warnings (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`, plus `-Wtsan`) | none |

What the tests cover: every guard at its boundary; every interlock predicate at its
boundary; hysteresis and debounce; the abort/fault decision matrix; every state
transition, timeout, abort, latch and reset path; ring-buffer ordering, exact overrun
counting and a 300 000-sample concurrent torn-read test; SPSC queue ordering and full
behaviour; PID clamping, anti-windup and closed-loop settling; plant physics; log
formatting and flush-on-stop; and closed-loop system runs (nominal and each fault) in
deterministic simulated time.

## Out of scope

GUI (console only), real hardware I/O, networking and multi-chamber coordination,
persistence beyond log files, Windows. Linux is the primary platform; macOS builds use
the Mach time-constraint policy instead of SCHED_FIFO (no `mlockall`, no CPU pinning).
