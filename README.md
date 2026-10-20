# Process Tool Controller

A C++17 control system for a simulated semiconductor process chamber: a fixed-tick
control loop, multi-threaded sensor acquisition, a recipe state machine, independent
safety interlocks, and non-blocking telemetry. The hardware is simulated; the control
software is real.

> **Status: Milestone 1 of 4 — skeleton that ticks.** The control thread, the
> absolute-deadline timing loop, jitter measurement, and clean shutdown are done and
> measured. The plant model, sensors, recipe, interlocks, and logging come in M2–M4.

**Soft real-time on a general-purpose OS.** This project does not claim hard real-time
guarantees. Every timing number below is a measurement of a specific run on a specific
machine, reported as percentiles, not a bound.

---

## Build

Linux only (uses `pthread_setschedparam`, `ppoll`, `prctl`). Requires CMake ≥ 3.20 and a
C++17 compiler. GoogleTest is fetched automatically if not installed.

```bash
# Release
cmake -S . -B build/release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/release
./build/release/ptc_tests

# ASan + UBSan
cmake -S . -B build/address -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPTC_SANITIZE=address
# TSan
cmake -S . -B build/thread  -G Ninja -DCMAKE_BUILD_TYPE=Debug -DPTC_SANITIZE=thread
```

## Run

```bash
./build/release/ptc                    # interactive: status | quit, Ctrl-C also quits
./build/release/ptc --duration 60      # headless 60 s run, then report
./build/release/ptc --duration 60 --cpu 1 --priority 80
```

| Flag | Meaning |
|---|---|
| `--duration <s>` | Run headless, then print the report |
| `--period-us <us>` | Tick period (default 1000) |
| `--cpu <n>` | Pin the control thread to one CPU |
| `--priority <1-99>` | SCHED_FIFO priority to request (default 80) |
| `--no-rt` | Don't request SCHED_FIFO (for comparison) |
| `--no-mlock` | Don't call `mlockall` |

SCHED_FIFO and `mlockall` need `CAP_SYS_NICE` / `CAP_IPC_LOCK` (or matching `ulimit -r`
/ `ulimit -l`). Without them the program still runs and the report says what was *not*
granted rather than hiding it.

---

## Measured results (M1)

Three 60-second Release runs, idle tick (measures the loop itself).
Machine: 2-vCPU Intel Xeon cloud VM (Firecracker), Linux 6.18, GCC 13.3.

| Configuration | p50 | p99 | p99.9 | max | missed deadlines |
|---|---|---|---|---|---|
| SCHED_OTHER (`--no-rt`) | 27 µs | 264 µs | 832 µs | 14.7 ms | 132 / 59 959 (0.22 %) |
| SCHED_FIFO 80 | 27 µs | 134 µs | 776 µs | 56.0 ms | 194 / 59 905 (0.32 %) |
| SCHED_FIFO 80, pinned to CPU 1 | 27 µs | 135 µs | 997 µs | 12.5 ms | 132 / 59 963 (0.22 %) |

All numbers are **wake lateness** = actual wake time − scheduled deadline. Tick execution
time was ≤ 3 µs at p99.9 in every run.

**What these numbers say:**

- **SCHED_FIFO halves p99** (264 → 134 µs): with a realtime policy the control thread
  pre-empts every normal thread the moment its timer fires, instead of waiting for the
  CFS scheduler to give it a turn.
- **The tail is not ours to fix on this machine.** The multi-millisecond maxima persist
  with SCHED_FIFO, pinning and `mlockall`. This is a virtual machine: when the hypervisor
  deschedules the vCPU (visible as nonzero `steal` in `/proc/stat`), no in-guest
  priority can run the thread. A bare-metal box with a `PREEMPT_RT` kernel and an
  isolated core would be expected to do far better — and that is exactly why the
  report gives percentiles rather than a claimed bound.
- **The ~27 µs floor** is timer interrupt + wakeup + context-switch cost on this VM
  (timer slack has already been cut from the 50 µs default to 1 ns; see below).

Run-to-run variation in the tail is large; treat p99.9 and max as "order of magnitude".

---

## Architecture (M1)

```
src/
  app/        main/UI thread: argument parsing, console, signal handling
  control/    ControlLoop: owns the control thread and the tick schedule
  platform/   SCHED_FIFO, CPU pinning, timer slack, mlockall (Linux specifics)
  telemetry/  LatencyHistogram (allocation-free), end-of-run report
tests/        GoogleTest: histogram math, tick schedule, overrun handling, lifecycle
```

Planned (per spec): `sim/` plant model, `io/` sensor thread + ring buffer, `recipe/`
state machine, `safety/` interlocks and abort sequencing.

**Threads in M1:** control thread + main/UI thread. (M2 adds the sensor thread, M4 the
logging thread — four in total.)

### Shared data in M1, and what protects it

| Data | Writer | Reader | Protection |
|---|---|---|---|
| `stop_requested_` | UI | control | `std::atomic<bool>`, relaxed |
| `ticks_`, `missed_`, `max_lateness_ns_` | control | UI (`status`) | `std::atomic`, relaxed |
| Histograms, run start/end | control | UI, **after join only** | `std::thread::join()` (happens-before) |
| `g_signalled` | signal handler | UI | lock-free `std::atomic<bool>` |

Relaxed ordering is enough everywhere because no atomic is used to *publish* other
data: each counter stands alone, and the stop flag only has to become visible
eventually (the loop polls it every tick). The non-atomic histograms are synchronised
by `join()`, which is a full happens-before edge — so the report reads them without
any atomics at all.

---

## Timing design

### `sleep_until` an absolute deadline, not `sleep_for`

Deadlines are fixed in advance on a grid: `deadline[n] = t0 + n × period`. Each tick
sleeps *until* its grid point.

With `sleep_for(1ms)` the sleep starts from *whenever the previous tick finished*, so
every bit of wakeup latency and tick execution time is added to the period and never
taken back: errors **accumulate** and the loop drifts slower than 1 kHz. With an
absolute deadline, a late tick is late by itself only — the next deadline does not
move, so errors **don't accumulate**. The test `DeadlinesStayOnTheAbsoluteGrid` checks
that every executed deadline is an exact multiple of the period from `t0`.

(Under the hood libstdc++ implements `sleep_until(steady_clock)` as "compute remaining
time, then sleep for it" — still relative at the syscall. That's fine: the target is
recomputed from the fixed deadline every tick, which is what prevents drift.)

### Missed deadlines: skip, don't burst

A deadline is *missed* when a tick finishes after the next tick should already have
started. The loop then counts every deadline that has passed and jumps to the next
future grid point. The alternative — running the missed ticks back-to-back to "catch
up" — would execute several control updates within microseconds on the same stale
sensor data, which is worse for a control loop than one honest, counted gap.
`OverrunsAreCountedAndSkippedNotBurst` verifies both the count and the phase.

### Why jitter exists here, and why this isn't hard real-time

Linux is a general-purpose OS. Between "timer expired" and "our thread runs" sit:
interrupt handling, other threads at equal or higher priority, kernel sections that
can't be preempted, timer slack, page faults, cache and TLB misses, SMIs, CPU
frequency/C-state transitions, and — on a VM — the hypervisor. None of these has a
bound the application can prove. Hard real-time requires a system where the
worst-case is *analysable*: an RTOS with fixed-priority preemptive scheduling and
bounded kernel paths, or at least `PREEMPT_RT` plus isolated cores and careful
measurement. Here we minimise jitter and measure it honestly.

### Percentiles, not averages

The mean (~37 µs) hides exactly what matters to a control loop: the rare late tick.
p99 = 134 µs means 99 % of ticks woke within 134 µs of their deadline — and 1 in 100
did not, which at 1 kHz is ten times a second. A controller is designed around the tail,
so the report gives p50 / p99 / p99.9 / max.

Percentiles come from a fixed 1 µs-resolution histogram (no allocation, O(1) record),
reported as the **upper edge** of the bucket (never under-reports; may over-report by
< 1 µs), and the max is tracked exactly.

### OS knobs applied (and reported)

- **SCHED_FIFO** priority 80 for the control thread.
- **Timer slack → 1 ns** (`PR_SET_TIMERSLACK`). Linux lets normal threads' timers fire up
  to 50 µs late to batch wakeups. Note: setting `0` *resets* it to the default — a trap.
- **`mlockall`** so the control path never takes a major page fault.
- **Optional CPU pinning.**

---

## Shutdown

`quit`, Ctrl-C (SIGINT), SIGTERM and `--duration` expiry all take the same path:
UI loop exits → `ControlLoop::stop()` sets the flag and joins → report printed from
the joined results. `ControlLoop`'s destructor also stops and joins, so an exception
can't leak a running thread (a joinable `std::thread` destructor would call
`std::terminate`).

Signals: SIGINT/SIGTERM are **blocked in every thread** before any thread is created,
and unblocked only inside `ppoll()` on the UI thread, atomically. So the handler never
interrupts the control thread mid-tick, and there is no window where a signal arrives
between checking the flag and going to sleep (the race `ppoll`'s mask argument exists
to close). The handler just sets a lock-free atomic.

## Memory in M1

Every heap allocation happens before the control thread starts: the `ControlLoop`
(which embeds both 80 KB histograms) via `std::make_unique` in `main`, the
`std::promise` used for the startup handshake, and the `std::thread`. The tick path
uses only members and stack scalars. Ownership: `main` owns the task and the loop
(`std::unique_ptr`); the loop holds a non-owning reference to the task, which is
declared first so it outlives the loop.

## Verification (M1)

| Check | Result |
|---|---|
| Unit tests (13) | pass in Release, ASan/UBSan and TSan builds |
| TSan, 60 s run with 20 concurrent `status` reads | no reports |
| ASan/UBSan, Ctrl-C and `quit` shutdown paths | no errors, no leaks, exit 0 |
| Compiler warnings (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`) | none |

## Out of scope

GUI (console only), real hardware I/O, networking / multi-chamber coordination,
persistence beyond log files, Windows/macOS.
