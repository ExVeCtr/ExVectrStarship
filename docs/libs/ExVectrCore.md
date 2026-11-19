# ExVectrCore
> Time base, cooperative scheduler, pub/sub topics, console printing and containers for ExVeCtr projects. Repo: https://github.com/ExVeCtr/ExVectrCore.git `Last verified against commit: 6fe3f8e (18bbb09 plus the ListBuffer overload fix; sections not about the NowNs rename or the additions below are as verified at a462465)`

## Purpose & scope
Foundation library of the ExVeCtr framework. Provides:
- A nanosecond `int64_t` time base (`NowNs()`), clock/time-source abstractions and date/time helpers.
- A single-threaded cooperative scheduler (`Scheduler`, `Scheduler::Task`, `Task_Periodic`).
- Synchronous pub/sub (`Topic<T>`, `Subscriber<T>` and ready-made subscribers).
- printf-style console output routed through topics (`printM/D/W/E`, `LOG_MSG`).
- Containers (`List` interface, `ListArray`, `ListBuffer`, `ListStatic`, `ListExtern`, `ListLinked`), `DataBuffer`, CRC-16, PRNG, `Message` interface.

Hardware-independent. A platform library must provide two symbols (README):
- `const Clock_Source& VCTR::Core::getPlatformClock()` (time_base.hpp:19). Must be fast; `NowNs()` calls it.
- `void VCTR::Core::initialise()` (ExVectrCore.hpp:21). Called first in `setup()`; sets up console output.

The README states it is under initial development and uses dynamic arrays (heap fragmentation), so it is not meant for critical long-life applications.

## Dependencies (ExVectr libs + external)
- ExVectr: none.
- External: C/C++ standard headers only (`stdint.h`, `stddef.h`, `stdarg.h`, `math.h`, `string.h`, `stdio.h` for `sscanf` in date_time.cpp). Bundles Marco Paland's tiny printf (MIT) inside src/print.cpp.
- Build: CMakeLists.txt globs `src/*.cpp`. There is no library.json; PlatformIO picks it up from `lib/` by default.

## Key concepts & API

### Time (time_definitions.hpp, time_base.hpp, clock_source.hpp, time_source.hpp, timestamped.hpp)
- Units: all times are `int64_t` nanoseconds. Constants: `NANOSECONDS=1`, `MICROSECONDS`, `MILLISECONDS`, `SECONDS`, `MINUTES`, `HOURS`, `DAYS`, `WEEKS`, `MONTHS` (30 d), `YEARS` (365 d). `END_OF_TIME = INT64_MAX`.
- `int64_t NowNs()`: returns `getPlatformClock().getCounter().data` (time_definitions.cpp:10-13), i.e. time since boot from the platform clock.
- `double NowS()`: `NowNs()` converted to seconds. (Renamed from `NOW()`/`NOWSeconds()` in 18bbb09; the old names no longer exist. `Time_Source::NOWSeconds()` kept its name.)
- `void delay(int64_t ns)`: busy-wait (time_definitions.cpp:20-25). Deprecated by its own doc comment; it blocks the scheduler.
- `class Clock_Source`: interface. `virtual const Timestamped<int64_t>& getCounter() const = 0;` returns a clock value in ns, stamped with the internal precise time. Also has a `Topic<Timestamped<int64_t>> timeTopic_` (`getTimeTopic()`) that external clocks (GNSS, RTC) publish to.
- `class Time_Source : Subscriber<Timestamped<int64_t>>`: an offset/factor view of `Core::NowNs()`.
  - `NowNs()` = `int64_t(Core::NowNs() * factor_) + offset_`.
  - `setTime(t)` sets the offset so that `NowNs() == t` at that instant.
  - `forceCorrect(src)` applies a one-shot offset from a clock source.
  - `setClockSource(src, correctionAmount)` subscribes to `src.getTimeTopic()`. Each received sample hard-sets the time (time_source.cpp:39-42).
  - `factor_` and `correctionLimit_` are never changed or used, so the documented slewing is not implemented.
- `template<T> struct Timestamped { T data; int64_t timestamp; }`: the timestamp defaults to 0. This is the standard payload wrapper for sensor topics.
- `TimedBool`: converts to `true` while `NowNs() < releaseTime`.
- `TimeSpan` (ns span with y/m/d accessors; months are 30 days) and `TimeDate` (RTClib port, 2000-based, no time zone/DST) in time_units.hpp / date_time.cpp.

### Added in 18bbb09 (not used by this project yet)
Present in the pinned Core but not referenced by `src/` (they are for the new ExVectrNetwork stack, see `../docs/network-design.md` in the workspace):
- `handler.hpp`: `HandlerGroup<Args...>` with `addHandler(std::function)`, `callHandlers(args...)`, `clearHandlers()`. Backed by a `ListArray` of `std::function` (heap, see the ExVectrNetwork heap notes).
- `IntervalTimer.hpp`: periodic/one-shot high/low phase timer with rising/falling edge callback (`start()`, `update()`, `sync()`, `adjustPhase()`, miss policy `FireAll`/`SkipMissed`). Driven by calling `update()` from a task; takes `int64_t` ns.
- `CanSerialize.hpp`: C++20 `concept` (`numBytes()`, `serialize()`, `deserialize()`). The Teensy build uses `-std=gnu++17`; including it compiles but warns (`-Wc++20-compat`: `concept`/`requires` are keywords in C++20). Probed 2026-11-19 together with handler.hpp and IntervalTimer.hpp: all three compile for Teensy 4.0.
- `ListArray`: `getPtr() const`, `pop()`, `pop(n)`, `popDiscard(n)`, `appendList()`.

### API changes in 18bbb09 (all mechanical for this project)
| Old | New |
|---|---|
| `Core::NOW()` | `Core::NowNs()` (same value: platform clock counter, int64 ns) |
| `Core::NOWSeconds()` | `Core::NowS()` |
| `Time_Source::NOW()` | `Time_Source::Now()` (`Time_Source::NOWSeconds()` unchanged) |
| `ListBuffer::getStandardDeviation(T avg = getAverage())` | Does not compile on arm-none-eabi-gcc (member call in default argument). Fixed on branch `feature/exvectrcore-update` (commit 6fe3f8e) with a no-argument overload plus the `(T average)` overload. Not yet on Core's `Development`. |
The scheduler, `Task_Periodic`, `TimedBool`, topic routers and print macros changed only by the rename and formatting; their logic is unchanged (checked with a whitespace-insensitive diff against a462465).

### Scheduler (scheduler2.hpp, scheduler2.cpp)
Threading model: one cooperative loop with no preemption and no RTOS. The app calls `getSystemScheduler().tick()` from `loop()`. Each `tick()` runs at most one task.

`class Scheduler::Task` (abstract, intrusively linked, so adding it allocates nothing):
- Override `virtual void taskThread() = 0`. Optional overrides: `taskInit()` (called once before the first run), `taskCheck()` (called on every tick, before selection, so a task can change its own release time).
- `virtual void taskRun()` is the hook for task types (`Task_Periodic` overrides it). It is not meant for application code.
- Timing window: `setRelease(t)` / `getRelease()` set the earliest run time and `setDeadline(t)` / `getDeadline()` the latest. Release is always kept ≤ deadline: each setter pushes the other one along.
- `setPriority(uint16_t)` (default 100), `setPaused(bool)`, `setInitialised(bool)`, `setAllowSleep(bool)`.
- Statistics: `getRate()` (Hz, recomputed every 5 s) and `getRuntime()` (ns, exponential moving average with α=0.02).
- Name: `char taskName_[50]` (49 characters plus NUL). The `Task_Periodic` doc says "up to 20 chars"; 49 is the real limit.
- Lifetime: `~Task()` removes the task from its scheduler. `removeFromScheduler()` / `getScheduler()` are available. The scheduler only stores pointers, so tasks must outlive their registration.

`class Scheduler`:
- `bool addTask(Task&)`: appends the task's embedded list node and always returns true. Adding the same task twice is not guarded against.
- `bool removeTask(Task&)`: unlinks the task (see the `ListLinked::remove` issue below).
- `const List<Task*>& getTasks() const`, `int64_t getNextTaskRelease() const`.
- `void setSleepFunction(void(*)(int64_t ns))`: optional. When no task is due, every task allows sleep, and the next release is more than `sleepMargin_ + minSleepTime_` (1 ms + 1 ms) away, the scheduler calls `sleep(next - NowNs() - 1 ms)`. `maxSleepTime_` (100 ms) is declared but not used.
- `Scheduler(Clock_Source&)` stores a `Time_Source`, but `tick()` always uses `Core::NowNs()`.
- `getSystemScheduler()`: a function-local static, so it is safe to use from global constructors in any translation unit.

`tick()` algorithm (scheduler2.cpp:120-225):
1. For every task: call `taskCheck()`, then compute the pseudo-priority `misses*10 + priority` (scheduler2.cpp:95).
2. For every unpaused task with `NowNs() > release`: increment `misses` and track the highest pseudo-priority.
3. Run the winner: reset `misses = 0`, call `taskInit()` if not yet initialised (the init flag is set before the call, so `taskInit` may clear it to signal failure), then `taskRun()`, then update runtime and rate.
4. If nothing was due, consider sleeping (see `setSleepFunction`).

Deadlines do not affect selection; only release time, priority and miss count do. A task that is starved for longer than another accumulates misses and eventually wins.

`class Task_Periodic : Scheduler::Task` (task_types.hpp/.cpp):
- `Task_Periodic(const char* name, int64_t interval_ns, int64_t start = NowNs(), int64_t timeSlip_ns = 1 ms, bool skipOverdueRun = true)`.
- `taskRun()` (final) computes the next release and then calls `taskThread()`:
  - With `skipOverdueRun`, the next release is the next multiple of `interval` counted from absolute 0 (not from `start`).
  - Otherwise, `deadline_ += interval`.
- `setInterval()` / `getInterval()`.

ISR safety: none. Do not call scheduler methods from interrupts.

### Pub/sub (topic.hpp, topic_subscribers.hpp, topic_routers.hpp)
- `template<T> class Topic final`: `publish(const T&)` synchronously calls `receive(item, this)` on every subscriber with receiving enabled, in subscription order and in the caller's context (task or ISR). It holds subscriber pointers in a `ListArray` (heap). `getSubscriberList()`, `unsubscribeAll()`. The destructor unsubscribes everyone.
- `template<T> class Subscriber` (abstract): implement `virtual void receive(const T&, const Topic<T>*) = 0`.
  - `subscribe(Topic&)`: a subscriber is attached to at most one topic; subscribing again first unsubscribes from the previous one.
  - `unsubscribe()`, `receiveEnable(bool)`, `isReceiveEnabled()`.
  - `publish(const T&)` publishes to the subscribed topic but skips this subscriber (used for bidirectional links).
  - The destructor unsubscribes.
- Ready-made subscribers:
  - `Simple_Subscriber<T>`: keeps the latest item. `isDataNew()`, `getItem()` (clears the new flag).
  - `Buffer_Subscriber<T,N>`: is also a `ListBuffer<T,N>`; each received item goes through `placeFront(item, overwrite_)`.
  - `Callback_Subscriber<T,C>(topic, C* obj, void (C::*)(const T&))`: calls a member function.
  - `StaticCallback_Subscriber<T>(topic, void(*)(const T&))`: calls a free function.
  - `Topic_Publisher<T>(topic)`: publish-only; its `receive` does nothing.
  - `Topic_Switch<T>`: re-publishes everything to its internal `getTopic()`. Re-subscribe the switch to change the source; subscribers of the internal topic stay attached.
- `TopicRouter` (topic_routers.hpp) is broken and does not compile (see Known issues).
- Payloads are copied by value into receivers that store them. `Topic<const char*>` passes a pointer that is only valid during `receive`.

### Printing (print.hpp, print.cpp)
- `printM` (message), `printD` (debug), `printW` (warning), `printE` (error), and `printTopic(Topic<const char*>&, fmt, ...)`.
- Each one sets a module-global `currentTopic`, formats with the bundled `vprintf`, and lets `_putchar` collect characters into a static `ListArray<char>`. On the terminating NUL it publishes the string to the topic.
- Topics are function-local statics: `getMessageTopic()`, `getDebugTopic()`, `getWarningTopic()`, `getErrorTopic()`. The platform subscribes an output (e.g. Serial) to them. If nothing is subscribed, output is dropped.
- `LOG_MSG(...)`: enabled by `#define EXVECTR_DEBUG_ENABLE` at print.hpp:4. Prints `"<sec> LOG from file, line, func: "` and then the message, on the debug topic. It uses unqualified `Core::NowS()`, so it only compiles where `VCTR` is in scope (e.g. after `using namespace VCTR`).
- `VRBS_MSG(...)`: enabled by `EXVECTR_DEBUG_VRBS_ENABLE` (print.hpp:5, commented out). The scheduler uses it per task run, so enabling it is very noisy.
- Not reentrant and not ISR-safe; it allocates on the heap.
- print.cpp defines global `printf`, `sprintf`, `snprintf`, `vprintf`, `vsnprintf` and `fctprintf`, replacing newlib's versions for the whole firmware.

### Containers (list*.hpp, data_buffer.hpp)
- `List<T>` (interface): `size()`, `operator[]` (unchecked), `operator()(int32_t)` (wrapping; negative indexes count from the end), element-wise `operator=`, and `static const List<T>& empty()`.
- `ListArray<T>(bool sizeControl=false)`: heap array that doubles when full and starts with capacity 1.
  - `append`, `appendIfNotInListArray`, `insert`, `removeAtIndex`, `removeWithPointer`, `removeAllEqual`, `clear`, `setSize`, `reduceSize`, `getPtr`.
  - O(1) access, O(n) insert/remove. Copyable (deep copy).
- `ListBuffer<T,N>`: statically allocated ring buffer usable as queue or stack.
  - `placeFront`/`placeBack(item, overwrite)`, `takeFront`/`takeBack`, `peekFront`/`peekBack`, `removeFront`/`removeBack(n)`, `removeElement`, `removeElementIndex`, `clear`.
  - Statistics: `getSum`, `getAverage`, `getMedian` (sorts a copy), `getStandardDeviation`, `getStandardError`, `sortElements` (quicksort in place).
  - `operator[](0)` is the element most recently placed at the front.
- `ListStatic<T,N>`: fixed array; `size()` is always `N`.
- `ListExtern<T>(T* ptr, size_t n)`: non-owning view over external memory.
- `ListLinked<T>`: one node of a doubly linked list; indexing is relative to the node it is called on. Used intrusively by the scheduler.
- `DataBuffer<N>`: byte serialiser with a cursor.
  - `write<T>` / `read<T>` copy raw bytes. `writeF`/`writeD(value, factor)` store `value/factor`; `readF`/`readD` multiply back.
  - Writes and reads past `N` are silently truncated.
- `computeCrc(buf, len, init)` / `computeCrc(const List<uint8_t>&, init)`: CRC-16-CCITT, polynomial 0x1021, MSB first, returned in a `uint32_t`.
- `randGen(max, seed)`: Lehmer/MINSTD-style generator with static state. `randNorm(sigma, mu)`: Box–Muller using π≈3.14.
- `Message`: abstract interface (type, data type, length, get/set data) with the `Mes_Type` and `Data_Type` enums.

## Usage
```cpp
#include "ExVectrCore.hpp"
#include "ExVectrCore/topic_subscribers.hpp"
using namespace VCTR;

Core::Topic<Core::Timestamped<float>> altitudeTopic;

class AltitudeTask : public Core::Task_Periodic {
  Core::Simple_Subscriber<Core::Timestamped<float>> altSub_;
public:
  AltitudeTask() : Task_Periodic("Altitude", 20 * Core::MILLISECONDS) {
    Core::getSystemScheduler().addTask(*this);   // pattern used throughout src/
    altSub_.subscribe(altitudeTopic);
  }
  void taskInit() override { /* one-time hw init; setInitialised(false) to retry */ }
  void taskThread() override {
    if (altSub_.isDataNew()) {
      auto a = altSub_.getItem();
      LOG_MSG("alt %.2f at %.3fs\n", a.data, double(a.timestamp) / Core::SECONDS);
    }
  }
} altitudeTask;                                   // global, like in main.cpp

void onAlt(const Core::Timestamped<float>& a) { /* ... */ }
Core::StaticCallback_Subscriber<Core::Timestamped<float>> altCb(altitudeTopic, onAlt);

void setup() { Core::initialise(); }
void loop()  { Core::getSystemScheduler().tick(); }
```
Producer side: `altitudeTopic.publish(Core::Timestamped<float>(alt, Core::NowNs()));`

## Conventions
- Namespace `VCTR::Core`. Headers live under `include/ExVectrCore/`; the umbrella header `ExVectrCore.hpp` pulls in print, time definitions, scheduler, task types, topic and topic subscribers.
- Class names: `Upper_Snake` for framework types (`Task_Periodic`, `Clock_Source`, `Simple_Subscriber`) and CamelCase for others (`ListArray`, `TimedBool`). Members have a trailing underscore. Task hooks use a `task` prefix (`taskInit`, `taskThread`, `taskRun`, `taskCheck`).
- Times are `int64_t` ns. Build values from the unit constants (`20 * Core::MILLISECONDS`).
- Ownership: everything takes references and stores raw pointers. Tasks and subscribers unregister themselves in their destructors. Objects are usually globals or function-local statics. Heap use: `ListArray` (topic subscriber lists, print buffer, user lists).
- Platform hooks are declared `extern` in Core and defined in the platform library.

## How this project uses it
- Tasks: every subsystem derives from `Core::Task_Periodic` and calls `getSystemScheduler().addTask(*this)` in its constructor. Examples:
  - `src/starship_tvc.hpp:22`, `src/starship_flaps.hpp:33`, `src/ublox_gnss.hpp:21`, `src/datalink_sx1280.hpp:27`.
  - In `src/main.cpp`: `MagnetometerCalibrationTask` :337, `VehicleSafetyAndControlTask` :456, `FlapModeObserverTask` :1258, `CommsSystemStateTask` :1337, `DisplayTask` :1394.
- `main.cpp:1982` calls `Core::initialise()` in `setup()`; `main.cpp:1991` has `loop()` call `getSystemScheduler().tick()`.
- Priorities and intervals are set in `initialiseHardware()` (`main.cpp:1757-1763`): the MPU driver runs at 250 µs with priority 110, imu/posEst tasks at priority 105.
- `setPaused()` is used for on-demand tasks: `ublox_gnss.cpp:67,115,165`, `datalink_sx1280.cpp:58,68,78`, `main.cpp:357-412`.
- `removeTask(*this)` is called from `DisplayTask::taskInit` (`main.cpp:1426`).
- Topics:
  - `Core::Topic_Switch<Core::Timestamped<Math::Vector<...>>>` selects sensor and simulator sources (`main.cpp:182-186`).
  - Many `StaticCallback_Subscriber`s (`main.cpp:218-333`), `Buffer_Subscriber<…,10>` (`main.cpp:339`), `Topic_Publisher` (`starship_flaps.hpp:40`), `Simple_Subscriber`s in the TVC, flaps and missions code.
- `Core::Time_Source missionTime_` holds mission time (`src/mission/mission_abstract.hpp:34`), set via `setTime()` in `mission_freefall.hpp:69` and `mission_rth.hpp:129`.
- Containers:
  - `Core::ListBuffer<uint8_t, …>` is the datalink TX queue (`datalink_sx1280.hpp:46`; it uses `placeBack` and `removeFront`).
  - `Core::ListArray` holds the mission list (`main.cpp:487`) and packet buffers (`datalink_sx1280.cpp:379,398`).
  - `gyroBiasBuffer.placeBack(…, true)` at `main.cpp:1708`.
- `LOG_MSG` / `printM` are used throughout for diagnostics.

## Known issues & limitations
| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | include/ExVectrCore/list_linked.hpp:273-279 | `remove()` sets `next->next_ = nullptr` before testing it, so the successor's `prev_` is never updated and still points at the removed node. | After `Scheduler::removeTask` (e.g. main.cpp:1426), removing or destroying the successor goes through a stale `prev_`. The successor then stays linked in the scheduler list (it can still run after "removal"), or memory is written through a dangling pointer if the removed node no longer exists. |
| High | src/print.cpp:1195-1232 (decls :53-93) | Defines global `printf`/`sprintf`/`snprintf`/`vprintf`/`vsnprintf`, overriding newlib for the whole firmware. Plain `printf` output goes to `currentTopic` (usually null), so it is discarded. | Third-party code calling `snprintf` silently gets the tiny-printf implementation (floats above `PRINTF_MAX_FLOAT`=1e9 (:282) switch to exponent notation, etc.). `printf` output is lost. |
| Med | src/task_types.cpp:10-18 (member task_types.hpp:40) | The `timeSlip_ns` constructor argument is never stored, so `timeSlip_ns_` stays 0. `offset_` (=start) is never used. | The deadline always equals the release; the documented slip has no effect. |
| Med | src/task_types.cpp:32-35 | With `skipOverdueRun=false`, `deadline_` starts at 0 (not `start`), so the task runs back-to-back until it has caught up from t=0. With skip enabled, releases align to multiples of `interval` from absolute 0, not from `start`, and `NowNs()` is called twice. | Bursts of runs at startup for non-skipping tasks; phase offsets via `start` are lost after the first run. |
| Med | include/ExVectrCore/scheduler2.hpp:203 | `tasks_` has no initializer. | Only zero-initialized for static or global `Scheduler`s (e.g. `getSystemScheduler()`). Any automatic or heap `Scheduler` dereferences garbage in `addTask` or `tick`. |
| Med | src/scheduler2.cpp:120-225, scheduler2.hpp:207 | `tick()` always uses `Core::NowNs()`. The `timeSource_` member (and the `Scheduler(Clock_Source&)` constructor) has no effect, although the header comment marks this feature "Finished". `maxSleepTime_` is unused. | A misleading API; the sleep length is not capped. |
| Med | src/time_source.cpp:15 | `Core::NowNs() * factor_` is an `int64 * float` multiplication, so the result is computed in `float` (24-bit mantissa). | `Time_Source::Now()` is quantised: about 65 µs steps after ~18 min, about 262 µs after ~1 h. Affects mission time (`mission_abstract.hpp:34`). |
| Med | src/time_source.cpp:39-42, time_source.hpp:28-30 | `receive()` hard-sets the time; `factor_` and `correctionLimit_` are never used. | Slewing correction as documented is not implemented; time jumps on every external clock sample (possibly backwards). |
| Med | include/ExVectrCore/list_buffer.hpp:492 | `peekFront` reads `(front_+1)%SIZE`; `takeFront` reads `front_-1`. | `peekFront` returns the wrong element (often stale data). |
| Med | include/ExVectrCore/list_buffer.hpp:396-402 | `removeElementIndex` shifts elements through `operator[]` and decrements the count without adjusting `back_`. | After a removal, `peekBack`/`takeBack`/`placeBack` see a corrupted buffer. Also reachable via `removeElement`. |
| Med | include/ExVectrCore/list.hpp:146 | The const `operator()(int32_t)` calls itself. | Infinite recursion (stack overflow) when used on a const `List` (e.g. `ListArray`, `ListStatic`, `ListExtern`, which don't override it). |
| Med | include/ExVectrCore/topic_routers.hpp:26,64,88 | Inherits the nonexistent `Subscriber_Interface<TYPE>`. The `receive` signature does not match the base. Compares `Topic*` with `const Topic**`. Calls the private `Topic::publish(item, sub)`. | The header fails to compile when included; `TopicRouter` cannot be used. Currently not included anywhere. |
| Med | include/ExVectrCore/topic.hpp:195-200 | `publish` loops over `subscribers_` by index while `receive` may subscribe or unsubscribe (which reallocates or shifts the `ListArray`). No locking. | Items can be skipped or delivered twice when receivers change subscriptions. Publishing from an ISR (e.g. the MPU interrupt path) races with task-context publishes and subscription changes. |
| Med | src/print.cpp:98-114,143-200 | A module-global `currentTopic` plus a static heap `ListArray<char>`. | Not reentrant: a print issued from inside a print-topic subscriber, or from an ISR during a print, corrupts or misroutes output. Each print allocates on the heap. |
| Low | include/ExVectrCore/list_array.hpp:179 | `reduceSize()` shrinks the capacity to `size_/2` instead of `maxSize_/2`. | Truncates and loses half the stored elements. Not called by this project. |
| Low | include/ExVectrCore/list_array.hpp:357-361 | `removeAllEqual` increments `i` after `removeAtIndex(i)`. | Adjacent duplicates are missed. Harmless for topics (entries are unique). |
| Low | include/ExVectrCore/list.hpp:140 | `operator()` computes `index % len` with no check for `len == 0`. | Division by zero on an empty list. |
| Low | include/ExVectrCore/list_buffer.hpp:189-214 (docs) | The doc comment says index 0 is the oldest element; it is actually the newest front element. | Misleading API documentation. |
| Low | include/ExVectrCore/topic_subscribers.hpp:116 | `Buffer_Subscriber(bool overwrite)` ignores its argument. | Overwrite mode silently stays off unless `setOverwrite()` is called. |
| Low | src/date_time.cpp:107-135 | The `TimeDate`/`TimeSpan` operators are defined in the global namespace but declared in `VCTR::Core` (time_units.hpp:103-111). | Undefined references at link time if used. |
| Low | src/date_time.cpp:63-65 | `sscanf("%s")` writes into a 4-byte static buffer. | Overflow on malformed date strings. |
| Low | include/ExVectrCore/data_buffer.hpp:157,192 | Uses `memcpy` without including `string.h`/`cstring`. | Compiles only if an earlier include provides it. |
| Low | include/ExVectrCore/print.hpp:72,82 | `LOG_MSG`/`VRBS_MSG` use unqualified `Core::NowS()`. `EXVECTR_DEBUG_ENABLE` is hard-coded on (:4). | Does not compile outside `namespace VCTR` or `using namespace VCTR`. Logging cannot be disabled without editing the library. |
| Low | src/time_definitions.cpp:20-25 | `delay()` busy-waits. | Blocks all tasks; deprecated per its own comment. |
| Low | src/random.cpp:9-19, :34 | Static PRNG state; a nonzero seed is added on every call; π approximated as 3.14. | Weak, non-reentrant randomness; slight bias in `randNorm`. |
| Low | include/ExVectrCore/task_types.hpp:50 vs scheduler2.hpp:60 | Name length documented as 20 chars; the buffer actually holds 49 + NUL. | Documentation mismatch. |
| Low | README.md | Lists documentation, simulation and vehicle library as unfinished (to-do items). | Project is in initial development. |

## Extending
- **New task:** derive from `Core::Task_Periodic` (or from `Scheduler::Task` for event-driven tasks that manage `setRelease()` themselves, e.g. set the release to `NowNs()` when data arrives and to `END_OF_TIME` when idle, as `PlatformArduino` does).
  - Call `getSystemScheduler().addTask(*this)` in the constructor.
  - Put hardware init in `taskInit()`; call `setInitialised(false)` inside it to retry on the next tick.
  - Keep `taskThread()` short: one long task delays all others.
- **New task type:** derive from `Scheduler::Task` and override `taskRun()` (plan the next release, then call `taskThread()`) and/or `taskCheck()` (runs on every tick).
- **New topic payload:** any copyable type works as `Topic<T>`. Wrap sensor data in `Timestamped<T>` and stamp it with `Core::NowNs()` at acquisition.
- **New subscriber kind:** derive from `Subscriber<T>` and implement `receive(const T&, const Topic<T>*)`. Keep it cheap and non-blocking; it runs in the publisher's context.
- **New container:** derive from `List<T>`; implement `size()` and both `operator[]`. Override `operator()` const (the base version recurses).
- **New platform:** implement `getPlatformClock()` (return a static `Clock_Source` whose `getCounter()` is monotonic ns since boot) and `initialise()` (subscribe console output to the four print topics). See ExVectrArduinoPlatform `src/time_implementation.cpp` and `src/print_implementation.cpp`.
