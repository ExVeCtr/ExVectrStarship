# ExVectrData
> Byte-addressable memory abstraction plus a stateless key/value allocator (`Memory_Manager`) on top of it. Repo: https://github.com/ExVeCtr/ExVectrData.git `Last verified against commit: 283596f`

## Purpose & scope
Storage only. It provides:
- `Memory_Interface`: a generic byte memory (read/write/transfer/size) with a cursor and typed helpers.
- Backends: `Memory_Internal<SIZE>` (RAM array) and `Memory_AT24CX` (I2C EEPROM, unusable, see Known issues). The Arduino EEPROM backend (`Memory_ArduinoEEPROM`) lives in ExVectrArduinoPlatform, not here.
- `Memory_Manager`: allocates keyed items in any `Memory_Interface`, so typed structs (calibrations etc.) can be stored and found again after reboot.
- `Memory_Buffer`: a primary/secondary sync helper. Its header says "DO NOT USE"; it is to be replaced by a cache class.

Out of scope despite the name: no logging, no parameter registry, no mission or state framework. Missions live in the application (`src/mission/`, `MissionAbstract`).

README status: "under initial development"; the roadmap items (architecture, docs) are unchecked.

Superproject state at time of writing: the submodule tree is clean (on `Development`, in sync with origin). The parent repo shows `M lib/ExVectrData` only because the recorded submodule pointer is 0e00d87 while the checkout is 283596f ("Changed include guard"). That pointer bump is not committed in the parent.

## Dependencies (ExVectr libs + external)
- ExVectrCore: `print.hpp` (`LOG_MSG`) in memory_manager.cpp and at24cx.cpp.
- ExVectrHAL: `HAL::DigitalIO` in at24cx only.
- README also lists ExVectrMath, but nothing includes it (it is commented out in CMakeLists.txt too).
- External: `stdint.h` and `stddef.h`. `memcpy` is used in memory_internal.hpp without including `<string.h>`; it builds only because Arduino headers bring it in.
- Build: CMakeLists.txt (globs `src/*.cpp`). There is no library.json. PlatformIO compiles every source file, including at24cx.cpp.

## Key concepts & API

### Memory_Interface (include/ExVectrData/memory_interface.hpp)
```cpp
virtual size_t readMem(uint8_t* buf, size_t n, size_t index, size_t bufferIndex = 0) = 0;
virtual size_t writeMem(const uint8_t* buf, size_t n, size_t index, size_t bufferIndex = 0) = 0;
virtual size_t transferFrom(Memory_Interface& src, size_t n = SIZE_MAX, size_t toIndex = 0, size_t fromIndex = 0) = 0;
virtual size_t size() = 0;                       // capacity in bytes, not usage
size_t accessIndex(size_t index = SIZE_MAX);     // get/set cursor
template<T> size_t readMem(T& v, size_t index = SIZE_MAX);        // raw sizeof(T) copy, advances cursor
template<T> size_t writeMem(const T& v, size_t index = SIZE_MAX); // raw sizeof(T) copy, advances cursor
```
- Every call returns the number of bytes actually transferred. Backends clip the count at the memory bounds instead of failing.
- The typed helpers do a raw byte copy, so stored data is native little-endian (ARM) with no padding control. Stored structs must be trivially copyable, and their layout must not change between firmware builds.
- The cursor `accessIndex_` is shared mutable state, so a memory is not reentrant. Don't use one from an ISR and a task at the same time.

### Memory_Internal\<SIZE\> (include/ExVectrData/memory_internal.hpp)
- A plain `uint8_t[SIZE]` array; reads and writes are `memcpy` calls.
- `transferFrom(src, n = SIZE, ...)` reads straight from `src` into the array.
- `getMemoryPointer<T>(index)` gives direct access to the array.

### Memory_AT24CX (include/ExVectrData/external_chips/at24cx.hpp)
- Constructor takes `(memorySize, pageSize = 32)`, then call `initMemory(HAL::DigitalIO& i2c)`. The bus must report `BUS_I2C` for both input and output, and the device address comes from the bus object (`AT24CX_ID` = 0x50 is never used).
- Not functional, see Known issues.

### Memory_Manager (include/ExVectrData/memory_manager.hpp, src/memory_manager.cpp)
- Stateless: all state is in the managed memory, so several managers can share one memory.
- Slower than direct access: every call walks the item list from the start, reading headers through the memory interface.
- API:
  - `uint32_t allocateItem<T>(const T& v = T(), uint32_t key = 0)`
  - `void deallocateItem(uint32_t key)`
  - `bool readItem<T>(T&, key)`
  - `bool writeItem<T>(const T&, key)`
  - `uint32_t getItemDataIndex(key, numBytes = 0)`
  - `void clearMemory()`: O(1). It rewrites the header only and does not erase data.
  - `getManagerVersion()` (compiled `VERSION_` = 1) and `getMemoryVersion()` (version stored in memory; 0 if the header is invalid).
- An item is found by key and by size: `readItem`/`writeItem` match only an item whose stored size equals `sizeof(T)`. Changing a stored struct's size makes the old item invisible, and a new allocation with the same key appends a duplicate.
- Allocation always appends after the last item. Freed gaps between items are never reused (`compressMemory()` and `usedMemory()` are commented out). Freeing the last item does make its space reusable.
- Nothing checks that a new item fits in `memory.size()`.

Memory layout (all multi-byte fields little-endian, as written by `writeMem<T>`):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Magic `0xABFA` |
| 2 | 2 | Manager version (`VERSION_`, currently 1) |
| 4 | 2 | Allocated item count (used only as a loop guard) |
| 6 | 4 | Index of first item (0 = empty) |
| 10 | 10 | Unused |
| 20 | ... | First item. Allocation starts here. |

Item layout (at `index`, items linked through `next`):

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Magic `0xEFA6` (validated only for the item at offset 20) |
| 2 | 4 | Index of next item (0 = last) |
| 6 | 2 | Key (uint16; the API's uint32 key is truncated) |
| 8 | 2 | Data size in bytes |
| 10 | n | Data |

With `key == 0`, `allocateSpace` uses the item's own index as its key.

## Usage
Boot sequence as in `src/main.cpp:1775-1871` (EEPROM mirrored in RAM, manager runs on the RAM copy):
```cpp
Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length()); // ExVectrArduinoPlatform
Data::Memory_Internal<512> internalMemory;
Data::Memory_Manager memoryManager(internalMemory);

EEPROM.begin();
internalMemory.transferFrom(eeprom);                         // load image
if (memoryManager.getManagerVersion() != memoryManager.getMemoryVersion())
  memoryManager.clearMemory();                               // fresh/foreign image

DSP::ValueCov<float, 3> cal;
if (!memoryManager.readItem(cal, MEMORY_KEY_MAGCALIB)) {
  cal.val = {0, 0, 0}; cal.cov = Math::Matrix<float,3,3>::eye();
  memoryManager.allocateItem(cal, MEMORY_KEY_MAGCALIB);      // always pass an explicit non-zero key
  memoryManager.writeItem(cal, MEMORY_KEY_MAGCALIB);
}
eeprom.transferFrom(internalMemory);                         // persist (EEPROM backend writes only changed bytes)
```

## Conventions
- Always use explicit, non-zero, application-wide unique keys below 65536, kept in one header (`src/memory_keys.hpp`). Don't rely on auto-keys (key 0).
- Check `getMemoryVersion()` against `getManagerVersion()` before use; on a mismatch every operation fails until `clearMemory()`.
- Store fixed-layout POD types (Math matrices/vectors, `DSP::ValueCov`). Treat a size change as a new key.
- Work on a RAM mirror (`Memory_Internal`) and persist to EEPROM explicitly. This limits EEPROM wear and keeps the slow list walks off flash.
- Namespace `VCTR::Data`. Include as `ExVectrData/<file>.hpp`. The umbrella `ExVectrData.hpp` is empty.

## How this project uses it
- `src/main.cpp:105-107`: `Memory_ArduinoEEPROM eeprom` (Teensy emulated EEPROM), `Memory_Internal<512> internalMemory`, and `Memory_Manager memoryManager(internalMemory)`.
- `initialiseMemory()` (`src/main.cpp:1775-1918`):
  - Loads EEPROM into RAM.
  - Checks the version and clears on a mismatch.
  - Reads or creates 6 items: three `DSP::ValueCov<float,3>` calibrations (48 B each) and three `Math::Matrix<float,3,3>` transforms (36 B each). Total is about 332 B including headers, well under 512.
  - Persists with `eeprom.transferFrom(internalMemory)` (:1871, :1916).
- Telecommand Save (`src/main.cpp:808-812`) persists RAM to EEPROM. The mag calibration flow writes `MEMORY_KEY_MAGCALIB` (:769) and reads `MEMORY_KEY_MAGTRANSFORM` (:787).
- Memory keys (`src/memory_keys.hpp`):

| Key | Value | Type stored |
|---|---|---|
| `MEMORY_KEY_GYROCALIB` | 100 | `DSP::ValueCov<float,3>` (bias in val, scale in cov) |
| `MEMORY_KEY_MAGCALIB` | 101 | `DSP::ValueCov<float,3>` |
| `MEMORY_KEY_ACCCALIB` | 102 | `DSP::ValueCov<float,3>` |
| `MEMORY_KEY_GYROTRANSFORM` | 200 | `Math::Matrix<float,3,3>` sensor-to-body |
| `MEMORY_KEY_MAGTRANSFORM` | 201 | `Math::Matrix<float,3,3>` |
| `MEMORY_KEY_ACCTRANSFORM` | 202 | `Math::Matrix<float,3,3>` |

- Not used: `Memory_AT24CX`, `Memory_Buffer`, `deallocateItem`, `getItemDataIndex`.

## Known issues & limitations
Paths are relative to `lib/ExVectrData/` unless noted.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Med | include/ExVectrData/memory_manager.hpp:41-48 | `allocateItem` ignores the key returned by `allocateSpace` and returns the `key` argument. With `key == 0` it returns 0 and the internal `writeItem(value, 0)` fails. Allocation failure is also reported as success when the key is non-zero. | Auto-keys are unusable (the item exists but its key is lost) and the caller can't detect a failed allocation. Explicit keys, as used in main.cpp, work. |
| Med | src/memory_manager.cpp:209-213, 247-255 | The free-space search returns "end of last item" without checking that `index + 10 + size <= memory.size()`. | Over-allocation writes a truncated header/data (the backend clips silently) and the item is still linked, so the list can be corrupted. |
| Low | src/memory_manager.cpp:254 | Key is stored as `uint16_t`, but the API takes `uint32_t`. | Keys ≥ 65536 alias. |
| Low | src/memory_manager.cpp:59-66 | Freeing the first item returns early without decrementing the item count at offset 4. | The count drifts upward; it is only a loop guard, so the effect is cosmetic. |
| Low | src/memory_manager.cpp:199 | The item magic `0xEFA6` is checked only for the item at index 20. | Corrupt links elsewhere are followed blindly (bounded by `allocatedItems` and `memory.size()`). |
| Low | include/ExVectrData/memory_manager.hpp:108-112 | `getItemDataIndex` returns `foundIndex + 10`, so it returns 10 (not 0) when the key isn't found. | Callers can't detect "not found"; using 10 as a pointer reads into the header. |
| Low | include/ExVectrData/memory_manager.hpp:119, 129 | `compressMemory()` and `usedMemory()` are declared only as comments. | Holes left by middle deallocations are never reclaimed. |
| Med | include/ExVectrData/memory_internal.hpp:46, 61 | `numBytes = SIZE - index` underflows when `index > SIZE`. | An out-of-range access becomes a huge `memcpy` out of bounds. |
| Low | include/ExVectrData/memory_internal.hpp:47, 62 | `bufferIndex` is bounds-checked against the memory `SIZE`, but the caller's buffer size is unknown. | Reads or writes can be wrongly truncated (the caller's buffer bound is not this memory's size). |
| Low | include/ExVectrData/memory_internal.hpp:35 | `getMemoryPointer<T>` returns `internalMem_ + index` (a `uint8_t*`) as `T*` without a cast. | Doesn't compile for any `T` other than `uint8_t`. It is a template, so the error only appears when instantiated. |
| Low | include/ExVectrData/memory_internal.hpp:48 | `memcpy` is used without `<string.h>`/`<cstring>`. | Breaks non-Arduino (CMake/host) builds. |
| High (if used) | src/at24cx.cpp:51, 65 | `readBus`/`writeBus` return true on success, but `readMem`/`writeMem` treat true as failure. | Every successful access returns 0 and logs an error. The driver is unusable. |
| Med (if used) | src/at24cx.cpp:51, 65 | `bufferIndex` is ignored: `bufferPtr` is passed to the bus without the offset. | Data lands at the wrong buffer offset. |
| Med (if used) | src/at24cx.cpp:85 | `bytesWritten` is initialised to `numBytes`, so the page-write loop never runs, yet the function returns `numBytes`. | `transferFrom` reports success without writing anything. |
| Med (if used) | src/at24cx.cpp:89-90 | Chunks are `pageSize_` long but not aligned to page boundaries, and there is no write-cycle (~5 ms) wait. | Writes wrap inside a page and corrupt data; back-to-back writes get NACKed. |
| Med (if used) | src/at24cx.cpp:111, 130 | The address is sent as the first 2 bytes of a little-endian `size_t` (LSB first); AT24Cxx expects MSB first. | Wrong addresses are accessed. |
| Low | src/at24cx.cpp:30, 36 | Error messages say "QMC5883" (copy-paste). | Misleading logs. |
| Low | include/ExVectrData/external_chips/at24cx.hpp:15 | `class HAL::DigitalIO;` is not a valid forward declaration (a qualified name inside `VCTR::Data`). GCC accepts it with a warning (the object file builds), and it only works because at24cx.cpp includes the HAL header first. | Breaks when the header is included standalone or with other compilers. |
| Low | src/memory_buffer.cpp:23, 57 | `endIndex > size()` should be `>=`; `endIndex == size()` is not clamped. | Off-by-one read/write one byte past the end. The class is marked "DO NOT USE". |

## Extending
- **New memory backend** (flash, FRAM, SD):
  - Subclass `Memory_Interface` and implement `readMem`, `writeMem`, `transferFrom` and `size`.
  - Honour `bufferIndex`, clip to bounds without underflow, and return the actual byte count (0 on failure).
  - In `transferFrom`, read the source in chunks (avoid large VLAs on the stack) and, for wear-limited media, write only changed bytes (see `Memory_ArduinoEEPROM::transferFrom`).
  - Add `using Memory_Interface::readMem; using Memory_Interface::writeMem;` so the typed helpers are not hidden by your overrides (`Memory_Internal` does this).
- **New persistent item**: add a key to `src/memory_keys.hpp`. In `initialiseMemory()`, follow the pattern `readItem` → on failure `allocateItem(default, KEY)` + `writeItem`. Persist with `eeprom.transferFrom(internalMemory)`.
- **Changing the memory layout or item format**: bump `Memory_Manager::VERSION_`. Every device then clears its memory (losing calibrations) on the next boot.
