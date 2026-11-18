# ExVectrArduinoPlatform
> Arduino/Teensy implementation of the ExVectrCore platform hooks (clock, console) and the ExVectrHAL interfaces (I2C, SPI, UART, GPIO, PWM, ADC, pin interrupt, EEPROM). Repo: https://github.com/ExVeCtr/ExVectrArduinoPlatform.git `Last verified against commit: 4f31846`

## Purpose & scope
- Provides the two symbols ExVectrCore requires:
  - `VCTR::Core::getPlatformClock()` in src/time_implementation.cpp.
  - `VCTR::Core::initialise()` in src/print_implementation.cpp.
- Provides concrete HAL classes in `VCTR::Platform`: `BusI2CDevice`, `BusSPIDevice`, `BusUART`, `PinGPIO`, `PinPWM`, `PinADC`, `Interrupt_GPIO<T>`.
- Provides `VCTR::Data::Memory_ArduinoEEPROM` (an ExVectrData memory backend).

Branch `Development` targets Teensy 4.0. `PinPWM::setPinFrequency` calls the Teensy-specific `analogWriteFrequency` (commit 4f31846), so this branch no longer builds for cores without that function. An `ESP32-Branch` exists for ESP32. Status per README: there is no DAC class, the Serial bus is still a to-do, and documentation is missing.

## Dependencies (ExVectr libs + external)
- ExVectrCore: clock, print topics, `Task_Periodic`, `StaticCallback_Subscriber`, `printE`.
- ExVectrHAL: all interfaces it implements.
- ExVectrData: `ExVectrData/memory_interface.hpp`, only for memory_eeprom.hpp. Declared in CMakeLists.txt:14; the README lists only Core and HAL.
- External: the Arduino core (`Arduino.h`, `micros`, `pinMode`, `digitalWrite`/`digitalRead`, `analogRead`/`analogWrite`, `analogReadResolution`, `analogWriteResolution`, `attachInterrupt`), Teensy `analogWriteFrequency`, `Wire.h` (`TwoWire`), `SPI.h` (`SPIClass`, `SPISettings`), `HardwareSerial.h`, `EEPROM.h` (`EEPROMClass`). `Serial` is used for console output.
- `ExVectrArduinoPlatform.hpp` includes pin_gpio, pin_adc, pin_pwm, bus_i2c, bus_spi and bus_uart. `interrupt_event.hpp` and `memory_eeprom.hpp` must be included explicitly.

## Key concepts & API

### Platform clock (src/time_implementation.cpp)
- `PlatformClock : Core::Clock_Source`, a function-local static returned by `getPlatformClock()` (:41-47).
- `getCounter()` reads the 32-bit `micros()` and accumulates the signed 32-bit difference into a static `int64_t` ns counter (:18-29). Both `data` and `timestamp` are set to that value.
- Resolution: 1 µs. Wrap-safe as long as it is called at least every ~35 min (2^31 µs). `Core::NOW()` calls it constantly.
- Re-entrancy: a plain `static bool lock` (:15). If the lock is held (an ISR interrupted a task-context call), the call computes a value without updating the state.
- Returns a reference to a shared static `Timestamped`, which an ISR call can overwrite while the caller reads it.

### Console / initialise (src/print_implementation.cpp)
- `Core::initialise()` (:85-97):
  - Prints a start message.
  - Constructs a static `PlatformArduino` task (a `Task_Periodic`, 1 s interval, priority 10, added to the system scheduler).
  - Calls its `taskInit()` directly.
- `taskInit()` (:58-73):
  - Calls `Serial.begin(115200)`.
  - Subscribes four `StaticCallback_Subscriber<const char*>`s to the Core message, debug, warning and error topics. They print to `Serial` with the prefixes `""`, `"Debug: "`, `"!WARNING: "` and `"!!!ERROR: "`.
  - Sets the task's release to `END_OF_TIME`, so the scheduler never runs it.
- Output is synchronous `Serial.print` inside the publisher's context (USB serial on Teensy).

### BusI2CDevice (bus_i2c.hpp / src/bus_i2c.cpp)
`BusI2CDevice(TwoWire& bus, uint32_t address)` is a `final` `HAL::DigitalIO`. Type: `BUS_I2C`.
- `set{Input,Output}Param(SPEED, hz)` calls `bus_.setClock(hz)`. This changes the clock for every device on that `TwoWire`. Other params log an error via `printE` and return false.
- `writeData(data, n, endTransfer)` (:63-84):
  - Calls `beginTransmission` unless a transaction is already open, then `write`.
  - If `endTransfer`, calls `endTransmission()`; on failure it returns 0.
  - Otherwise it leaves the transaction open.
- `readData(data, n, endTransfer)` (:91-105):
  - If a transaction is open, calls `endTransmission()` (sends STOP, result ignored).
  - Then `requestFrom(addr, n, endTransfer)` and copies the bytes.
  - Returns 0 if fewer bytes arrived than requested, or if extra bytes remain.
  - The register-pointer write and the read are therefore separate transactions (STOP, then START); there is no repeated start.
- `writable()` always returns 32; `readable()` always returns 1.
- Blocking (Wire). Not ISR-safe. No bus arbitration between devices beyond Wire itself.

### BusSPIDevice (bus_spi.hpp / src/bus_spi.cpp)
`BusSPIDevice(SPIClass& bus, HAL::PinGPIO& cs, bool invertSelectPin)` is a `final` `HAL::DigitalIO`. Type: `BUS_SPI`.
- Chip select:
  - Select drives CS to `!invertSelectPin`; deselect drives it to `invertSelectPin`. Pass `true` for normal active-low CS.
  - The CS `PinGPIO` must already be `init(IOMODE_OUTPUT)`; `Platform::PinGPIO::setPinValue` ignores calls on non-output pins.
- Params (input and output share the same fields):
  - `SPEED` (Hz), `MSB_FIRST` (bool), `SPI_MODE` (0-3).
  - Defaults are 100 kHz, LSB first, mode 0 (bus_spi.hpp:28-30). Drivers must set `MSB_FIRST`; ExVectrSensor's MPU9250 and BME280 drivers do.
  - `SPISettings` are rebuilt on every transfer.
- `writeData` (:109-138):
  - Copies the data into a stack VLA (because `transfer` needs a non-const buffer).
  - Asserts CS and calls `beginTransaction` unless a transaction is already open, then calls `transfer(buf, n)`.
  - If `endTransfer`, calls `endTransaction()` and releases CS.
  - Always returns `n`.
- `readData` (:145-170): the same sequence, but transfers in place on `data`. Whatever is already in `data` is clocked out on MOSI.
- `writeRead(wbuf, rbuf, wn, rn, end)` (:172-205): a single full-duplex transfer of `max(wn, rn)` bytes. `rbuf` receives the bytes clocked during that same transfer, not bytes read after the write phase.
- `writable()` returns 32; `readable()` returns 1.
- Shared-bus safety relies on `SPI.beginTransaction`. Do not use one SPI bus from an ISR and a task at the same time.

### BusUART (bus_uart.hpp / src/bus_uart.cpp)
`BusUART(HardwareSerial& bus)` is a `final` `HAL::DigitalIO`. Type: `BUS_UART`.
- `SPEED` calls `bus_.begin(baud)`.
- `writable()` returns `availableForWrite()`; `readable()` returns `available()`.
- `writeData` calls `write`. `readData` calls `readBytes`, which blocks up to the Stream timeout (1 s by default).
- `endTransfer` is ignored. `writeRead` does a write followed by `readBytes`.
- Requires a `HardwareSerial` (Serial1..8 on Teensy); USB `Serial` is a different class.

### Pins
- `PinGPIO` (src/pin_gpio.cpp):
  - Caches the mode, the pull and the last written output value (`currentValue_`).
  - `init(mode)` sets `pinMode` and, for outputs, writes LOW.
  - `setPinValue` is a no-op unless the pin is in output mode, and skips the write if the value is unchanged.
  - `getPinValue` returns the cached value for outputs and calls `digitalRead` for inputs.
  - `setPinPull` only works in input mode; it uses `INPUT_PULLUP`/`INPUT_PULLDOWN` if those are defined.
- `PinPWM` (src/pin_pwm.cpp):
  - `init()` calls `pinMode(OUTPUT)` and `analogWrite(0)`.
  - `setPinValue(duty)` clamps to 0..1 and writes `analogWrite(pin, duty * 2^res)`.
  - `setPinFrequency(hz)` stores the value and calls `analogWriteFrequency(pin, hz)` (Teensy: applies to the pin's whole timer group).
  - `setPinResolution(bits)` calls the global `analogWriteResolution`. `res_` is `static` (default 8) and shared by all pins.
  - `freq_` defaults to 200 but is only applied when `setPinFrequency` is called.
- `PinADC` (src/pin_adc.cpp):
  - `init()` calls `pinMode(INPUT)`; `getPinValue()` calls `analogRead`.
  - `setPinResolution` calls the global `analogReadResolution`; `res_` is per-object (default 10).

### Interrupt_GPIO<T> (include/ExVectrArduinoPlatform/interrupt_event.hpp)
- `Interrupt_GPIO<T>(int32_t pin)`.
- `setCallback(void (T::*fn)(), T& obj)`:
  - Stores the callback and points the header-global `interruptEvent` at this object.
  - Sets `pinMode(pin, INPUT_PULLUP)` and calls `attachInterrupt(digitalPinToInterrupt(pin), pinInterruptHandler, RISING)`.
- `trigger()` (from the ISR) calls `(obj.*fn)()` in ISR context.
- Only one instance can be active at a time; the last `setCallback` wins.

### Memory_ArduinoEEPROM (include/ExVectrArduinoPlatform/memory_eeprom.hpp)
- `Memory_ArduinoEEPROM(EEPROMClass& eeprom, size_t size)`, in `namespace VCTR::Data`; implements `Memory_Interface`.
- `readMem` / `writeMem` loop byte by byte (`EEPROM.read`/`write`) with bounds clamping.
- `transferFrom(src, n, to, from)` reads both sides into stack VLAs and writes only the bytes that changed (wear reduction).
- `size()` returns the size passed to the constructor.
- Blocking. Teensy EEPROM writes are slow (flash emulation).

## Usage
Mirrors `src/main.cpp`:
```cpp
#include "ExVectrArduinoPlatform.hpp"
#include "ExVectrArduinoPlatform/interrupt_event.hpp"   // include from ONE .cpp only
#include "ExVectrArduinoPlatform/memory_eeprom.hpp"
#include "ExVectrCore.hpp"
using namespace VCTR;

Platform::PinGPIO mpuCs(MPU9250_NCS_PIN);
Platform::BusSPIDevice spiMPU(SPI, mpuCs, true);           // true => active-low CS
Platform::BusI2CDevice baroBus(Wire, 0x76);
Platform::PinPWM servo(TVC_SERVO_PIN_1);
Platform::Interrupt_GPIO<SNSR::MPU9250Driver> mpuInt(MPU9250_INT_PIN);
Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length());

void setup() {
  Core::initialise();                    // Serial + print topics
  SPI.begin(); Wire.begin(); Wire.setClock(100000);
  mpuCs.init(HAL::GPIO_IOMODE_t::IOMODE_OUTPUT);
  mpuCs.setPinValue(1);                  // deselect
  spiMPU.setOutputParam(HAL::IO_PARAM_t::MSB_FIRST, 1);
  spiMPU.setOutputParam(HAL::IO_PARAM_t::SPI_MODE, 3);
  servo.init(); servo.setPinFrequency(400); servo.setPinValue(0.6f);
  mpuInt.setCallback(&SNSR::MPU9250Driver::interruptHandler, mpuDriver); // runs in ISR
}
void loop() { Core::getSystemScheduler().tick(); }
```

## Conventions
- Concrete classes live in `VCTR::Platform`, are named after the HAL interface they implement (`PinGPIO`, `PinPWM`, …), and are `final`. The exception is `Memory_ArduinoEEPROM`, which is in `VCTR::Data`.
- Constructors take Arduino objects by reference (`TwoWire&`, `SPIClass&`, `HardwareSerial&`, `EEPROMClass&`) and do not touch hardware. Hardware is configured in `init()`, `set*Param()` and `Core::initialise()`. The app must call `SPI.begin()` and `Wire.begin()` itself.
- Instances are created as globals in main.cpp. There is no heap use in this library; transfers use stack VLAs.
- Source files use `using namespace VCTR::Platform;` and define members out of line. Errors are reported via `Core::printE`.

## How this project uses it
- `src/main.cpp:5-9` includes the umbrella header, bus_i2c, bus_spi, interrupt_event and memory_eeprom.
- Hardware globals (`main.cpp:71-105`):
  - `PinGPIO mpuIntPin` :71; `Interrupt_GPIO<SNSR::MPU9250Driver> mpuIntHandler` :72; `PinGPIO mpuPin` :73.
  - `BusSPIDevice spiMPU(SPI, mpuPin, true)` :74.
  - `BusI2CDevice` for the BME280 (0x76) :79, the QMC5883 (0x0D) :86 and the VL53L0X (0x29) :89.
  - Ten `PinPWM`s for the TVC servos, motors and flap servos (:92-102).
  - `Data::Memory_ArduinoEEPROM eeprom(EEPROM, EEPROM.length())` :105.
- `initialiseHardware()` (`main.cpp:1729-1763`):
  - Calls `SPI.begin()`, `Wire.begin()`, `Wire.setClock(100000)`.
  - Sets CS pins to output/high.
  - Configures the MPU at 250 µs.
  - `mpuIntHandler.setCallback(...)` at :1756.
- `Core::initialise()` is called at `main.cpp:1982`.
- PWM frequency is set by ExVectrActuator (`pwm_output.cpp:27,40,53`; `hbridge.cpp:29-30,95`), which relies on commit 4f31846.
- Several drivers call `setParam(SPEED, 400000)` on their `BusI2CDevice` (e.g. `src/vl53l0x/vl53l0x_driver.cpp:72-73`; ExVectrSensor `bme280.cpp:114,123`). Because this calls `Wire.setClock`, the whole `Wire` bus runs at 400 kHz from then on, overriding `Wire.setClock(100000)`.

## Known issues & limitations
| Severity | Location | Issue | Impact |
|---|---|---|---|
| High | include/ExVectrArduinoPlatform/interrupt_event.hpp:17-25 | A non-inline global `interruptEvent` and the function `pinInterruptHandler()` are defined in a header. | A multiple-definition link error as soon as a second .cpp includes it. Today only main.cpp does. |
| Med | include/ExVectrArduinoPlatform/interrupt_event.hpp:17,46-52 | A single global slot (not `volatile`); pull-up and RISING edge are hard-coded; the callback runs in ISR context. | Only one pin interrupt is possible. The callback (MPU driver) must be ISR-safe, but Core topics, print and `NOW()` are not. |
| Med | src/time_implementation.cpp:13-34 | The `lock` flag is a plain `static bool` (not atomic or volatile). The function returns a reference to a shared static that the ISR path also writes (:21). | A 64-bit time value can be torn or overwritten mid-read when `NOW()` is called from both an ISR (e.g. the MPU interrupt) and a task. Time stays monotonic only if no call gap exceeds ~35 min. |
| Med | src/bus_spi.cpp:178-179 | `writeRead` copies `max(wn, rn)` bytes from `writeBuf`. | Reads past the end of `writeBuf` when `readSize > writeSize`. |
| Med | src/bus_spi.cpp:172-205 | `writeRead` is a single full-duplex transfer; `readBuf` receives bytes clocked during the write. | The header doc says "writes then reads". Callers must offset by `writeSize` themselves. |
| Med | include/ExVectrArduinoPlatform/bus_spi.hpp:28-30 | Defaults are LSB first, 100 kHz, mode 0. | A driver that forgets `MSB_FIRST` silently gets bit-reversed data. |
| Med | src/bus_i2c.cpp:94-99 | `readData` closes an open write with `endTransmission()` (STOP, result ignored) before `requestFrom`. | No repeated start, which devices that require repeated-start register reads cannot work with. A NACK on the address/register write goes undetected. |
| Med | src/bus_uart.cpp:74,81 | `readBytes` blocks up to the Stream timeout (1 s by default). | Stalls the cooperative scheduler when called with fewer bytes available than requested. |
| Med | src/bus_i2c.cpp:18-20,40-42 | `SPEED` calls `TwoWire::setClock`, which applies bus-wide. | Per-device speed settings override each other; the last caller wins (see the project section). |
| Low | src/bus_spi.cpp:29-44,73-88 | An invalid `SPI_MODE` value logs an error but still returns true. | Misconfiguration is not reported to the caller. |
| Low | src/bus_spi.cpp:113,178; memory_eeprom.hpp:85,88 | Stack VLAs sized by the caller (and `numBytes` defaults to `SIZE_MAX`, clamped to the EEPROM size). | Large transfers risk stack overflow. VLAs are a GCC extension. |
| Low | src/bus_spi.cpp:137, :169 | `writeData` and `readData` always return `size`. | Transfer failures are never reported. |
| Low | src/bus_i2c.cpp:60,88; src/bus_spi.cpp:104-107,140-143 | `writable()` and `readable()` return the constants 32 and 1. | Not meaningful for flow control. |
| Low | src/bus_i2c.cpp:73-75 | Calls `endTransmission(true)` again after a failed `endTransmission()`. | Pointless retry; produces an extra bus transaction on error. |
| Low | src/bus_uart.cpp:60-75 | `endTransfer` is ignored; `setParam(SPEED)` calls `begin()` again on every call. | Fine for UART, but a speed change re-initialises the port. |
| Low | include/ExVectrArduinoPlatform/bus_uart.hpp:24,27 | Takes `HardwareSerial&`. | Cannot wrap USB `Serial` (`usb_serial_class`) on Teensy. |
| Low | src/pin_pwm.cpp:31 | Writes `duty * (1 << res)`, so duty 1.0 gives 2^res (one past the maximum). | Relies on the core clamping it to 100 %; minor off-by-one in scaling. |
| Low | src/pin_pwm.cpp:43 (commit 4f31846) | `analogWriteFrequency` is Teensy-only and changes the frequency for all pins on the same timer. | Not portable to non-Teensy Arduino cores. Setting one servo's frequency affects its timer siblings. |
| Low | src/pin_pwm.cpp:9,51-55; include/.../pin_pwm.hpp:22-23 | `res_` is static and shared by all pins. `freq_` (200) is not applied in `init()`. | `getPinFrequency()` reports 200 Hz while the hardware runs at the core default until `setPinFrequency` is called. |
| Low | src/pin_adc.cpp:22-26 | `analogReadResolution` is global, but `res_` is stored per pin. | Pins report stale resolutions after another pin changes it. |
| Low | src/pin_gpio.cpp:9-19 | `init()` writes LOW but does not reset the cached `currentValue_`. | After re-init, `setPinValue(true)` may be skipped (the cache still says true). |
| Low | src/pin_gpio.cpp:55-75 | `PULL_NONE` is never applied; changing the mode does not reset the pull state. | An enabled pull cannot be removed via the API. |
| Low | include/.../memory_eeprom.hpp:44-45,63-64,82-83 | `bufferIndex` is checked against the EEPROM size rather than the buffer size. `size_ - index` underflows when `index > size_`. | Out-of-bounds index arguments become huge counts, leading to buffer overruns. |
| Low | include/.../memory_eeprom.hpp:86-98 | `transferFrom` writes `numBytes` bytes even if the source returned fewer (`bytesRead`). | Writes garbage on a short read. |
| Low | src/print_implementation.cpp:87 vs :60-69 | The "Beginning platform initialisation" message is printed before `Serial` is subscribed. | The first message is always lost. |
| Low | README.md, CMakeLists.txt:14 | The README omits the ExVectrData dependency and lists Serial and documentation as to-dos. | Out-of-date documentation. |

## Extending
- **New bus:** add `include/ExVectrArduinoPlatform/bus_x.hpp` and `src/bus_x.cpp` with a `final` class deriving from `HAL::DigitalIO` (see the ExVectrHAL "Extending" section). Take the Arduino peripheral by reference, honour `endTransfer`, return byte counts, and report bad params via `Core::printE` while returning false. Add it to `ExVectrArduinoPlatform.hpp`.
- **New pin type** (e.g. DAC): derive from `HAL::PinDAC` (note its mis-named getter) and implement it in `src/pin_dac.cpp`.
- **Interrupts:** to support more than one pin, replace the global slot with a per-pin table and `inline` or `.cpp`-defined trampolines. Keep `trigger()` callbacks ISR-safe, for example by setting a flag or task release time that a task acts on.
- **New platform** (e.g. a native/simulation build): copy the structure of `src/time_implementation.cpp` (a static `Clock_Source` returned by `getPlatformClock()`, monotonic ns) and `src/print_implementation.cpp` (an `initialise()` that subscribes an output to the four Core print topics), then implement the HAL classes the application needs.
