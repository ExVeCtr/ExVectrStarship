# ExVectrHAL
> Platform-neutral hardware abstraction interfaces (buses, pins, interrupts) for ExVeCtr projects. Repo: https://github.com/ExVeCtr/ExVectrHAL.git `Last verified against commit: bb9ea02`

## Purpose & scope
Defines abstract interfaces that drivers program against and that platform libraries (e.g. ExVectrArduinoPlatform) implement:
- Byte-stream I/O: `DigitalInput`, `DigitalOutput`, `DigitalIO`, used for I2C, SPI, UART, CAN and topic-backed links.
- Value-stream I/O: `AnalogInput`, `AnalogOutput`.
- Pins: `PinGPIO`, `PinPWM`, `PinADC`, `PinDAC`.
- Interrupts: `InterruptEvent`.
- `BusDevice`, a register read/write helper on top of `DigitalIO`.

Almost no logic lives here: only the single-byte/single-value convenience defaults, the `getPin()` accessors and `BusDevice`. The README to-do lists "Basic HAL interfaces", "interrupt system" and documentation as unfinished.

## Dependencies (ExVectr libs + external)
- ExVectrCore: only `ExVectrCore/print.hpp` (for `LOG_MSG`) in src/bus_device.cpp. CMakeLists.txt:12 declares `addExVectrDependency(ExVectrCore)`.
- External: `stdint.h` and `stddef.h` only.

## Key concepts & API
All types live in `namespace VCTR::HAL`.

### Enums (io_types.hpp, io_params.hpp)
- `enum class IO_TYPE_t { BUS_I2C=0, BUS_SPI, BUS_UART, BUS_CAN, ANALOG_PWM_O, ANALOG_PWM_I, ANALOG_DAC, ANALOG_ADC, ANALOG_HBRIDGE, ANALOG_TOUCH, DIGITAL_I, DIGITAL_O, TOPIC, OTHER }`.
  - `OTHER` must stay last; values ≥ `OTHER` are custom types.
  - `TOPIC` is used by ExVectrNetwork's topic-backed IO.
- `enum class IO_PARAM_t { SPEED=0, SPI_MODE, MSB_FIRST, VOLTAGE_MAX, VOLTAGE_MIN }`.
  - `SPEED` is a bus clock in Hz or a baud rate.
  - `SPI_MODE` takes values 0-3.
  - `MSB_FIRST` takes a boolean (nonzero means MSB first).

### Byte-stream I/O (digital_input.hpp, digital_output.hpp, digital_io.hpp)
`DigitalInput` declares:
```cpp
virtual IO_TYPE_t getInputType() const = 0;
virtual bool   setInputParam(IO_PARAM_t, int32_t) = 0;   // true if supported
virtual size_t readable() = 0;                            // bytes readable now
virtual size_t readData(void* data, size_t size, bool endTransfer = true) = 0; // returns bytes read
virtual bool   readByte(uint8_t& b, bool endTransfer = true);  // default: readData(&b,1)==1
```

`DigitalOutput` is the mirror: `getOutputType`, `setOutputParam`, `writable()`, `writeData(const void*, size_t, bool endTransfer=true)` (returns bytes written), and `writeByte(uint8_t, bool endTransfer=true)`.

`class DigitalIO : public DigitalInput, public DigitalOutput {}` is the interface every bus driver takes (`HAL::DigitalIO&`).
- Input and output params and types are separate virtuals, so implementations normally return the same value for both and callers set both.
- There is no virtual inheritance; `DigitalIO` holds two vtable subobjects.

`endTransfer` semantics: `false` keeps the transaction open (SPI: chip select stays asserted; I2C: no STOP after a write). The last call of a sequence must pass `true`. The typical register read is:
```cpp
bus.writeByte(reg | READ_FLAG, false);   // keep CS / transaction
bus.readData(buf, n, true);              // end transfer
```
Semantics are only as good as the platform implementation; see the ExVectrArduinoPlatform doc for I2C repeated-start behaviour.

Return values: the `bool` helpers return true on success, and the `size_t` functions return a count (0 means failure).

Threading: none implied. Bus objects are not locked; calls from an ISR and a task on the same bus will collide.

### Value-stream I/O (analog_input.hpp, analog_output.hpp)
Same pattern with `float` values:
- `AnalogInput`: `getInputType`, `setInputParam`, `readable()`, `readValue(float*, size_t)`, and `readValue(float&)` (defaults to reading one value).
- `AnalogOutput`: `getOutputType`, `setOutputParam`, `writable()`, `writeValue(const float*, size_t)`, and `writeValue(float)`.

### Pins (pin_gpio.hpp, pin_pwm.hpp, pin_adc.hpp, pin_dac.hpp)
Each pin class stores `int32_t pin_` (set in the constructor) and has a virtual `getPin()`.
- `PinGPIO`:
  - `init(GPIO_IOMODE_t)`, `setPinValue(bool)`, `getPinValue()`, `setPinMode(GPIO_IOMODE_t)`, `getPinMode()`, `setPinPull(GPIO_PULL_t)`, `getPinPull()`.
  - `GPIO_IOMODE_t { IOMODE_NONE=0, IOMODE_INPUT=1, IOMODE_OUTPUT=2 }`.
  - `GPIO_PULL_t { PULL_NONE=0, PULL_UP=1, PULL_DOWN=2 }`.
- `PinPWM`: `init()`, `setPinValue(float duty 0..1)`, `getPinValue()`, `setPinFrequency(float Hz)`, `getPinFrequency()`, `setPinResolution(uint32_t bits)`, `getPinResolution()`.
- `PinADC`: `init()`, `int32_t getPinValue()` (raw counts), `setPinResolution(uint32_t)`, `getPinResolution()`.
- `PinDAC`: `init()`, `setPinValue(float 0..1)`, `setPinResolution(uint32_t)`, and a second, mis-named pure virtual `uint32_t setPinResolution()` that was meant to be the getter.

### InterruptEvent (interrupt_event.hpp)
`class InterruptEvent { public: virtual ~InterruptEvent(); virtual void trigger() = 0; };`
- A platform implementation calls `trigger()` from an ISR, so whatever `trigger()` does runs in ISR context.
- Arduino implementation: `Platform::Interrupt_GPIO<T>`.

### BusDevice (bus_device.hpp, bus_device.cpp)
```cpp
BusDevice(DigitalIO& ioBus, int spiRWBit = -1, bool invertRWBit = false, bool verifyWrite = false);
bool writeReg(uint8_t reg, uint8_t byte, bool verifyWrite = false);
bool writeReg(uint8_t reg, const uint8_t* bytes, size_t length, bool verifyWrite = false);
bool readReg(uint8_t reg, uint8_t* bytes, size_t length = 1);
```
- Intended behaviour: a write sends the register byte (`endTransfer=false`) followed by the data. A read sends the register byte (`false`) and then reads `length` bytes (`true`).
- For SPI, bit `spiRWBit` of the register byte marks the direction: set means read, unless `invertRWBit`.
- Optional verification reads the first byte back.
- The implementation deviates from this; see Known issues.
- Holds a non-owning `DigitalIO*`.

## Usage
Driver written against the HAL (pattern used by the ExVectrSensor drivers and `src/vl53l0x/vl53l0x_driver.cpp`):
```cpp
#include "ExVectrHAL/digital_io.hpp"
using namespace VCTR;

class MySensor {
  HAL::DigitalIO* bus_;
public:
  explicit MySensor(HAL::DigitalIO& bus) : bus_(&bus) {}
  bool init() {
    bus_->setInputParam(HAL::IO_PARAM_t::SPEED, 400000);   // set both directions
    bus_->setOutputParam(HAL::IO_PARAM_t::SPEED, 400000);
    uint8_t id;
    return readRegs(0x0F, &id, 1) && id == 0x42;
  }
  bool readRegs(uint8_t reg, uint8_t* buf, size_t n) {
    uint8_t cmd = reg;
    if (bus_->getOutputType() == HAL::IO_TYPE_t::BUS_SPI) cmd |= 0x80; // SPI read flag
    if (!bus_->writeByte(cmd, false)) return false;          // writeByte returns true on success
    return bus_->readData(buf, n, true) == n;
  }
};
```
Actuator code takes `HAL::PinPWM&` so that it does not depend on the platform:
```cpp
StarshipFlaps(HAL::PinPWM& ul, HAL::PinPWM& ur, HAL::PinPWM& ll, HAL::PinPWM& lr); // src/starship_flaps.hpp:66
// in taskInit(): pin.init(); pin.setPinFrequency(...); pin.setPinValue(0.5f);
```

## Conventions
- Namespace `VCTR::HAL`. Headers live in `include/ExVectrHAL/`. The umbrella header `include/ExVectrHAL.hpp` is empty; include individual headers.
- Interfaces are pure-virtual classes with virtual destructors. Enums are `enum class` with a `_t` suffix. Members have a trailing underscore.
- Drivers take HAL interfaces by reference and keep raw pointers. The application owns concrete platform objects, usually as globals in main.cpp.
- No allocation anywhere in the library.

## How this project uses it
- `HAL::DigitalIO&` is the bus type for:
  - `src/vl53l0x/vl53l0x_driver.hpp:114,167,220,227`. Speed is set via both param calls at `vl53l0x_driver.cpp:72-73`.
  - `src/ublox_gnss.hpp:42` / `ublox_gnss.cpp:51`.
  - ExVectrSensor's MPU9250, BME280 and QMC5883 drivers.
- `HAL::PinPWM&` is used for the TVC servos and motors (`src/starship_tvc.hpp:89-91`, `starship_tvc.cpp:14-16`, `init()` at `:41-47`) and for the flap servos (`src/starship_flaps.hpp:66-67`, `starship_flaps.cpp:37-53`). The concrete pins are `Platform::PinPWM` globals at `main.cpp:92-102`.
- `HAL::GPIO_IOMODE_t` is used in `initialiseHardware()` (`main.cpp:1734-1742`).
- `HAL::InterruptEvent` is included by ExVectrSensor's `mpu9250.hpp:24`. It is implemented by `Platform::Interrupt_GPIO` (`main.cpp:72`, wired at `:1756`).
- `HAL::BusDevice` is not used by `src/` or the sensor libraries.
- Cross-lib note: `ExVectrSensor/src/mpu9250.cpp:910-911` ORs `writeByte()` results into a `failure` flag. `writeByte` returns true on success, so that logic is inverted.

## Known issues & limitations
| Severity | Location | Issue | Impact |
|---|---|---|---|
| Med | src/bus_device.cpp:40-55 | `writeReg` computes `command` (with the R/W bit applied) but sends the raw `reg`. | The R/W bit is never applied on writes. This breaks devices that need the write bit set (`invertRWBit=true`). |
| Med | src/bus_device.cpp:95-104 | `readReg` applies the R/W bit whenever the bus is `BUS_SPI`, without checking `spiRWBit_ >= 0`. | With the default `spiRWBit=-1`, `1 << -1` is undefined behaviour. With the default constructor (`spiRWBit_=0`, bus_device.hpp:29) it flips bit 0 of the register address. |
| Med | src/bus_device.cpp:42-53 | `writeReg` applies the R/W bit regardless of bus type (`readReg` checks for SPI). | On I2C with `spiRWBit` set, the register address would be altered on write (currently masked by the bug above). |
| Low | src/bus_device.cpp:67-79 | Write verification reads back and compares only `bytes[0]`. | Multi-byte writes are not verified beyond the first byte. |
| Low | include/ExVectrHAL/bus_device.hpp:29 vs :43 | The member default `spiRWBit_ = 0` differs from the constructor default `-1`. | A default-constructed `BusDevice` behaves differently from `BusDevice(bus)`. |
| Low | include/ExVectrHAL/pin_dac.hpp:45 | The getter is declared as `virtual uint32_t setPinResolution() = 0;`. | Implementers must override a mis-named function; there is no `getPinResolution`. |
| Low | include/ExVectrHAL/pin_gpio.hpp:74,80 | The comment says "1 pullup, 0 none, -1 pulldown"; the enum is `PULL_UP=1, PULL_DOWN=2`. | Misleading documentation. |
| Low | include/ExVectrHAL.hpp | The umbrella header includes nothing. | `#include "ExVectrHAL.hpp"` provides no declarations. |
| Low | include/ExVectrHAL/digital_io.hpp:23 | Separate input and output `getType`/`setParam` virtuals; no locking or ownership model for shared buses. | Callers must set params twice. Concurrent use (ISR plus task) of one bus is unsafe. |
| Low | include/ExVectrHAL/interrupt_event.hpp | A bare `trigger()` with no enable/disable or edge configuration. README: "Add interrupt system" is still a to-do. | Platform-specific behaviour (edge, pull) is hard-coded in implementations. |

## Extending
- **New bus implementation** (in a platform library): derive `final` from `HAL::DigitalIO` and implement all eight pure virtuals.
  - Honour `endTransfer` (keep CS or the transaction open while it is false).
  - Return byte counts (0 on failure), and return `false` from `set*Param` for unsupported params.
  - Return the same `IO_TYPE_t` from both `getInputType` and `getOutputType`.
- **New pin type:** derive from `HAL::PinGPIO`, `PinPWM`, `PinADC` or `PinDAC`; pass the pin to the base constructor; implement the pure virtuals.
- **New interface:** add a header under `include/ExVectrHAL/` in `namespace VCTR::HAL` with pure virtuals and a virtual destructor. If it is a new bus kind, add an `IO_TYPE_t` entry before `OTHER`; if it needs a new param, add an `IO_PARAM_t` entry.
- **New interrupt source:** derive from `HAL::InterruptEvent`, call `trigger()` from the ISR, and keep the callback minimal (ISR context).
