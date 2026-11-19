# ExVectrSensor
> Abstract sensor interfaces that publish timestamped value+covariance samples on topics, plus drivers for MPU9250 (IMU), BME280/BMP280 (baro) and QMC5883L (mag). Repo: https://github.com/ExVeCtr/ExVectrSensor.git `Last verified against commit: 6bd4b01 (pinned now: bfe9b4e = 6bd4b01 plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09; content otherwise as verified)`

## Purpose & scope
- `include/ExVectrSensor/*.hpp`: one interface per sensor kind. Each owns a `Core::Topic` and a pure virtual `readX()`. Fusion code (ExVectrDSP) depends only on these interfaces.
- `include/ExVectrSensor/Sensors/`: concrete drivers. Each has a passive class (`initSensor(bus)` + `readX()`) and a `...Driver` subclass that is a `Core::Task_Periodic` and self-registers with a scheduler.

README roadmap: MPU9250, BME280 and QMC5883 are done. Still open: u-blox GPS (implemented in the app instead, `src/ublox_gnss.*`), "reduce MPU9250 read overhead", docs. The rangefinder (VL53L0X, `src/vl53l0x/`) and GNSS drivers live in the application, not in this library.

## Dependencies (ExVectr libs + external)
- ExVectrCore: `Topic`, `Timestamped<T>` (`data` + `int64_t timestamp` in ns), `Task_Periodic`, `Scheduler`, `NowNs()`, `delay()`, `print`.
- ExVectrHAL: `DigitalIO` (I2C/SPI bus device), `IO_TYPE_t`, `IO_PARAM_t`, `PinGPIO`.
- ExVectrDSP: `DSP::ValueCov<T,N>` (`Math::Vector<T,N> val`, `Math::Matrix<T,N,N> cov`). README omits this dependency. Assigning a scalar to `cov` sets the diagonal.
- ExVectrMath: vectors and matrices.
- External: none. MPU9250 code is derived from Bolder Flight `invensense-imu` (MIT). BME280 code is derived from the SparkFun BME280 library.
- Build: CMakeLists.txt references `ExVectrHal` with a lowercase 'al', which breaks on case-sensitive filesystems (host builds only).

## Key concepts & API

### Interfaces (include/ExVectrSensor/)
All are in namespace `VCTR::SNSR`. Topic payload is `Core::Timestamped<DSP::ValueCov<float,N>>` unless noted.

| Class | Topic getter | N | `read` method | Documented unit / frame |
|---|---|---|---|---|
| `Accelerometer` | `getAccelTopic()` | 3 | `readAccel()` | m/s², sensor frame |
| `Gyroscope` | `getGyroTopic()` | 3 | `readGyro()` | rad/s, sensor frame |
| `Magnetometer` | `getMagTopic()` | 3 | `readMag()` | tesla, sensor frame (**drivers don't comply**, see below) |
| `Barometer` | `getBaroTopic()` | 1 | `readBaro()` | Pa |
| `Thermometer` | `getThermoTopic()` | 1 | `readTherm()` | °C |
| `Hygrometer` | `getHygroTopic()` | 1 | `readHygro()` | doc says °C (copy-paste); BME280 publishes relative humidity as a 0..1 fraction |
| `RangeFinder` | `getRangeTopic()` | 1 | `readRange()` | m, sensor frame |
| `Encoder` | `getEncoderTopic()` | `ValueCov<int32_t,1>` | `readEncoder()` | counts |
| `GNSS` | `getGNSSTopic()` | `Timestamped<GNSSData>` | `readGNSS()` | see below |

- `GNSSData` (gnss.hpp):
  - `Vector_D position` = [lat rad, lon rad, alt m above ellipsoid]
  - `Vector_F velocity` (NED m/s)
  - `Vector_F positionCov`, `velocityCov` (defaults 1)
  - `positionValid`, `velocityValid`, `numSats`
- `readX()` returns false on failure or no new data, and publishes only on success.
- Timestamps are `Core::NowNs()` in ns, taken *before* the bus read (or the MPU9250 interrupt time).

### MPU9250 / MPU9250Driver (Sensors/mpu9250.hpp, src/mpu9250.cpp)
- `MPU9250 : Gyroscope, Accelerometer, Magnetometer, Thermometer`.
  - `initSensor(DigitalIO&)` accepts SPI (mode 3, 1 MHz configuration, 20 MHz burst reads) or I2C (400 kHz).
  - It then sets the defaults: DRDY interrupt on, ±16 g, ±2000 dps, DLPF 184 Hz, SRD 0 (1 kHz).
- Configuration: `ConfigAccelRange`, `ConfigGyroRange`, `ConfigDlpf` (`DLPF_BANDWIDTH_250HZ_4kHz` … `5HZ`, `DISABLE_32kHz`), `ConfigSrd`, `EnableDrdyInt`/`DisableDrdyInt`. Status: `IMUFailed()`, `MagnetometerFailed()`.
- `readAll()`:
  - One 22-byte burst from `INT_STATUS`; returns false if DRDY isn't set.
  - Then publishes gyro, accel and temperature, plus mag unless `disableMag`.
  - Uses the interrupt timestamp if `interruptHandler()` stored one.
  - The individual `readGyro/readAccel/readMag/readTherm` each repeat the full burst.
- Output conversion (`Read()`, mpu9250.cpp:855-903). The axes are remapped to the AK8963 magnetometer frame (Bolder Flight convention), a proper rotation:
  - accel/gyro: `x = raw_y`, `y = raw_x`, `z = -raw_z`. At rest, level, accel z ≈ -9.81 m/s².
  - accel uses `9.80665` m/s² per g; gyro is converted from deg/s to rad/s.
  - mag: AK8963 raw axes × factory ASA scale, in **µT**.
  - temperature: `(counts - 21)/333.87 + 21` °C.
- Covariances (diagonal): gyro 0.002, accel 0.78, mag 1, temperature 0.5 (mpu9250.hpp:50-53).
- `MPU9250Driver(bus, disableMag = false)` or `(bus, scheduler, disableMag)`:
  - Task period 1 ms; `taskInit()` calls `initSensor`; `taskThread()` calls `readAll()`.
  - `interruptHandler()` stores `NowNs()` for precise timestamps.
  - `enable4kHzRate()` sets DLPF 250 Hz and a 250 µs period; `enable32kHzRate()` disables the DLPF and sets a 31.25 µs period.

### BME280 / BME280Driver (Sensors/bme280.hpp, src/bme280.cpp)
- `BME280 : Barometer, Thermometer, Hygrometer`.
  - I2C (400 kHz) or SPI (mode 0).
  - Chip ID `0x60` = BME280; `0x58` = BMP280, which sets `noHumidity_`.
  - Reads the calibration registers, then applies `settings`: normal mode, standby 0.5 ms, `filter = 1`, temperature and pressure ×16 oversampling, humidity oversampling 0 (humidity measurement skipped).
- `readSensorData()` (= `readBaro/readTherm/readHygro`):
  - pressure (Pa, cov `altCov_` = 1)
  - humidity / 100 (0..1 fraction, cov 0.5; not published for BMP280)
  - temperature (°C, cov 0.5)
  - Each value is published only if non-zero. All three share one timestamp.
- Extra API (SparkFun-style): `setMode`, `set*OverSample`, `setStandbyTime`, `setFilter`, `setReferencePressure`, `setPressureOffset`, `setHumidityOffset`, `setTemperatureCorrection`, `isMeasuring`, `reset`, `readAllMeasurements`.
- `BME280Driver(bus[, scheduler])`: 50 ms task. `taskThread()` calls `readBaro()`.

### QMC5883 / QMC5883Driver (Sensors/qmc5883.hpp, src/qmc5883.cpp)
- `QMC5883 : Magnetometer`, I2C only (default address 0x0D).
- Init sequence:
  - Checks that chip-ID register 0x0D reads 0xFF.
  - Soft reset; SET/RESET period register 0x0B = 0x01.
  - CONFIG = `0x1D`: continuous mode, 200 Hz ODR, ±8 G range, OSR 512.
- `readMag()`:
  - If the STATUS DRDY bit is set, reads 6 bytes little-endian.
  - Publishes `raw * 8 / 32767`, i.e. **gauss**, in raw sensor axes (no remap), with cov `eye(0.05)`.
- `QMC5883Driver(bus[, scheduler])`: 20 ms task. If no sample arrives for 500 ms it calls `setInitialised(false)`, so the scheduler re-runs `taskInit()` (re-init).

## Usage
As in `src/main.cpp:71-87, 1750-1757, 1921-1933`:
```cpp
Platform::PinGPIO mpuPin(MPU9250_NCS_PIN);
Platform::BusSPIDevice spiMPU(SPI, mpuPin, true);
SNSR::MPU9250Driver mpuDriver(spiMPU, true);            // internal AK8963 disabled; QMC5883 used for mag
Platform::Interrupt_GPIO<SNSR::MPU9250Driver> mpuIntHandler(MPU9250_INT_PIN);

Platform::BusI2CDevice bmeBus(Wire, 0x76);  SNSR::BME280Driver bme(bmeBus);
Platform::BusI2CDevice qmcBus(Wire, 0x0D);  SNSR::QMC5883Driver qmc(qmcBus);

// setup()
mpuDriver.taskInit();                                   // init now instead of waiting for the scheduler
mpuDriver.ConfigDlpf(SNSR::MPU9250::DlpfBandwidth::DLPF_BANDWIDTH_41HZ);
mpuDriver.setInterval(250 * Core::MICROSECONDS);        // poll at 4 kHz; DRDY gates actual 1 kHz samples
mpuDriver.setInitialised(true);
mpuIntHandler.setCallback(&SNSR::MPU9250Driver::interruptHandler, mpuDriver);

gyroTransformTopic.setInputTopic(mpuDriver.getGyroTopic()); // calibration + sensor→body transform (DSP)
posEstTask.setBaroInput(bme.getBaroTopic());
```
Consuming a topic:
```cpp
Core::Callback_Subscriber<Core::Timestamped<DSP::ValueCov<float,3>>, MyClass> sub;
sub.setCallback(this, &MyClass::onGyro); sub.subscribe(imu.getGyroTopic());
```

## Conventions
- Publish SI units in the sensor frame. Let downstream code (`DSP::TopicCoordTransform` with calibration from ExVectrData) handle bias, scale and sensor-to-body rotation. The existing mag drivers break this rule (µT / gauss). Because the mag calibration scale absorbs the unit, the mag units only have to be consistent per sensor.
- Always fill `cov` (a scalar sets the diagonal) and timestamp with `Core::NowNs()` (ns), as close to the physical sample as possible.
- Drivers are passive classes plus an optional `...Driver : Task_Periodic` that registers with `getSystemScheduler()` by default.
- Bus parameters (speed, SPI mode) are set by the driver in `initSensor()` through `setInput/OutputParam`.
- Namespace `VCTR::SNSR`. Include `ExVectrSensor/Sensors/<drv>.hpp` or `ExVectrSensor/<iface>.hpp`. The umbrella `ExVectrSensor.hpp` is empty.

## How this project uses it
- `src/main.cpp:71-75`: `MPU9250Driver mpuDriver(spiMPU, true)` on SPI, with the AK8963 disabled. In `setup()` (:1750-1757): DLPF 41 Hz, 250 µs poll interval, priority 110, interrupt timestamping through `Interrupt_GPIO`. Its thermometer topic is subscribed at :1647.
- `src/main.cpp:80`: `BME280Driver bme(bmeBus)`, I2C at 0x76. Baro feeds `posEstTask.setBaroInput` (:1931) and other subscribers (:543, :1649).
- `src/main.cpp:87`: `QMC5883Driver qmc(qmcBus)`, I2C at 0x0D. Feeds `magTransformTopic` (:1923) and the mag-calibration subscriber (:380).
- App-side drivers implementing these interfaces:
  - `SNSR::UbloxSerialGNSS gnss(Serial5)` (`src/ublox_gnss.*`, `GNSS`)
  - `SNSR::VL53L0XDriver` (`src/vl53l0x/`, `RangeFinder`, I2C 0x29), fed to `posEstTask.setRangeInput(..., 0.4, 1.5)` (:1932)
- The sensor→body transforms and calibrations come from ExVectrData keys 100-102 / 200-202 (see ExVectrData.md).

## Known issues & limitations
Paths are relative to `lib/ExVectrSensor/`.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| Med | include/ExVectrSensor/magnetometer.hpp:20; src/mpu9250.cpp:900-902; src/qmc5883.cpp:99-101 | The interface says tesla; MPU9250 publishes µT and QMC5883 publishes gauss. | Mags are not interchangeable without recalibration. Any consumer assuming tesla is off by 1e6 or 1e4. Fusion works only because calibration or normalisation absorbs the scale. |
| Med | src/bme280.cpp:196-198 | Pressure is compensated before temperature is read, so `readFloatPressure()` uses the `t_fine` from the previous cycle. `t_fine` (bme280.hpp:146) is uninitialised for the first sample. | First published pressure is wrong (possibly wildly). Later samples use 50 ms-stale temperature compensation (negligible). Reorder to temperature → pressure → humidity. |
| Low | src/bme280.cpp:186-198 | Three separate register reads instead of one burst. | Values can come from different conversions; extra bus time. |
| Low | src/bme280.cpp:82, 197-216 | Default humidity oversampling is 0 (humidity disabled), yet humidity is still computed and published if non-zero. | Bogus humidity may be published on a BME280. |
| Low | include/ExVectrSensor/hygrometer.hpp:20, 25 | Doc says "degrees celsius"; the value is relative humidity (BME280 publishes a 0..1 fraction). | Misleading API docs. |
| Low | include/ExVectrSensor/Sensors/bme280.hpp:156 | `altCov_ = 1` is commented "altitude covariance in meters" but is used as the pressure covariance (Pa²). | Baro covariance is unrealistically small for raw pressure; the consumer must override or ignore it. |
| Med | src/bme280.cpp:56-63 | `taskThread` calls `setPaused(true)` permanently when the sensor isn't initialised (init failed); there is no retry. | A baro that fails at boot (e.g. bus glitch) never recovers. QMC5883 re-inits; BME280 doesn't. |
| Low | src/bme280.cpp:30 | Section comment says "MPU9250Driver Implementation" (copy-paste). | Cosmetic. |
| Low | src/mpu9250.cpp:73-76, 302-306 | `realMeasurentTimestamp_` (int64, not volatile) is written in the ISR and read and cleared in the task without a critical section. | Rare torn 64-bit read gives a timestamp off by about 4.3 s, or a lost interrupt timestamp. |
| Low | include/ExVectrSensor/Sensors/mpu9250.hpp:234; src/mpu9250.cpp:47-55 | `pinInterrupt_` has no setter (main.cpp has `enablePinInterrupt` commented out), so `taskCheck()`'s DRDY-pin path is dead code. | DRDY polling only; with a 250 µs period about 3 of 4 runs find no data (`Read()` returns false). |
| Low | src/mpu9250.cpp:855-863 | `Read()` sets bus `SPEED` to 20 MHz and back to 1 MHz on every read via `setOutputParam`, including when the bus is I2C. | 2 extra HAL calls per sample. On I2C it requests a 20 MHz clock. |
| Low | src/mpu9250.cpp:134-139 | In the SPI branch of the input-type configuration, output params are set instead of input params. | Harmless duplicate today; wrong if input and output buses differ. |
| Low | src/mpu9250.cpp:174-290 | `readGyro/readAccel/readMag/readTherm` each perform a full 22-byte burst. | Calling several of them multiplies bus load (README TODO). Use `readAll()`. |
| Low | src/mpu9250.cpp:79-101 | If `ioBus_` is null, `taskInit` returns without stopping the task, and `taskThread` then prints an error every period (1 ms). | Console flooding. (An init *failure* sets the release time to END_OF_TIME correctly.) |
| Low | include/ExVectrSensor/Sensors/mpu9250.hpp:39 | "Currently the magnetometer is not implemented" is stale; AK8963 reading is implemented. | Misleading docs. |
| Low | src/qmc5883.cpp:87, 121 | `!ioBus_->writeByte(...)` results are discarded (the expression has no effect). | Register-pointer write failures go undetected (the following read usually fails anyway). |
| Low | src/qmc5883.cpp:58 | Log text says "more than 100ms" but the threshold is 500 ms. | Misleading log. |
| Low | src/qmc5883.cpp:63-105 | The STATUS overflow bit (DOR/OVL) is ignored. | Saturated samples (> 8 G) are published as valid. |
| Low | include/ExVectrSensor/gnss.hpp:23, 43 | Uses `Math::Vector_D` without including an ExVectrMath header (it arrives through the DSP include). The topic comment says "accelerometer values … m/s/s" (copy-paste). | Fragile include; misleading docs. |
| Low | include/ExVectrSensor/*.hpp (e.g. barometer.hpp:17) | Interfaces have virtual methods but no virtual destructor. | UB if a driver is deleted through an interface pointer (not done today). |
| Low | CMakeLists.txt:13 | `addExVectrDependency(ExVectrHal)` has the wrong case. | Host CMake build fails on Linux. |

## Extending
New sensor driver (pattern of qmc5883.*):
1. Header `include/ExVectrSensor/Sensors/<chip>.hpp`: `class CHIP : public <Interface>[, ...]` with `HAL::DigitalIO* ioBus_`, `bool initialised_`, `bool initSensor(HAL::DigitalIO&)` and `bool readX() override`.
2. In `initSensor`:
   - check `getInputType()`/`getOutputType()` (`BUS_I2C`/`BUS_SPI`)
   - set the bus `SPEED`/`SPI_MODE`
   - verify the WHOAMI register, configure the chip
   - set `initialised_ = true` last
3. In `readX`:
   - capture `int64_t t = Core::NowNs()` before the transfer
   - read in one burst and convert to SI units in the sensor frame
   - fill `DSP::ValueCov` (val + cov), then `topic_.publish(Core::Timestamped<...>(vc, t))`
   - return false without publishing when there is no new data
4. Optional `CHIPDriver : public CHIP, public Core::Task_Periodic`:
   - constructor `Task_Periodic("CHIP Driver", period)` + `scheduler.addTask(*this)`
   - `taskInit()` → `initSensor` (on failure, retry or `setRelease(Core::END_OF_TIME)` explicitly)
   - `taskThread()` → `readX()`
   - optionally a watchdog that calls `setInitialised(false)` to force re-init
5. New sensor *kind*: add `include/ExVectrSensor/<kind>.hpp` + `src/<kind>.cpp`, following `barometer.hpp`: protected `Core::Topic<Core::Timestamped<DSP::ValueCov<float,N>>> kindTopic_`, getter, `virtual bool readKind() = 0`, and document the unit and frame on the topic. Add a virtual destructor.
