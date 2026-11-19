# ExVectrActuator
> Actuator output layer: servo/ESC pulse-width PWM protocols (`PWM_Output`), a rate-limited servo task (`Servo_Control`), H-bridge DC-motor outputs and a servo-motor interface. Repo: https://github.com/ExVeCtr/ExVectrActuator.git `Last verified against commit: db98892 (pinned now: 4d3c87a = db98892 plus a mechanical NOW() -> NowNs() port for ExVectrCore 18bbb09; content otherwise as verified)`

## Purpose & scope
Maps normalised commands onto `HAL::PinPWM`/`HAL::PinGPIO` hardware:
- `PWM_Output`: value 0..1 → pulse width for the STANDARD / ONESHOT125 / ONESHOT42 / MULTISHOT protocols. Digital protocols (DShot) are explicitly unsupported.
- `Servo_Control`: a periodic task meant to limit servo velocity and acceleration and estimate the real servo position. It does not work; see Known issues.
- `HBridge2Wire`, `HBridge3Wire`: `HAL::AnalogOutput` for bidirectional DC motors, command -1..1.
- `ServoMotor`: abstract `getServoState()/setServoState()` ([angle, speed]). No implementation exists.

README ToDo: "Create pwm output. Requires testing" and "Servo control … Requires testing" (both unchecked). The last commit message mentions "Tested and fixed Servo PWM protocol" (c56ad14), which refers to the STANDARD protocol.

## Dependencies (ExVectr libs + external)
- ExVectrHAL: `PinPWM` (`init`, `setPinValue(0..1 duty)`, `setPinFrequency(Hz)`), `PinGPIO`, `AnalogOutput`, `IO_TYPE_t::ANALOG_HBRIDGE`, `IO_PARAM_t::SPEED`.
- ExVectrCore: `Task_Periodic`, `Scheduler`, `NowNs()`, `printW`.
- ExVectrMath and ExVectrSensor: only through `servo_motor.hpp`, which includes `ExVectrMath.hpp` and `ExVectrSensor/encoder.hpp`. README lists only Core and HAL.
- External: none.
- Build: CMakeLists.txt references `ExVectrHal` (wrong case) and lacks Math/Sensor (host builds only).

## Key concepts & API

### PWM_Output (include/ExVectrActuator/pwm_output.hpp, src/pwm_output.cpp)
```cpp
PWM_Output(HAL::PinPWM& pin, PWM_Output_Protocol p = PWM_Output_Protocol::STANDARD);
void  init();                 // MUST be called: pin.init(), set frequency, set mapping constants
void  setValue(float v);      // clamped to 0..1; duty = (v*factor_ + offset_) / interval_
float getValue();             // last commanded value (not the physical position)
void  enableOutput(bool en);  // false → duty 0 (no pulses at all, not "minimum pulse")
```
- Before `init()`: `factor_ = offset_ = 0` and `interval_ = 0.5`, so any `setValue` outputs duty 0 (safe).
- Disabling outputs duty 0, i.e. no pulse train. ESCs treat this as signal loss (disarm or failsafe); servos go limp.

Protocol timing (pulse = duty × period; the platform `PinPWM` clamps duty to 0..1):

| Protocol | Pin freq set | Period | Code mapping (offset, factor, interval) | Actual pulse for v = 0 → 1 | Intended pulse |
|---|---|---|---|---|---|
| STANDARD | 400 Hz | 2500 µs | 1, 1, 2.5 (ms units) | 1000 → 2000 µs ✔ | 1000–2000 µs |
| ONESHOT125 | 2000 Hz | 500 µs | 125, 125, 500 | 125 → 250 µs ✔ | 125–250 µs |
| ONESHOT42 | 11904 Hz | 84 µs | 42, 84, 100 | 35.3 → 84 µs (saturates at v ≥ 0.69) ✘ | 42–84 µs (needs 42, 42, 84) |
| MULTISHOT | 25000 Hz | 40 µs | 5, 25, 100 | 2 → 12 µs ✘ | 5–25 µs (needs 5, 20, 40) |

- Resolution: the Arduino platform `PinPWM` defaults to 8 bits (`res_ = 8`), and nothing in this project changes it.
  - STANDARD at 400 Hz: 2500 µs / 256 = 9.8 µs per step, about 1 % of the 1000 µs range (about 100 steps end to end).
  - Call `PinPWM::setPinResolution(12..15)` for finer servo control. This is global (`analogWriteResolution`).
- Teensy caveat: `analogWriteFrequency` sets the frequency of the pin's hardware timer, which other pins on that timer share. Mixing protocols on pins of the same timer changes the frequency of all of them.

### Servo_Control (include/ExVectrActuator/servo_control.hpp, src/servo_control.cpp)
- `Servo_Control(maxPos, minPos, maxVel, maxAccel, PinPWM&, protocol)` owns a `PWM_Output` and registers itself (20 ms task) with the system scheduler.
- `setPosition(v)` clamps to [min, max]. `getAngleSet()` returns the setpoint; `getAngleReal()` returns the simulated position.
- Output mapping: `(pos - min)/(max - min)` → `PWM_Output::setValue`.
- `Task_Periodic` is inherited **privately** (`class Servo_Control : Core::Task_Periodic`), so callers can't change its interval or priority.
- Not usable as is (see Known issues).

### HBridge2Wire / HBridge3Wire (include/ExVectrActuator/hbridge.hpp, src/hbridge.cpp)
- Both are `HAL::AnalogOutput`: `writeValue(float)`, `writeValue(const float*, n)` (uses element 0), `writable() = 1`, `getOutputType() = ANALOG_HBRIDGE`, `setOutputParam(SPEED, Hz)` sets the PWM frequency.
- 2-wire (two PWM pins):
  - v > 0: A = v, B = 0
  - otherwise: A = 0, B = -v
  - No clamping (the platform clamps duty to ≤ 1).
- 3-wire (A/B direction GPIOs + one PWM pin, `limit` default 1):
  - v is clamped to ±limit
  - v > 0: A = 1, B = 0, PWM = v
  - otherwise: A = 0, B = 1, PWM = -v
  - v = 0 therefore drives B high with 0 duty. What that does (coast or brake) depends on the bridge.

### ServoMotor (include/ExVectrActuator/servo_motor.hpp)
- Pure interface: `const Math::Vector<float,2>& getServoState() const`, `void setServoState(const Math::Vector<float,2>&)`.
- No virtual destructor, no implementers.

## Usage
As in `src/starship_tvc.cpp:20-44` / `src/starship_flaps.cpp:42-58, 140-165`:
```cpp
Platform::PinPWM servoPin(TVC_SERVO_PIN_1);                       // ExVectrArduinoPlatform
ACTR::PWM_Output servo(servoPin, ACTR::PWM_Output_Protocol::STANDARD);

servo.init();                    // sets 400 Hz and the mapping; required
servo.enableOutput(false);       // start with no pulses
...
servo.enableOutput(armed);       // call every update (see Known issues)
if (armed) servo.setValue(cmd * 0.5f + 0.5f);   // map -1..1 → 0..1 (1000..2000 µs)
```

## Conventions
- Always call `init()` after the HAL pin exists (e.g. in a subsystem `init`/`taskInit`), never rely on the defaults.
- Commands are normalised: `PWM_Output` 0..1, H-bridges -1..1. Angle-to-value mapping, trims and direction reversal belong to the caller (see `StarshipFlaps`: `1 - angle/π*2` for mirrored servos).
- Gate `setValue` with your own enable or arm flag, and call `enableOutput(flag)` every cycle. Because of the `outputEnabled_` bug, `enableOutput(false)` is not latched.
- Namespace `VCTR::ACTR`. Include `ExVectrActuator/<file>.hpp`. The umbrella `ExVectrActuator.hpp` is empty.

## How this project uses it
- Only `PWM_Output`, all with the STANDARD protocol (400 Hz, 1000-2000 µs, 8-bit duty, about 9.8 µs steps):
  - `CTRL::StarshipTVC` (`src/starship_tvc.hpp:37-43`, `.cpp:20-44, 242-277`): 4 TVC servos (`servoXP_/XN_/YP_/YN_`) with `setValue(out*0.5+0.5)`, and 2 motor ESCs (`motorCW_/CCW_`). The ESCs get 0.04 idle added after a 2 s arming delay. Motors are enabled only when `enableMotors_ && enableActuators_ && actuatorTestState_ == Idle`.
  - `CTRL::StarshipFlaps` (`src/starship_flaps.hpp:42-45`, `.cpp:42-58, 140-165`): 4 flap servos, angle in rad → `angle/π*2` (TL/BR mirrored as `1 - …`).
  - Pins are declared in `src/main.cpp:92-102` (`TVC_SERVO_PIN_1..4`, `MOTOR_PIN_CW/CCW`, `FLAP_SERVO_PIN_*`).
- Both subsystems call `enableOutput(x)` every update and gate `setValue` on the same flag, so the non-latching `enableOutput` bug currently has no effect on behaviour.
- `servo_control.hpp` is included (`src/main.cpp:4`, `src/starship_flaps.hpp:14`), but no `Servo_Control` object is created.
- `HBridge*` and `ServoMotor` are not used.

## Known issues & limitations
Paths are relative to `lib/ExVectrActuator/`.

| Severity | Location | Issue | Impact |
|---|---|---|---|
| High (if used) | src/servo_control.cpp:54 | `(Core::NowNs() - lastUpdate_) / Core::SECONDS` is int64 division, so `dTime` is 0 for any period < 1 s. | Every 20 ms update divides by zero (:64). `deltaPos = 0` gives NaN, which then propagates permanently. Unusable. |
| High (if used) | src/servo_control.cpp:65-67 | `accel_` (the configured maximum) is overwritten with the clamped required acceleration. | The limit shrinks or flips sign on every update, so the motion model diverges or stalls. |
| Med (if used) | src/servo_control.cpp:23 | `maxVel_` is stored but never used. | No velocity limiting despite the API. |
| Med (if used) | include/ExVectrActuator/servo_control.hpp:30, 33 | `currentVelocity_` is uninitialised. `lastUpdate_ = 0`, so the first `dTime` is the time since boot. | Garbage initial state and a huge first step. |
| Med (if used) | src/servo_control.cpp:17-31 | The constructor never calls `pwm_.init()` (and `enableOutput(true)` runs before init). | Output stays at duty 0, so the servo never moves. |
| Low | include/ExVectrActuator/servo_control.hpp:23 | `Task_Periodic` is inherited privately. | Callers can't adjust interval or priority, or pause it. |
| Med | src/pwm_output.cpp:102-114 | `enableOutput` never assigns `outputEnabled_`, so it stays true. `enableOutput(false)` writes duty 0 once, and the next `setValue` re-enables pulses. `enableOutput(true)` never re-applies the value. | "Disable" is not latched: any `setValue` while "disabled" drives the actuator (or motor). Safe only with caller-side gating (as done in StarshipTVC/Flaps). |
| Med | src/pwm_output.cpp:58-60 | ONESHOT42 mapping uses factor 84 and interval 100 (the period is 84 µs). | Pulses are 35-84 µs and saturate at v ≥ 0.69. The ESC sees a wrong throttle curve. |
| Med | src/pwm_output.cpp:66, 71-73 | MULTISHOT runs at 25 kHz (the comment says 40 kHz) with factor 25 and interval 100 (the period is 40 µs). | Pulses are 2-12 µs instead of 5-25 µs. ESCs will not arm or will misread the throttle. |
| Low | src/pwm_output.cpp:27-30, include/ExVectrActuator/pwm_output.hpp:20 | STANDARD sets 400 Hz, but the warning text and docs say 500 Hz. The frequency check compares floats with `!=`. | Misleading logs; a spurious warning if the platform rounds the frequency. |
| Low | (platform) lib/ExVectrArduinoPlatform/src/pin_pwm.cpp:9 | 8-bit default PWM resolution; this project never calls `setPinResolution`. | About 1 % (about 0.9° on a 90° servo) command quantisation on TVC and flap servos. |
| Low | src/hbridge.cpp:43-78 | `HBridge2Wire` doesn't clamp its input, and `HBridge3Wire` with v = 0 drives B high (brake or coast depends on the driver). There's no dead-band. | Depends on the platform clamp; direction pins toggle around 0. |
| Low | include/ExVectrActuator/servo_motor.hpp | No virtual destructor. It includes Sensor and Math headers that aren't declared dependencies. | Coupling and a UB risk on polymorphic delete. |
| Low | CMakeLists.txt:13 | `ExVectrHal` has the wrong case; Math and Sensor are missing. | Host CMake build fails. |

## Extending
- **New pulse protocol**: add an enum value to `PWM_Output_Protocol` and a `case` in `PWM_Output::init()`. Set `setPinFrequency(f)` with `1/f` ≥ the maximum pulse, then `offset_ = minPulse`, `factor_ = maxPulse - minPulse`, `interval_ = 1/f`, all in the same time unit. Check that duty stays ≤ 1 at v = 1.
- **New actuator type**: implement `HAL::AnalogOutput` (`getOutputType`, `setOutputParam`, `writable`, `writeValue`) as the H-bridges do, so control code stays hardware-agnostic. Clamp inputs inside the class.
- **Rate-limited servo**: fix `Servo_Control` first. Use `float dt = (NowNs()-last) * 1e-9f`, initialise `lastUpdate_` in `taskInit`, keep `maxAccel` and `maxVel` constant and clamp velocity, then call `pwm_.init()`. Alternatively implement `ServoMotor`.
