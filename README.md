# EEEBot — Autonomous Robot (ESP32 / C++)

A year-long autonomous robot build: PID line following, wireless maze navigation
from a custom handheld remote, ultrasonic obstacle detection and colour-triggered
behaviours. Built and programmed from bare PCB to working vehicle as part of a
first-year Electrical & Electronic Engineering construction project at the
University of Nottingham.

---

## Hardware

| Part | Detail |
|---|---|
| Microcontroller | ESP32 (mainboard) + second ESP32 (expansion / remote) |
| Motor driver | L298N with heatsink |
| Drive | 2 DC motors (PWM) + servo steering |
| Encoders | Optical rotary, 24 PPR, half-quadrature → 48 counts/rev |
| Line sensors | 6 optical photodiodes, ~20 mm spacing |
| Distance | HC-SR04 ultrasonic |
| Colour | TCS3200 |
| Remote | 4×3 keypad, 16×2 LCD, status LED |
| Comms | I2C (mainboard ↔ master), ESP-NOW (remote ↔ robot) |

Wheel diameter 6 cm → 18.85 cm circumference → **0.39 cm per encoder count**.

The HC-SR04 echo pin outputs 5 V, which the ESP32's 3.3 V logic cannot take
directly, so a 1.8 kΩ / 3.3 kΩ divider drops it to 3.23 V:

```
Vout = 5 × 3.3k / (3.3k + 1.8k) = 3.23 V
```

---

## Line following (`Line_following_code.ino`)

Six photodiodes are calibrated at startup over black and white surfaces, taking
100 samples each and storing per-sensor min/max. Readings are then mapped to
0–1000 with black as high.

Line position comes from a weighted average across the array, with sensor
positions in mm from centre (−50 to +50):

```
position = Σ(weightᵢ × readingᵢ) / Σ(readingᵢ)
```

Error is the deviation from centre, driving a PID controller:

```
Kp = 1.0    Ki = 0.005    Kd = 0.5
```

The output steers the servo and differentially trims the two motors around a
base speed of 110:

```
steeringAngle = 90 + pidOutput
leftMotor     = 110 + 0.3 × pidOutput
rightMotor    = 110 − 0.3 × pidOutput
```

Loop runs at 50 ms. Motor and steering values are packed into bytes and sent to
the mainboard over I2C at address 0x04.

**Tuning notes.** Ki is deliberately small — the integral term accumulates every
loop with no windup limit, and larger values made the robot weave. Kd = 0.5
damped the overshoot on tight corners.

---

## Wireless maze navigation

**Remote (`Controller_firmware.ino`)** — a handheld unit built around a keypad
and LCD. Movement keys (2/4/6/8) are followed by a quantity, using a two-state
input machine so the display always shows what stage the entry is at. Turn
shortcuts map 7 → 90° and 9 → 180°. `#` clears, `*` transmits. Commands are
stored in a 30-slot array and sent to the robot over ESP-NOW, with delivery
success reported back on the LCD.

**Robot (`Master_Board_Navigation.ino`)** — navigation runs as a state machine
(FORWARD → TURN_LEFT → CHECK_FRONT → TURN_180 → BACKTRACK). Ultrasonic distance
under 20 cm counts as a wall. Turns are timed (410 ms for 90°, 950 ms for 180°)
with an 800 ms settle between manoeuvres.

---

## Colour response (TCS3200)

Calibrated against coloured cards to establish RGB thresholds, then each colour
triggers a behaviour: red → light show, blue → music, green → emergency stop,
yellow → 180° turn.

---

## Problems worth recording

**Reversed encoder.** One encoder was wired with its channels swapped, so it
counted backwards. Fixed by swapping the signal wiring.

**Two short circuits.** The first came from PCB pads that hadn't been drilled
out, overloading the ESP32. The second stopped the board being detected over USB
at all and took far longer — it turned out to be a single bridge of solder.
Continuity testing alone didn't find it; it needed a careful visual inspection
under magnification.

**Steering calibration.** A commanded 90° didn't produce a 90° turn in practice,
so the servo angle had to be trimmed against measured behaviour rather than
trusting the nominal value.

**Ambient light.** Colour detection worked reliably on the bench but failed
under different room lighting during assessment — the thresholds were calibrated
in one lighting condition and didn't generalise.

---

## What I'd do differently

- **Clamp the integral term.** With no windup limit, error accumulates
  indefinitely whenever the line is lost.
- **Close the loop on turns.** Timed turns drift with battery voltage; the
  encoders were already fitted and would have given far better repeatability.
- **Normalise the colour readings** against a clear-channel reference so
  detection survives changes in ambient light.
- **Design the PCB for probing.** Finding the solder bridge would have been much
  quicker with accessible test points.

---

## Files

| File | |
|---|---|
| `Line_following_code.ino` | PID line following, sensor calibration, I2C output |
| `Controller_firmware.ino` | Keypad/LCD remote, command entry, ESP-NOW transmit |
| `Master_Board_Navigation.ino` | Maze navigation state machine, ultrasonic sensing |

Built with Arduino IDE. Libraries: `ESP32Encoder`, `NewPing`, `Keypad`,
`LiquidCrystal`, `Wire`, `esp_now`.
