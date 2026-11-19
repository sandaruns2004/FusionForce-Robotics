# RUNNER4 v5.0 — 9-Channel Line Array Integration Report

## 1. What was delivered

| File | Status | Purpose |
|---|---|---|
| `RUNNER4_Robot_Line.ino` | **Updated** (full file) | Main robot; adds line module, new I2C pins, `linePoll()` in all waits |
| `line_sensor.h` | **New** | Public API + pin list (`LINE_PIN_LIST`) |
| `line_sensor.cpp` | **New** | Scan, calibration, centroid, PD, junction, lost/safe-stop, web handlers, NVS config |
| `line_page.h` | **New** | `/line` web page (Line Array Lab) |
| `color_sensor.h` | **Changed (1 line)** | `COLOR_LED_PIN` 4 → 15 |
| `robot_page.h` | **Changed (1 line, you apply)** | Add button to `/line` (see §6) |
| `color_sensor.cpp`, `color_page.h` | Unchanged | — |

All files go in ONE folder named `RUNNER4_Robot_Line`.

## 2. Strategy carried over from the colour module

| Colour module pattern | Line module equivalent |
|---|---|
| Anonymous namespace, small public API | Same: `lineBegin / linePoll / lineFound / lineErrorMm / lineWz / ...` |
| Own web root `/color/*` | Own web root `/line/*` (`data, config, set, cmd, wifi, export`) |
| Non-blocking `colorPoll()` called from `loop()` and every motion wait | `linePoll()` in the same two places via `serviceBackground()` |
| Calibration wizard in the browser | 5 s sweep calibration in the browser |
| Config in NVS (`Preferences`), magic number, defaults fallback | Same, namespace `"line"`, magic `0x4C494E02` |
| Filter pipeline (EMA, vote, confirm) | EMA, threshold, centroid, N-cycle junction confirm |
| Export constants for the other platform | `/line/export` → paste into the standalone sketch |
| Health page (heap, rate, errors) | `/line/wifi` (rate, scan time, pins, cal state) |

## 3. Pin changes (the important part)

**Constraint:** the WiFi AP is always on. On ESP32-S3, **ADC2 (GPIO 11–20) cannot be read while WiFi is active**, so all 9 sensors must be on **ADC1 = GPIO 1–10**. GPIO 3 is a strapping pin and is skipped.

That needs 9 of the 10 ADC1 pins, but GPIO 4 (colour LED) and GPIO 8/9 (I2C) were already taken. Resolution:

| Function | v4.0 | v5.0 | Why |
|---|---|---|---|
| I2C SDA | GPIO 8 | **GPIO 11** | frees ADC1 pin; I2C works on any GPIO |
| I2C SCL | GPIO 9 | **GPIO 12** | same |
| TCS34725 LED | GPIO 4 | **GPIO 15** | frees ADC1 pin |
| S1 … S9 | – | **1, 2, 4, 5, 6, 7, 8, 9, 10** | all ADC1 |

**Rewiring needed:** move SDA/SCL wires (both PCA9685 and TCS34725 share them) to GPIO 11/12, and the TCS LED wire to GPIO 15. Servo wiring (via PCA9685) is untouched.

If you prefer not to move I2C: define fewer sensors is not supported by the current code without edits, so the rewire is the recommended path. Pins are all in two places only: `LINE_PIN_LIST` (line_sensor.h) and `I2C_SDA/I2C_SCL` (.ino).

**Electrical:** power the TCRT5000 modules from **3.3 V** so analog outputs never exceed 3.3 V.

## 4. Upgrades vs the standalone sketch (with reasons)

| # | Standalone sketch | v5.0 module | Benefit |
|---|---|---|---|
| 1 | `delay(5)` blocking 5 s calibration at power-up, array must be swept immediately | Web-triggered, **non-blocking** sweep with progress bar | Robot boots and serves WiFi normally; calibrate whenever the floor/lighting changes |
| 2 | Calibration lost on every reset | Saved to **flash (NVS)**; loaded at boot | No recalibration per power cycle |
| 3 | `RAW_TEST` compile flag | Live raw + normalised bars for all 9 sensors on `/line` | No reflashing to inspect raw values |
| 4 | Constants (`Kp`, `Kd`, threshold, polarity, lost limit) need re-upload | All tunable live from the browser, saved to flash | Tune on the floor without a USB cable |
| 5 | 8 ADC reads per sensor (72 reads per scan) always | `samples` adjustable 1–16, default 4 | Shorter scan; less CPU stolen from servo timing |
| 6 | No filtering on raw | Optional EMA on raw (α default 0.7) | Less centroid jitter, less derivative noise |
| 7 | Dead/flat sensor: returns 0 silently | Per-sensor health (`okmask`), faded in UI, listed after calibration | Spot a broken sensor immediately |
| 8 | Junction = bool only | Confirmed **and typed**: LEFT / RIGHT / T/CROSS / WIDE | Needed for branch decisions with the colour ball test |
| 9 | Lost line → prints, safe stop only text | State machine: `LINE LOST` → `SAFE STOP`, with last-side memory, ms counter, API flags | Robot logic can react (`lineSafeStop()`) |
| 10 | Wz output unbounded | Clamped to ±`wzMax` | Prevents extreme steering commands |
| 11 | Fixed 50 Hz via `delay(20)` | Rate-limited, non-blocking cycle (default 20 ms) with measured Hz and scan time | Works alongside gait code; real rate visible |
| 12 | Serial print every loop | Optional serial log toggle (same format as before) | Keeps USB quiet unless debugging |
| 13 | Spacing hard-coded | `spacing` tunable | Different array pitch without recompiling |
| 14 | – | `/line/export` generates `calMin/calMax/threshold/Kp/Kd` | Move tuned values back to the standalone sketch |
| 15 | – | Error history chart + position marker | Visual PD tuning |

Behaviour kept identical to the tested code: weighted centroid, threshold zeroing, `(centroid − 4) × spacing` error, `Kp·err + Kd·Δerr`, junction `≥ 6` sensors for `≥ 3` cycles, 3000 ms lost limit.

## 5. Changes to existing files

**`RUNNER4_Robot_Line.ino`**
- Header/wiring comments rewritten for the new pin map; version 5.0.
- `#include "line_sensor.h"`.
- `I2C_SDA 8 → 11`, `I2C_SCL 9 → 12`.
- `serviceBackground()` now also calls `linePoll()`; `loop()` calls `linePoll()`.
- `setup()`: `lineBegin(server, AP_SSID)` before `server.begin()`; banner/log updated.
- Gaits, poses, IK/FK, job system, handlers: **unchanged**.

**`color_sensor.h`**: `COLOR_LED_PIN 4 → 15` (must be changed here, not in the .ino, because `color_sensor.cpp` includes this header separately).

## 6. Manual edit in `robot_page.h`

In the header block, next to the colour button, add:

```html
<a class="btn btn-col" href="/line">&#128739; Line Array</a>
```

(The `/line` page already links back to `/` and `/color`.)

## 7. First-run procedure

1. Rewire (§3), put all files in the `RUNNER4_Robot_Line` folder, flash.
2. Serial monitor should show `[LINE] 9-ch array, calibration MISSING`.
3. Join `RUNNER4-Config`, open `http://192.168.4.1/line`.
4. Hold the array over black floor, check raw values move on a white line. If white reads **low**, untick *White line is HIGH*.
5. **Start 5 s sweep**, slide across black and white; confirm all 9 sensors are bright (not faded). **Save to flash**.
6. Tune threshold first (~0.5), then `Kp`, then `Kd`. Watch the error chart and Wz.
7. Test lost (lift array) → `LINE LOST` then `SAFE STOP` after the limit; test a cross-line → `JUNCTION · T/CROSS`.

## 8. Known limits and next phase

- **No closed-loop walking yet.** This version delivers sensing, tuning and the API; the robot does not steer itself. Intended next step: a `JOB_LINE` gait that calls `lineWz()` once per gait cycle (Wz < −dead-band → turn-left cycle, > +dead-band → turn-right cycle, else walk cycle), stops on `lineSafeStop()`, and uses `lineJunction()` + `colorConfirmed()` vs `colorStoredBall()` for branch choice.
- Quadruped gaits move in discrete cycles, so `Kp/Kd` tuned on a wheeled/static test will need re-tuning; derivative term should be evaluated per gait cycle, not per 20 ms.
- Scan time (~1–3 ms per cycle at default settings) is small but not zero: check the `scan µs` value on `/line` and lower `samples` if gait timing feels affected.
- Servo motion can inject supply noise on the analog lines; keep sensor ground/3.3 V wiring short and separate from the 6 V servo rail, add a 100 nF cap per sensor output if readings jitter while walking.
- Sensor values depend on height above the floor: calibrate at the real operating height.
- Page polls every 150 ms; the single-threaded server handles this together with the colour page, but avoid opening many browser tabs at once.
