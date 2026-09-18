# TROUBLESHOOTING GUIDE

## 1. Overview
This guide covers common issues encountered during development and testing of the RUNNER-4 embedded system. All troubleshooting uses a debug UART (USART1, PA9 TX, 115200 baud) and ST-Link SWD debugger during development.

> [!WARNING]
> The debug UART cable MUST be physically disconnected before competition. The competition rules prohibit any external wired communication during the run.

---

## 2. STM32 Boot & General Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| STM32 not powering on | 3.3V LDO missing; USB not connected | Verify STM32 VBUS pin has 5V; check LDO output with multimeter |
| Firmware not loading | ST-Link not detected | Check SWD wires (SWDIO, SWDCLK, GND); press BOOT0 + RESET then release RESET |
| Loop runs at wrong speed | TIM2 misconfigured | Verify TIM2 period = `(APB1_clock / prescaler) - 1` gives 50Hz; check with oscilloscope or DWT |
| I2C devices not found | Wrong bus or pull-up | Run I2C address scan; verify PB6/PB7 (I2C1) and PB10/PB3 (I2C2) have 4.7kΩ pull-ups to 3.3V |

---

## 3. 9-Channel TCRT5000 IR Line Array (ADC1 DMA)

### Sensor Calibration Procedure
1. Power on STM32 with debug UART connected.
2. Place array over **plain black surface**. Send `la_cal black\r\n` via UART.
   - All 9 normalised values should approach 0.0. If any stay above 0.1, check mounting height (should be 5mm).
3. Place array over **30mm white matte paper**. Send `la_cal white\r\n` via UART.
   - Active sensors over white should return normalised value > 0.8.
4. Send `la_test\r\n`. When centred on line, centroid ≈ **4.0**. error ≈ 0.0.
5. Slide array slowly sideways — verify smooth centroid transition, no discontinuities.

### Common Line Array Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| All sensors return same high value | ADC DMA not started — `LA_Init()` not called | Verify `HAL_ADC_Start_DMA()` called in `App_Init()` |
| All sensors return same low value | 10kΩ pull-ups missing or ADC reference wrong | Check pull-ups on PA0-PA7, PB0; verify VREF=3.3V |
| Erratic readings at line edge | Sensor mounted too high (>8mm) or too low (<3mm) | Adjust bracket to exactly 5mm height |
| Line lost mid-run | Line array vibrating loose | Tighten bracket screws; add vibration-damping foam |
| `la_cal` not saving | Flash write failed | Verify Flash sector not write-protected; re-flash |
| S9 (rightmost) not responding | PB0 not in ADC1 scan sequence in CubeMX | Re-check CubeMX: ADC1 regular channel 8 = PB0 IN8 |
| Centroid stuck at 0 or 7 | All sensors on one side active | Robot is too far off-line; PD gain too low → increase Kp |
| False intersection triggers | Wide curve activating 6+ sensors | Raise intersection threshold from 6 to 7 sensors |

---

## 4. TCS34725 Colour Sensor

### Colour Classification Calibration (2-Minute Pre-Competition Procedure)
1. Connect debug UART; power robot.
2. Turn on LED (PC0 HIGH).
3. Hold arm in **MODE A** position (0°, ball-reading angle).
4. Hold a **RED coloured object** 2cm from sensor. Log `r_n, g_n, b_n` values from UART.
   - Expected: `r_n > 0.40`, `r_n >> g_n`, `r_n >> b_n`.
5. Repeat with **GREEN** object: expected `g_n > 0.35`, dominant.
6. Repeat with **BLUE** object: expected `b_n > 0.30`, dominant.
7. If ratios are borderline: adjust gain constant in firmware (`TCS34725_GAIN_16X` instead of `4X`) and reflash.

### Arm Position Calibration
1. Set arm servo to `MODE A` PWM constant.
2. Verify arm is horizontal (use a spirit level or angle gauge).
3. Verify sensor tip is ~5cm above ground (matches ball pedestal height).
4. Set arm servo to `MODE B` PWM constant.
5. Verify arm points ~70° below horizontal.
6. Hold sensor 2cm above a colour line on the floor → verify reading.

### Common TCS34725 Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Always returns `COLOR_UNKNOWN` | Sensor too far from target or wrong gain | Move arm closer; increase gain to 16× |
| Wrong colour classified | Arena lighting interfering | Turn LED ON (PC0 HIGH); increase integration time; shroud sensor |
| I2C NACK on address 0x29 | Address conflict with VL53L0X (wrong bus) | Verify TCS34725 is on I2C1 and VL53L0X on I2C2 |
| Reading blocks control loop | Blocking I2C read inside 20ms loop | Implement non-blocking: start integration, poll AVALID bit on next cycles |
| Colour flickers between RED/UNKNOWN | Too close to ball edge | Ensure arm is fully lowered to pedestal height before reading |

---

## 5. VL53L0X Time-of-Flight Sensors

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| All 3 ToF at same address 0x29 | XSHUT remap failed | Verify PB12/13/14 are configured as GPIO output; step through remap procedure with `HAL_Delay` |
| ToF reads 0 or 8191 | Sensor initialisation failed | Add 10ms delay after XSHUT release before sending init command |
| Erratic distance readings | Infrared cross-talk between sensors | Separate sensors physically; use median filter (5 samples) |

---

## 6. Mission State Machine

### Debug UART State Trace
Enable state transition logging during development:
```c
// In state_machine.c:
void StateMachine_Transition(State_t new_state) {
    printf("[SM] %s → %s\r\n", state_names[current_state], state_names[new_state]);
    current_state = new_state;
}
```
This allows tracing the full mission flow via a serial terminal (disable before competition).

### Flash EEPROM — Ball Colour Persistence Debug
To verify Flash write/read:
```c
// Test sequence:
Flash_StoreBallColor(COLOR_RED);
HAL_NVIC_SystemReset();  // Simulate restart
// On next boot:
uint8_t c = Flash_ReadBallColor();
printf("Recalled: %d (expect 1=RED)\r\n", c);
```
If value is `0xFF`, the Flash write failed — check sector erase + program sequence.

---

## 7. Power & Servo Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Servos twitch randomly | Servo rail brownout from current spike | Add 4700µF cap across PCA9685 V+/GND; check BEC rating ≥15A |
| STM32 resets when servos move | Shared power rail | Verify STM32 powered from separate 5V buck, not BEC |
| Servo jitter on one channel | Loose connector or wrong PWM value | Re-seat connector; verify PWM tick calculation (50Hz, 4096 ticks = 20ms) |
| Robot collapses when lifting | MG90S torque insufficient | Verify 6V supply; reduce mass; check for binding in joint |
