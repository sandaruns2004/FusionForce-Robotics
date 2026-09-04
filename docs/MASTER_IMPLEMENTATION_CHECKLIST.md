# MASTER IMPLEMENTATION CHECKLIST

## 1. Hardware & Electronics
- `[ ]` 3S/2S LiPo battery tested and fully charged.
- `[ ]` 5V/15A BEC output voltage verified at exactly 5.0V–6.0V using multimeter.
- `[ ]` STM32F411CEU6 Black Pill powers on via 3.3V or USB.
- `[ ]` Common Ground (GND) wire physically connected between Battery, BEC, STM32, and all sensors (star topology).
- `[ ]` PCA9685 I2C driver board connected to STM32 I2C1 (PB6/PB7).
- `[ ]` All 12× MG90S leg servos connected to PCA9685 CH0–CH11.
- `[ ]` Arm pitch servo connected to PCA9685 CH12.
- `[ ]` Gripper servo connected to PCA9685 CH13.
- `[ ]` Storage gate servo connected to PCA9685 CH14.
- `[ ]` 3× VL53L0X ToF sensors connected to STM32 I2C2 (PB10/PB3); XSHUT pins on PB12, PB13, PB14.
- `[ ]` MPU6050 IMU connected to STM32 I2C1 (PB6/PB7, address 0x68).
- `[ ]` **8-channel TCRT5000 IR line array** connected to STM32 GPIO PA0–PA7 (digital input, pull-down).
- `[ ]` **TCS34725 colour sensor** connected to STM32 I2C1 (PB6/PB7, address 0x29); LED control on PC0.
- `[ ]` TCS34725 mounted securely on gripper arm tip with light-shielding shroud.
- `[ ]` Line array mounted on front-underside bracket, 5–8mm above floor, centred on robot midline.
- `[ ]` Hardware kill switch installed on battery main positive line.
- `[ ]` Battery voltage divider connected to PC1 (ADC) for low-battery monitoring.

## 2. STM32 Firmware (Complete System)
- `[ ]` STM32CubeMX project created targeting **STM32F411CEU6** at 100MHz.
- `[ ]` I2C1 (PB6/PB7, 400kHz) and I2C2 (PB10/PB3, 400kHz) configured.
- `[ ]` GPIO PA0–PA7 configured as digital input with pull-down.
- `[ ]` GPIO PB12, PB13, PB14 configured as output (XSHUT pins).
- `[ ]` GPIO PC0 configured as output (TCS34725 LED).
- `[ ]` ADC1 CH11 (PC1) configured for battery monitoring.
- `[ ]` TIM2 configured for 50Hz (20ms) interrupt.
- `[ ]` PCA9685 I2C communication verified (servo sweep test on all channels).
- `[ ]` MPU6050 I2C reading verified (stable roll/pitch values).
- `[ ]` VL53L0X XSHUT address remap verified (3 sensors at 0x30, 0x31, 0x32 confirmed by I2C scan).
- `[ ]` **8-channel line array GPIO reading verified** (paper white/black test; correct sensors HIGH).
- `[ ]` **TCS34725 I2C driver working** (raw R/G/B/Clear values readable via debug UART).
- `[ ]` **TCS34725 non-blocking integration** implemented (start → poll AVALID → read, no loop blocking).
- `[ ]` **Colour classification algorithm verified** (Red/Green/Blue paper samples → >95% accuracy).
- `[ ]` **Weighted centroid line following** verified on 30mm white line (centroid ≈ 3.5 when centred).
- `[ ]` **Intersection detection** (≥6 sensors, ≥3 consecutive cycles) verified with paper cross pattern.
- `[ ]` Inverse Kinematics (IK) math implemented and unit tested.
- `[ ]` Crawl gait generator implemented (Bezier foot trajectory, FL→BR→FR→BL sequence).
- `[ ]` Postural PID controller implemented (IMU pitch/roll → foot Z corrections).
- `[ ]` **18-state Mission State Machine implemented** in C (all subtasks Task 1–4).
- `[ ]` **`stored_ball_color` Flash EEPROM emulation** working (write after Task 1; read on boot; survives reset).
- `[ ]` Arm position MODE A (0°) and MODE B (−70°) PWM constants calibrated and stored.
- `[ ]` Safety watchdogs: tilt >30° → SAFE_STOP; line lost >3s → SAFE_STOP; battery <6.4V → halt.
- `[ ]` **50Hz loop timing profiled** with DWT counter — verified <18ms total with all sensors active.

## 3. Mechanical & Assembly
- `[ ]` Chassis baseplate 3D printed (PETG).
- `[ ]` 12× Servo brackets 3D printed.
- `[ ]` 4× Legs assembled.
- `[ ]` Actuator angle limits mechanically verified (no self-collision).
- `[ ]` 2-DOF Front Arm + Gripper printed and assembled.
- `[ ]` **TCS34725 tip mount** 3D printed and fitted at arm tip with light shroud.
- `[ ]` Arm reach verified: arm at 0° reaches 5cm pedestal height ±5mm.
- `[ ]` **Line array bracket** 3D printed and mounted at 5–8mm floor height.
- `[ ]` Line array alignment verified (centred, perpendicular to robot forward axis).
- `[ ]` Integrated Bumper Plate attached to front-bottom chassis.
- `[ ]` Components mounted (STM32, BEC, battery) for symmetrical CoG.
- `[ ]` Cables routed neatly; no snagging during leg swing.
- `[ ]` Total robot weight verified **<550g** (weigh before competition).

## 4. Integration & Testing

| # | Test | Pass Criteria |
|---|------|--------------|
| 1 | Single servo sweep | Full range, no jitter |
| 2 | One-leg IK | Smooth foot positioning |
| 3 | One-leg loaded | Supports 200g without stall |
| 4 | 4-leg standing | Stable, within 250×250mm footprint |
| 5 | Crawl walking + turning | 1m forward; 90° turn reliable |
| 6 | IMU stabilisation | Level on ±5° tilted surface |
| 7 | Wall following (Task 2 corridor) | Centres in 30cm corridor; handles 1 gap |
| 8 | **Line following** (8-sensor) | Follows 2m curved line without losing track |
| 9 | **Intersection detection** | Triggers correctly at paper cross; no false triggers on curves |
| 10 | **Ball colour detection** (TCS34725 MODE A) | Correct colour for Red/Green/Blue ball; >95% accuracy |
| 11 | **Floor zone detection** (TCS34725 MODE B) | Correct branch selection at paper 3-way junction |
| 12 | Ball pickup + store | Reliable grab from 5cm pedestal; stored securely |
| 13 | Obstacle push (Task 3) | Pushes 30cm without tipping; detects cleared |
| 14 | **Flash colour persistence** | Reset STM32 mid-run; verify colour recalled for Task 4 |
| 15 | **Full circuit (Task 1→4)** | All 4 subtasks completed autonomously |
| 16 | Endurance | Full 15-min run; no servo brownout or watchdog trigger |

## 5. Pre-Competition Final Checks
- `[ ]` Footprint ≤ 250×250mm at start position (measure with jig).
- `[ ]` Robot stands independently without assistance.
- `[ ]` Single start switch works; robot responds on first press.
- `[ ]` Debug UART cable **physically disconnected** (competition rule: no external comms).
- `[ ]` Wi-Fi/Bluetooth on any device turned off (robots must not emit wireless signals).
- `[ ]` Battery fully charged (<8.4V for 2S).
- `[ ]` Line array thresholds tuned under arena lighting (2-min prep window).
- `[ ]` TCS34725 colour classification verified under arena lighting (2-min prep window).
- `[ ]` Flash memory contains no stale ball colour from previous test run (or intentionally cleared).
