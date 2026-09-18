# 04 Electronics Mounting
## RUNNER-4 | STM32F411CEU6 + PCA9685 + Sensor Suite

## Objective
Securely mount all electronics to withstand continuous walking vibrations and ensure correct sensor field-of-view (FoV) placement.

## Electronics Stack Layout

```
  TOP DECK (upper plate):
  ┌─────────────────────────────────────────┐
  │  STM32F411CEU6 Black Pill               │  <- Screwed into M2.5 brass standoffs
  │  (on perfboard/custom PCB)              │
  │                                         │
  │  PCA9685 PWM Driver (0x40)              │  <- Screwed into M2.5 standoffs
  │  [4.7kΩ pull-ups I2C1 SDA/SCL]         │
  └─────────────────────────────────────────┘

  SENSOR POSITIONS (top deck, front face):
  ┌─────────────────────────────────────────┐
  │  [VL53L0X LEFT]  [VL53L0X FRONT]  [VL53L0X RIGHT]  │
  │  addr 0x31        addr 0x30         addr 0x32       │
  │  XSHUT PB13       XSHUT PB12       XSHUT PB14      │
  │                                                     │
  │  [MPU6050 IMU] — mount at geometric centre, flat    │
  │  addr 0x68  I2C1                                    │
  └─────────────────────────────────────────┘

  BOTTOM DECK (base plate):
  ┌─────────────────────────────────────────┐
  │  LiPo Battery 2S/3S        (Velcro strap, centre)  │
  │  Switching BEC 6V ≥5A      (Zip-tie side mount)    │
  │  3.3V LDO/SMPS             (Side mount)            │
  └─────────────────────────────────────────┘

  FRONT EDGE (IR sensor bar):
  ┌─────────────────────────────────────────┐
  │  9x TCRT5000 IR sensors (S1-S9)         │
  │  PA0-PA7, PB0 → ADC1 DMA               │
  │  Mounted 5mm above arena floor          │
  │  80mm total width, 10mm pitch           │
  └─────────────────────────────────────────┘
```

## Component List

| Component | Mount Location | Mount Method | Notes |
|-----------|---------------|--------------|-------|
| STM32F411CEU6 | Top deck | M2.5 nylon standoffs + rubber washers | Vibration isolation |
| PCA9685 | Top deck | M2.5 brass standoffs | Keep away from IR sensors |
| MPU6050 IMU | Top deck, geometric centre | M2.5 standoffs, perfectly flat | ±1° tilt tolerance |
| TCS34725 Colour Sensor | Arm tip (CH12) | M2 screws | Moves with arm |
| VL53L0X Front | Front edge, centre | M2 mount, forward-facing | 0° horizontal |
| VL53L0X Left | Left edge | M2 mount, 90° left-facing | Exactly perpendicular |
| VL53L0X Right | Right edge | M2 mount, 90° right-facing | Exactly perpendicular |
| 9x TCRT5000 IR | Underside front bar | Custom PCB / perfboard | 5mm from floor |
| Switching BEC | Bottom deck | Zip-tie | 6V servo power |
| LiPo Battery | Bottom deck centre | Velcro strap | CoG alignment critical |

## I2C Bus Wiring

```
I2C1 (SDA=PB7, SCL=PB6) — 4.7kΩ pull-ups to 3.3V:
  PCA9685      @ 0x40   (PWM driver, 15 servos)
  MPU6050      @ 0x68   (IMU)
  TCS34725     @ 0x29   (colour sensor on arm)

I2C2 (SDA=PB3, SCL=PB10) — 4.7kΩ pull-ups to 3.3V:
  VL53L0X Front @ 0x30  (remapped from 0x29 via XSHUT PB12)
  VL53L0X Left  @ 0x31  (remapped from 0x29 via XSHUT PB13)
  VL53L0X Right @ 0x32  (remapped from 0x29 via XSHUT PB14)
```

> **Why separate I2C buses?** VL53L0X readings take ~30ms each. If on I2C1 with PCA9685, ToF measurements would block servo updates. I2C2 isolates them completely.

## ADC / DMA Wiring

```
ADC1 + DMA1 (circular scan, 16x oversampling):
  S1 PA0   S2 PA1   S3 PA2   S4 PA3   S5 PA4
  S6 PA5   S7 PA6   S8 PA7   S9 PB0

  Pull-up resistors: 10kΩ to 3.3V for each TCRT5000 emitter
  DMA buffer: 9 × 16 = 144 uint32_t words
```

## GPIO Assignments

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| PB12 | VL53L0X FRONT XSHUT | Output PP | LOW=reset, HIGH=active |
| PB13 | VL53L0X LEFT XSHUT | Output PP | |
| PB14 | VL53L0X RIGHT XSHUT | Output PP | |
| PC0 | TCS34725 LED enable | Output PP | HIGH=LED ON |
| PC13 | Start button | Input PU | Active LOW (press to start) |
| PA9 | USART1 TX | Alt AF7 | Debug 115200 baud |
| PA10 | USART1 RX | Alt AF7 | |

## Sensor Placement Rules

### MPU6050 IMU (CRITICAL)
- Must be at **exact geometric centre** of robot body
- Must be **perfectly horizontal** (±1° tolerance)
- If tilted, the complementary filter will report false pitch/roll and the state machine will trigger `IMU_TIP_THRESHOLD` incorrectly
- Mount with rubber washers to dampen servo vibration from IMU accelerometer

### VL53L0X ToF Sensors
- Front sensor: pointing **dead ahead**, 0° from robot X-axis
- Left sensor: pointing **exactly 90° left**, perpendicular to robot Y-axis
- Right sensor: pointing **exactly 90° right**
- No chassis parts in the 25° cone FoV — check by shining a torch through the FoV
- Sensors mounted at **same height** as corridor wall mid-height (approx. 40mm from floor)

### IR Line Array (9x TCRT5000)
- Mounted **5mm above arena floor** — optimal TCRT5000 sensing distance
- Total width: 80mm, pitch: 10mm between sensors (S1-S9 left to right)
- S5 (PA4, centre) must align with robot X-axis centreline
- Sensor PCB must be rigid — flex PCB causes variable readings

## Vibration Mitigation

- Use **nylon M2.5 standoffs** + rubber washers for all PCB mounts
- Apply **hot glue** or cable tie to all I2C/power connector joints (arena vibrations loosen DuPont connectors)
- Route servo cables along structural members, not free-hanging

---
🔙 **[Back to Mechanical](../README.md)**
