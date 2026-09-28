# 03 Gripper & Gate Mechanism
## RUNNER-4 | CH12 Arm Pitch | CH13 Gripper | CH14 Gate

## Objective
Build the three mechanism servos that handle ball retrieval (Task 1), storage, and ball release into the colour-sorted zone (Task 4).

## Mechanism Overview

```
  SIDE VIEW OF ROBOT:

  HOME (CH12 = 90°):           ARM pointing UP — safe for travel/walk

  MODE_A (CH12 = 0°):          ARM horizontal FORWARD
                                Sensor at ball height (~5cm above pedestal)
                                Use for: Task 1 ball approach + colour ID

  MODE_B (CH12 = 160°):        ARM angled DOWN to floor
                                Sensor 2cm above floor zone
                                Use for: Task 4 zone colour identification

  Gripper CH13:
    OPEN  = 60°  → fingers wider than 40mm ball diameter
    CLOSE = 115° → fingers grip ball firmly

  Gate CH14 (belly storage door):
    LOCKED = 0°  → plate closed, retains ball inside body
    OPEN   = 90° → plate rotates, ball rolls out into zone
```

## PCA9685 Channels — Mechanism Servos

| CH | Function | Servo | HOME tick | Key positions |
|----|----------|-------|-----------|--------------|
| 12 | Arm pitch | MG90S | 307 (90°) | 102=MODE_A(0°), 450=MODE_B(160°), 380=STORE(135°) |
| 13 | Gripper | MG90S | 184 (60°) | 184=OPEN(60°), 286=CLOSE(115°) |
| 14 | Gate | MG90S | 102 (0°) | 102=LOCKED(0°), 307=OPEN(90°) |

## Torque Budget

| Servo | Load | Required | MG90S @ 6V | Margin |
|-------|------|----------|-----------|--------|
| CH12 Arm | 40mm ball at 65mm arm | 2.35 kg·cm | 2.2 kg·cm | **0.94x — marginal** |
| CH13 Gripper | Ball grip force | 1.02 kg·cm | 2.2 kg·cm | 2.15x OK |
| CH14 Gate | Spring-loaded door | 0.76 kg·cm | 2.2 kg·cm | 2.89x OK |

> **⚠️ CH12 Arm Warning:** Torque margin is below 1.0x at maximum arm extension.
> Fix options: (A) Shorten arm link to 55mm, (B) Upgrade CH12 to MG996R (10 kg·cm), (C) Add counterweight.

## Hardware Required

| Part | Qty | Notes |
|------|-----|-------|
| MG90S metal-gear servo | 3 | CH12, CH13, CH14 |
| Arm link (PETG, 100% infill) | 1 | 65mm long, servo horn mount one end, TCS34725 mount other end |
| Gripper fingers (PETG or TPU) | 2 | Line with rubber band or foam tape for friction |
| Gate plate (PETG, 60% infill) | 1 | Hinged to belly of chassis |
| M3×8 screws | 8 | Servo mounting |
| M2 screws | 4 | TCS34725 sensor mount on arm tip |

## TCS34725 Colour Sensor on Arm

The TCS34725 colour sensor mounts at the **tip of the arm link**:
- ARM_MODE_A (0°): sensor faces ball at pedestal height → identifies ball colour
- ARM_MODE_B (160°): sensor faces floor zone → identifies floor zone colour for Task 4

```
  Arm tip wiring:
  TCS34725 -> I2C1 bus (SDA=PB7, SCL=PB6) shared with MPU6050
  LED control -> PC0 (GPIO output, HIGH = white LED ON for measurement)
  I2C address: 0x29
```

## Sequence per Task

### Task 1 — Ball Retrieval
```
1. SM: T1_LINE_FOLLOW  -> ARM_HOME,  GRIPPER_OPEN,  GATE_LOCKED
2. SM: T1_BALL_APPROACH-> ARM_MODE_A, GRIPPER_OPEN,  GATE_LOCKED  (800ms)
3. SM: T1_COLOR_ID     -> ARM_MODE_A, GRIPPER_OPEN,  GATE_LOCKED  (3 stable reads)
4. SM: T1_BALL_GRAB    -> ARM_MODE_A, GRIPPER_CLOSE, GATE_LOCKED  (1200ms)
5. SM: T1_STORE        -> ARM_HOME,   GRIPPER_CLOSE, GATE_LOCKED  (1000ms)
```

### Task 4 — Ball Release
```
6. SM: T4_JUNCTION     -> ARM_MODE_B, GRIPPER_CLOSE, GATE_LOCKED  (floor zone ID)
7. SM: T4_BALL_RELEASE -> ARM_HOME,   GRIPPER_OPEN,  GATE_OPEN    (1500ms, ball exits)
```

## Assembly Steps

1. Mount CH12 arm servo to chassis front face, servo shaft pointing UP
2. Attach arm link to servo horn (do NOT attach horn until Module 06 zeroing)
3. Mount TCS34725 PCB to arm tip with M2 screws, LED facing outward
4. Route TCS34725 I2C cable along arm, secure with small zip ties at 10mm intervals — must not snag during arm movement
5. Mount CH13 gripper servo to arm link midpoint
6. Attach finger assembly to CH13 servo horn
7. Line finger interior with 2mm foam tape or rubber band strips
8. Mount gate plate hinge to belly chassis with M3×16 bolts
9. Attach CH14 gate servo to chassis side, servo horn attached to gate plate
10. Verify gate: at 0° plate fully closes, at 90° plate swings clear (ball can exit)

## Debugging

| Problem | Cause | Fix |
|---------|-------|-----|
| Arm droops under ball weight | CH12 margin too low | Shorten arm link or upgrade to MG996R |
| Gripper can't hold ball | Fingers too smooth | Add rubber band / foam tape lining |
| Gate doesn't open fully | Servo horn angle wrong | Re-zero CH14 at Module 06 |
| Colour reads wrong at MODE_A | LED (PC0) not powered | Verify PC0 GPIO HIGH before reading |
| Arm cable snags during walk | Cable too short/loose | Add 50mm slack loop, zip-tie to chassis |

---
🔙 **[Back to Mechanical](../README.md)**
