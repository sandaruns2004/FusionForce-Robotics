# 06 Calibration & Zeroing
## RUNNER-4 | Servo Zeroing + Software Offset Calibration

## Objective
Mechanically align all 15 servos to exactly 90° neutral, then calibrate sensor offsets via UART. This is the **single most important step** — IK math fails if servos are off-centre.

## Why Zeroing is Critical

```
The IK solver in kinematics.c assumes:
  command 90° -> physical leg segment exactly perpendicular to body

If servo horn attached 5° off-centre:
  command 90° -> physical angle is 85° or 95°
  Robot walks crooked, limps, or falls over

Example: if all 12 leg servos are 3° off:
  FR leg extends further than FL -> robot circles right
  Body pitch wrong -> IK solutions are all offset -> unstable trot
```

## Part A: Mechanical Servo Zeroing (do BEFORE attaching horns)

```
Process:
1. DO NOT attach servo horns or leg linkages yet
2. Wire all 15 servos to PCA9685 (CH0-CH14)
3. Open STM32 debug UART at 115200 baud (PA9)
4. Send command: "servo_all 90\r\n"
   -> All PCA9685 channels set to tick 307 (1500µs = 90°)
   -> All servos powered and actively holding 90°
5. While servos hold 90°, push servo spline horns onto shafts
   so that physical leg position matches your CAD reference angle
6. Immediately screw the servo horn M2.5 self-tapping screws
   (do NOT release power while screwing — servo will snap to new position)
```

### Reference Angles for Horn Attachment

| CH | Leg | Joint | Physical angle when servo at 90° |
|----|-----|-------|----------------------------------|
| 0 | FL | Coxa | Leg pointing straight sideways |
| 1 | FL | Femur | Femur at 45° from horizontal (partial lift) |
| 2 | FL | Tibia | Tibia extended 135° from femur axis |
| 3 | FR | Coxa | Leg pointing straight sideways (mirrored) |
| 4 | FR | Femur | Femur at 135° (180 - 45, mirrored) |
| 5 | FR | Tibia | Tibia at 45° from femur axis (mirrored) |
| 6 | BL | Coxa | Same as FL |
| 7 | BL | Femur | Same as FL |
| 8 | BL | Tibia | Same as FL |
| 9 | BR | Coxa | Same as FR |
| 10 | BR | Femur | Same as FR |
| 11 | BR | Tibia | Same as FR |
| 12 | Arm | Pitch | Arm pointing straight up (vertical HOME) |
| 13 | Gripper | Open/Close | Fingers at 60° open position |
| 14 | Gate | Lock/Open | Gate plate fully closed (locked) |

## Part B: Software Trim Offsets

Because servo splines have discrete teeth, mechanical zeroing is rarely perfect. Create a `calibration.h` offset file:

```c
/* firmware/Drivers/PCA9685/calibration.h */
#ifndef CALIBRATION_H
#define CALIBRATION_H

/**
 * Servo trim offsets in degrees.
 * Positive = servo physically reads too low (add offset to correct).
 * Negative = servo physically reads too high.
 *
 * Measure: command 90°, measure actual physical angle with protractor.
 * offset[i] = 90 - measured_angle
 *
 * Apply in pca9685.c:
 *   corrected_angle = commanded_angle + servo_trim_offsets[ch]
 */
const float servo_trim_offsets[15] = {
    /*CH0  FL Coxa  */ 0.0f,
    /*CH1  FL Femur */ 0.0f,
    /*CH2  FL Tibia */ 0.0f,
    /*CH3  FR Coxa  */ 0.0f,
    /*CH4  FR Femur */ 0.0f,
    /*CH5  FR Tibia */ 0.0f,
    /*CH6  BL Coxa  */ 0.0f,
    /*CH7  BL Femur */ 0.0f,
    /*CH8  BL Tibia */ 0.0f,
    /*CH9  BR Coxa  */ 0.0f,
    /*CH10 BR Femur */ 0.0f,
    /*CH11 BR Tibia */ 0.0f,
    /*CH12 Arm      */ 0.0f,
    /*CH13 Gripper  */ 0.0f,
    /*CH14 Gate     */ 0.0f,
};

#endif
```

### How to Measure and Fill the Offset Array

```
For each channel CH0 to CH14:
  1. Send UART command: "servo <CH> 90\r\n"
  2. With a digital protractor / angle gauge, measure actual physical angle
  3. offset = 90 - measured_angle
     (e.g., if servo reads 87° when commanded 90°: offset = +3)
  4. Fill calibration.h with measured offsets
  5. Rebuild and flash firmware

Repeat until all channels read 90° ± 1° when commanded 90°.
```

## Part C: IR Line Array Calibration (via UART)

```
1. Place all 9 TCRT5000 sensors over WHITE surface (arena white floor):
   UART: "la_cal white\r\n"
   -> Captures and stores cal_white[9] to STM32 Flash

2. Place all 9 sensors over BLACK surface (black line tape):
   UART: "la_cal black\r\n"
   -> Captures and stores cal_black[9] to Flash

3. Verify calibration:
   UART: "la_test\r\n"
   -> Returns line_val[0..8] on Serial
   Black surface: all values < 0.10
   White line:    active sensors  > 0.80
   Centre (S5):   centroid ~ 4.0 ± 0.3
```

## Part D: ToF Sensor Verification

```
1. UART: "tof_scan\r\n"
   -> Must report: FRONT=0x30 OK, LEFT=0x31 OK, RIGHT=0x32 OK

2. Place flat wall 200mm from front:
   UART: "tof_read\r\n"
   -> FRONT should read 195-205mm

3. Remove left wall:
   -> tof[LEFT].is_gap should become TRUE after 3 readings

4. Place robot in corridor (150mm walls each side):
   TOF_WallFollowError() should return ≈ 0 ± 10mm
```

## Part E: IMU Verification

```
1. Place robot on known-flat surface:
   UART: "imu_read\r\n"
   -> pitch_deg = 0 ± 3°
   -> roll_deg  = 0 ± 3°

If outside tolerance:
   a) Check MPU6050 is mechanically level (use a bubble level)
   b) Wait 5 seconds after power-on before reading (gyro warm-up)
   c) Add CF_ALPHA bias correction in mpu6050.c if systematic drift
```

## Full Calibration Checklist

```
[ ] All 15 servo horns attached at correct 90° reference angles
[ ] calibration.h filled with measured trim offsets for CH0-CH14
[ ] la_cal white  -> saved to Flash
[ ] la_cal black  -> saved to Flash
[ ] IR test: black < 0.10, white > 0.80, centroid ±0.3
[ ] ToF scan: all three sensors found (0x30, 0x31, 0x32)
[ ] ToF distance: 200mm wall reads 195-205mm
[ ] IMU: pitch=0 ±3° and roll=0 ±3° on flat surface
[ ] Arm HOME (90°): sensor vertical, no chassis contact
[ ] Gripper OPEN (60°): fingers clear of 40mm ball
[ ] Gate LOCKED (0°): plate fully sealed, no ball gap
```

---
🔙 **[Back to Mechanical](../README.md)**
