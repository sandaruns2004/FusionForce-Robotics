# 06. System Integration, Calibration & Testing Guide

## 1. Executive Summary & Integration Quality Control

Before attempting autonomous competition runs on the track (`Task/tasks_circuit_v1.md`), every subsystem must undergo systematic calibration and verification. This guide defines standard calibration procedures for sensors/actuators and four rigorous unit integration tests.

---

## 2. Step-by-Step Hardware Calibration Procedures

### 2.1 Servo Horn Zero-Point Trimming (12 Leg Servos + 2 Arm Servos)
1. **Power Up PCA9685 at 1500 µs Neutral:** Send PWM pulse width `1500 µs` (90 degrees) to all 16 channels without servo horns attached.
2. **Mechanical Horn Attachment:**
   - Attach the **Coxa horn** so the leg extends orthogonally (90° lateral to the body chassis).
   - Attach the **Femur horn** horizontally parallel to the ground (0° inclination).
   - Attach the **Tibia horn** perpendicular (90° downward toward the floor).
3. **Firmware Trim Table Calibration:**
   - Use a digital protractor or right-angle square to measure joint angular errors.
   - Enter offsets ($\pm 15^\circ$ max) into the STM32 Flash trim array `SERVO_TRIM_DEG[14]`.

```
        SERVO MECHANICAL ZERO ALIGNMENT (Front View)
        
        [Chassis Body] -- (Coxa: 90 deg) -- O [Femur: 0 deg Horizontal]
                                             \
                                              \  [Tibia: 90 deg Downward]
                                               O --- (Foot Tip at Floor)
```

---

### 2.2 VL53L0X ToF Sensor Cross-Talk & Offset Calibration
1. **Zero-Offset Trim:** Place a matte white target block exactly **100.0 mm** in front of each sensor. Compare `tof_raw_mm` reading against 100 mm and record static offset `TOF_OFFSET_MM[3]`.
2. **Cross-Talk Suppression:**
   - Ensure a **thin black neoprene baffle tube** surrounds each ToF lens.
   - Verify that when `ToF_Left` fires, `ToF_Right` and `ToF_Front` register zero interference counts.

---

### 2.3 Camera HSV & Color Histogram Calibration (Arena Spotlight Adaptation)
1. Place standard competition colored balls (Red, Green, Blue) at grid intersection distance ($d \approx 40\text{ cm}$).
2. Run the `calibrate_hsv.py` utility script on the Raspberry Pi 4B:
   - Lock camera exposure (`EXPOSURE_OFFLINE = True`) and Auto-White Balance (`AWB_MODE = 'off'`).
   - Adjust Hue/Saturation upper and lower bounds in `MasterMissionController.py` until each ball achieves $>95\%$ mask coverage without floor reflections.

---

## 3. Subsystem Integration Testing Checklists

### 3.1 Test 1: Benchtop Leg IK & Trot/Crawl Gait Verification
- **Purpose:** Ensure mathematical IK solvers never produce unreachable servo angles or jitter.
- **Procedure:**
  1. Place the robot chassis on an elevated assembly block so all four feet hang freely.
  2. Send `CMD_SET_VELOCITY(vx=100, vy=0, omega=0, gait=1: TROT)` via USB-UART terminal.
  3. Verify diagonal leg pairs (FL+BR, FR+BL) swing in phase with smooth Bezier arcs.
  4. Measure total bus current on the 5V/10A UBEC: must remain **$< 3.5\text{A}$ unloaded**.
- **Pass Criteria:** Zero mechanical collisions, zero PCA9685 brownouts, clean symmetrical 50 Hz footsteps.

---

### 3.2 Test 2: Dual-Board UART CRC8 Loopback & Packet Jitter Test
- **Purpose:** Validate 115200 baud binary IPC communication between Pi 4B and STM32.
- **Procedure:**
  1. Execute a 10,000-packet stress test script on Pi 4B sending random `CMD_SET_VELOCITY` frames.
  2. STM32 echoes back CRC8 verification and telemetry stream `TEL_SENSOR_STREAM`.
- **Pass Criteria:** **$< 0.01\%$ CRC error rate**; average round-trip latency **$< 4.0\text{ ms}$**.

---

### 3.3 Test 3: Wall Corridor & Gap Rejection Simulation
- **Purpose:** Verify Task #02 and Task #03 corridor centering and gap rejection reflex.
- **Procedure:**
  1. Build a 400 mm wide mock corridor using cardboard sheets.
  2. Create a 300 mm gap in the left wall partway through the track.
  3. Command the robot to enter at $v_x = 120\text{ mm/s}$.
- **Pass Criteria:**
  - Robot maintains $e_{\text{corridor}} < 25\text{ mm}$ within solid wall boundaries.
  - When passing the gap, the outlier rejection filter triggers: robot **does not turn into the gap** and clears the 300 mm opening smoothly.

---

### 3.4 Test 4: End-to-End Full Track Competition Dry-Run

| Step # | Competition Task Checkpoint | Expected Behavior | Pass / Fail Sign-off |
| :---: | :--- | :--- | :---: |
| **1** | `STATE_0`: Start Box to Grid | Starts within 200 ms of button push; follows guideline into grid. | `[ ] PASS` |
| **2** | `STATE_1`: Grid Search & Pick (Task #01) | Stops at cross; camera detects ball; Gripper grabs and drops ball into ventral cage; stores color in memory. | `[ ] PASS` |
| **3** | `STATE_2`: Corridor Wall Following (Task #02) | Centers between walls; rejects wall gap on left/right; clears corridor. | `[ ] PASS` |
| **4** | `STATE_3`: Obstacle Pushing (Task #03) | Detects block at $d < 150\text{ mm}$; lowers front bumper; switches to Crawl gait; pushes block 300+ mm forward. | `[ ] PASS` |
| **5** | `STATE_4`: Color Sorting Delivery (Task #04) | Matches `stored_ball_color`; navigates correct branch (Red/Green/Blue); opens belly cage to release ball. | `[ ] PASS` |
| **6** | `STATE_5`: Finish Line Run | Resumes guideline to End box; halts cleanly; LED pulses Blue. | `[ ] PASS` |

---

## 4. Competition Day Fail-Safe & Battery Protocols

1. **Minimum Voltage Cut-Off:** If telemetry reports `battery_mv < 10800 mV` (3.6V per cell), immediately trigger a flashing Red status LED and disable servo PWM outputs to prevent LiPo cell damage or brownout resets.
2. **Emergency Stop (E-Stop) Button:** Ensure the top-deck hardware switch breaks `GPIO 23` on the Pi and `EXTI0` on the STM32 simultaneously, driving all PCA9685 PWM outputs to 0 µs within 2 ms.
