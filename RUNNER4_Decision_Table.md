# RUNNER-4 — Final Engineering Decision Table

| # | Design Area | Final Decision | Reason | Backup Plan |
|---|------------|----------------|--------|-------------|
| 1 | **Robot Type** | 4-leg quadruped | Minimum legs for static stability; fewer servos than hexapod; fits 250mm footprint | — (only viable legged option) |
| 2 | **DOF** | 12-DOF (3 per leg) | Full 3D foot control; lateral movement; body attitude adjustment; stride width control for pushing | — |
| 3 | **Leg Servos** | 12× MG90S (metal gear) | 2.2 kg·cm @ 6V; 13.4g each; lightest option that avoids weight spiral; 1.4× margin for crawl gait | Upgrade femur/tibia to MG996R (10 kg·cm, 55g); hybrid config |
| 4 | **Mechanism Servos** | 3× MG90S (arm, gripper, gate) | Same model simplifies spares; adequate torque for ball and gate | — |
| 5 | **Servo Driver** | 1× PCA9685 (16-ch) | 15/16 channels used; single board sufficient; adding second board is wasteful | — |
| 6 | **MCU** | STM32F401CCU6 Black Pill | 84 MHz Cortex-M4F; hardware FPU for IK math; multiple I2C/UART; 256KB flash; adequate for 50Hz loop | — |
| 7 | **SBC (Vision)** | Raspberry Pi 4B (2GB) | 25–35 FPS OpenCV; dedicated UART; 2GB RAM; USB debug ports; Pi Zero too slow (8–12 FPS); Pi 5 overkill with heat issues | Pi Zero 2 W if FPS validation passes (saves 37g) |
| 8 | **Camera** | Pi Camera Module (CSI) | Native CSI interface; 640×480 @ 30+ FPS; fixed/auto-focus; replaces IR + colour sensors | USB webcam (heavier, more latency) |
| 9 | **IMU** | MPU6050 | 6-axis; cheap; well-documented; adequate for pitch/roll stabilization; I2C | BNO055 (better fusion, more expensive) |
| 10 | **Distance Sensors** | 3× VL53L0X ToF | 30–1200mm range; 25° FoV; I2C; XSHUT address remapping (no mux needed) | Add TCA9548A mux if XSHUT unreliable |
| 11 | **Battery** | 2S LiPo (7.4V, 1300–2200mAh, 25C+) | Lighter than 3S (saves ~30g); sufficient for 5V BEC; ~24min runtime; ≤24V compliant | 3S LiPo if BEC dropout is problematic at low charge |
| 12 | **Servo BEC** | 5V–6V / 15A switching regulator | Handles peak servo current; efficient step-down from 7.4V; dedicated servo rail | Two smaller BECs if single unit overheats |
| 13 | **Ball Mechanism** | 2-servo arm + gripper | Ball on 5cm pedestal requires active reach; scoop/funnel cannot reach; 2 servos minimum for grab+lift | Modified scoop at pedestal height (knock ball off pedestal) |
| 14 | **Ball Storage** | Internal compartment + servo gate | Enclosed; ball cannot fall out during any manoeuvre; gravity-assisted release | Gripper continuous hold (risky — servo heating) |
| 15 | **Ball Release** | Bottom trapdoor (same gate servo) | Simplest; no additional actuator; gravity-based; same servo as storage lock | Arm re-grab and place (complex but more controlled) |
| 16 | **Pushing Mechanism** | Passive front bumper plate | Zero extra actuators; zero failure modes; uses existing leg force; PETG 50%+ infill | Servo-deployed bumper (adds 1 servo) |
| 17 | **Primary Gait** | Crawl (exclusively) | 3 feet on ground at all times; within MG90S torque limits; static stability guaranteed | — (trot exceeds servo capability) |
| 18 | **Vision Approach** | OpenCV HSV + contours (no ML) | HSV segmentation sufficient for line/ball/colour/junction; deterministic; no training data needed; runs at 25+ FPS on Pi 4 | TFLite model if HSV fails under complex lighting |
| 19 | **Communication** | UART 115200 baud + CRC8 | Simple; reliable; no wireless; hardware UART on both Pi and STM32; CRC8 catches corruption | Increase to 921600 baud if 50Hz telemetry is bandwidth-limited |
| 20 | **Body Material** | 3D-printed PETG | Good strength-to-weight; heat resistant; affordable; easy to iterate | PLA+ (lighter, adequate strength for competition) |
