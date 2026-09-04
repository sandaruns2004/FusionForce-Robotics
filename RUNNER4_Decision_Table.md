# RUNNER-4 — Final Engineering Decision Table

| # | Design Area | Final Decision | Reason | Backup Plan |
|---|------------|----------------|--------|-------------|
| 1 | **Robot Type** | 4-leg quadruped | Minimum legs for static stability; fewer servos than hexapod; fits 250mm footprint | — (only viable legged option) |
| 2 | **DOF** | 12-DOF (3 per leg) | Full 3D foot control; lateral movement; body attitude adjustment; stride width control for pushing | — |
| 3 | **Leg Servos** | 12× MG90S (metal gear) | 2.2 kg·cm @ 6V; 13.4g each; lightest option that avoids weight spiral; 1.4× margin for crawl gait | Upgrade femur/tibia to MG996R (10 kg·cm, 55g); hybrid config |
| 4 | **Mechanism Servos** | 3× MG90S (arm, gripper, gate) | Same model simplifies spares; adequate torque for ball and gate | — |
| 5 | **Servo Driver** | 1× PCA9685 (16-ch) | 15/16 channels used; single board sufficient; adding second board is wasteful | — |
| 6 | **MCU** | **STM32F411CEU6 Black Pill** | 100 MHz Cortex-M4F; hardware FPU; 512KB Flash (needed for IK + gait + state machine + all drivers); 128KB RAM; same Black Pill pinout as F401 — drop-in replacement | STM32F401 (256KB Flash — borderline tight) |
| 7 | **SBC (Vision)** | **None — removed** | All perception + decision moved to STM32 embedded sensors; Pi adds 46g, 1.5A, 30-second boot delay, UART failure point, and Linux complexity with no advantage for this sensor set | — |
| 8 | **Line Sensor** | **8-ch TCRT5000 IR Array (GPIO)** | Digital GPIO read; <0.1ms latency vs 33ms camera frame; 8-sensor weighted centroid for steering; ≥6-sensor temporal filter for intersection; 200× faster than camera pipeline | Pololu QTRX-8A (analog, slightly higher cost) |
| 8b | **Colour Sensor** | **TCS34725 RGBC (I2C1, arm tip)** | Single sensor in two modes: arm 0° reads ball colour at pedestal; arm −70° reads floor zone at junction; built-in LED for consistent illumination; deterministic R/G/B ratio algorithm | Second TCS34725 at underside for simultaneous reads |
| 9 | **IMU** | MPU6050 | 6-axis; cheap; well-documented; adequate for pitch/roll stabilization; I2C | BNO055 (better fusion, more expensive) |
| 10 | **Distance Sensors** | 3× VL53L0X ToF | 30–1200mm range; 25° FoV; I2C; XSHUT address remapping (no mux needed) | Add TCA9548A mux if XSHUT unreliable |
| 11 | **Battery** | 2S LiPo (7.4V, 1300–2200mAh, 25C+) | Lighter than 3S (saves ~30g); sufficient for 5V BEC; ~24min runtime; ≤24V compliant | 3S LiPo if BEC dropout is problematic at low charge |
| 12 | **Servo BEC** | 5V–6V / 15A switching regulator | Handles peak servo current; efficient step-down from 7.4V; dedicated servo rail | Two smaller BECs if single unit overheats |
| 13 | **Ball Mechanism** | 2-servo arm + gripper | Ball on 5cm pedestal requires active reach; scoop/funnel cannot reach; 2 servos minimum for grab+lift | Modified scoop at pedestal height (knock ball off pedestal) |
| 14 | **Ball Storage** | Internal compartment + servo gate | Enclosed; ball cannot fall out during any manoeuvre; gravity-assisted release | Gripper continuous hold (risky — servo heating) |
| 15 | **Ball Release** | Bottom trapdoor (same gate servo) | Simplest; no additional actuator; gravity-based; same servo as storage lock | Arm re-grab and place (complex but more controlled) |
| 16 | **Pushing Mechanism** | Passive front bumper plate | Zero extra actuators; zero failure modes; uses existing leg force; PETG 50%+ infill | Servo-deployed bumper (adds 1 servo) |
| 17 | **Primary Gait** | Crawl (exclusively) | 3 feet on ground at all times; within MG90S torque limits; static stability guaranteed | — (trot exceeds servo capability) |
| 18 | **Perception Approach** | **8-sensor weighted centroid PD + TCS34725 R/G/B ratio** | Deterministic; sub-millisecond line reads; stable colour classification under controlled LED lighting; no training data; works without Linux/camera | Add TFLite on second MCU if colour classification fails |
| 19 | **Communication** | **I2C internal only (no inter-CPU UART)** | No Raspberry Pi to communicate with; all buses internal to STM32; debug UART (PA9/PA10, 115200 baud) used during development only — disconnected at competition | — |
| 20 | **Body Material** | 3D-printed PETG | Good strength-to-weight; heat resistant; affordable; easy to iterate | PLA+ (lighter, adequate strength for competition) |
