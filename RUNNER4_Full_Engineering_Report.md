# RUNNER-4 Autonomous Quadruped Robot
## Project Proposal and System Design Report
### FusionForce Robotics — EN2533 Robotic Design and Competition
### Competition Date: 5 September 2026

---

# 1. Executive Summary

The RUNNER-4 project delivers a fully autonomous 12-DOF quadruped robot designed to complete four sequential subtasks in the EN2533 BREACH PROTOCOL competition: grid navigation with ball pickup, curved wall-following through corridors with gaps, obstacle pushing, and colour-based ball sorting. The robot uses 12 × MG90S metal-gear micro servos driven by a single PCA9685 PWM controller, with an STM32F401CCU6 microcontroller handling real-time motion control, inverse kinematics, gait generation, and sensor fusion. A Raspberry Pi processes camera-based vision for line following, ball detection, colour recognition, and junction identification, replacing traditional IR and colour sensor arrays with a single CSI camera running OpenCV pipelines. Three VL53L0X time-of-flight sensors provide wall distance measurements for corridor navigation, and an MPU6050 IMU enables body stabilization. The robot operates from a 2S LiPo battery with separated power rails for servo and logic subsystems, maintaining a footprint under 250 × 250 mm. The design philosophy prioritises reliability, mechanical simplicity, and rule compliance over complexity or speed.

---

# 2. Competition Requirements

| Requirement | Specification | Compliance Strategy |
|---|---|---|
| **Autonomy** | Fully autonomous, no external input | Onboard processing only; no wireless modules |
| **Locomotion** | Active-legged robot; active wheels prohibited | 4-leg quadruped with 12 active servos |
| **Passive wheels** | Allowed | Not used in current design |
| **Robot integrity** | Must remain single entity; cannot split or leave parts | Monolithic chassis; all parts secured |
| **Starting** | Single onboard switch | Tactile push-button on STM32 PA0 |
| **Footprint** | ≤ 250 mm × 250 mm | Target body: ~190 × 170 mm with legs folded |
| **Height** | No restriction | Estimated ~150 mm standing |
| **Expansion** | Allowed if arena not damaged | Legs extend beyond footprint during walking |
| **Electronics** | Pre-made MCU/sensor kits allowed | STM32 dev board + commercial sensor breakouts |
| **Prohibited** | Wireless modules, LEGO/off-the-shelf robot kits | None used |
| **Power** | Onboard; max 24V DC | 2S LiPo = 8.4V max (compliant) |
| **Communication** | No external wireless/wired during operation | No Wi-Fi, Bluetooth, or tethering |
| **Arena lines** | 30 mm wide | Camera-based detection |
| **Arena surface** | Non-reflective matte | Simplifies vision thresholding |
| **Arena walls** | 20 cm high, white | ToF sensors detect at wall height |
| **Time limit** | 15 minutes maximum | Target completion: under 10 minutes |
| **Restarts** | Up to 3 within 10 minutes; timer continues | Robust state recovery design |
| **Pre-run** | 2 minutes for hardware calibration; no code changes | Quick-calibration procedure defined |

---

# 3. Problem Definition

The robot must autonomously complete four sequential tasks:

1. **Subtask 1 — Grid + Ball**: Navigate a 4×4 grid (25 cm cells, 3 cm lines), locate a coloured ball (~2 cm radius on a 5 cm pedestal), identify its colour (Red/Green/Blue), grasp it, store it internally, and exit.
2. **Subtask 2 — Curved Wall Following**: Traverse a 30 cm-wide corridor with non-straight walls containing gaps on either side without touching walls or exiting.
3. **Subtask 3 — Pushing Obstacle**: Enter a 30 cm corridor, detect a 25×25×20 cm passive obstacle, push it forward to clear the path, execute a left turn, and exit.
4. **Subtask 4 — Colour Sorting**: Follow a white line to a three-way junction (Red/Blue/Green branches), recall the stored ball colour, navigate the matching branch, release the ball, return to the main line, and finish.

The fundamental engineering challenge is building a legged robot that is mechanically stable enough to walk reliably, manipulate a small ball, push a heavy obstacle, and complete camera-based navigation — all within a 250 × 250 mm footprint and 15-minute time limit.

---

# 4. Design Objectives

Ranked by priority:

1. **Reliability** — Complete all four subtasks without mechanical or software failure
2. **Stability** — Maintain balance during walking, turning, pushing, and ball manipulation
3. **Rule compliance** — Meet all dimensional, electrical, and operational constraints
4. **Repeatability** — Produce consistent results across multiple runs
5. **Mechanical simplicity** — Minimise actuator count and mechanism complexity
6. **Power efficiency** — Maximise runtime from a compact battery
7. **Speed** — Complete tasks within time limit (not a speed competition)

---

# 5. Overall Strategy

**Key strategic decisions:**
- Use a **crawl gait** (one leg swinging at a time) as the primary locomotion mode for maximum static stability.
- Use **camera vision** as the primary perception system, replacing multiple discrete sensors.
- Use **ToF sensors** for wall-following tasks where camera view is obstructed.
- Use **passive front bumper** for pushing, leveraging the quadruped's leg force.
- Use a **simple servo-gated compartment** for ball storage.

---

# 6. System Architecture

```
    ┌─────────────────────────────────────────────────────────┐
    │                    RASPBERRY PI 4B                       │
    │  Camera → OpenCV Pipeline → Mission State Machine       │
    │  (Line/Ball/Colour/Junction detection)                  │
    └──────────────────────────┬──────────────────────────────┘
                               │ UART (115200 baud, CRC8)
    ┌──────────────────────────┴──────────────────────────────┐
    │                    STM32F401CCU6                         │
    │  IMU + ToF → Motion Controller (IK + Gait + Stabilize) │
    │  → PCA9685 → 15× MG90S Servos                          │
    └─────────────────────────────────────────────────────────┘

    Power: 2S LiPo → Switch → Fuse → 5V/15A BEC (Servos)
                                    → 5V/3A Reg  (Pi)
                                    → STM32 → 3.3V LDO (Sensors)
```

---

# 7. Mechanical Design

## 7.1 Robot Configuration

The robot is a **4-legged quadruped** with **12 degrees of freedom** (3 DOF per leg).

- **Quadruped vs. hexapod**: Fewer servos (12 vs 18), lower weight, simpler control. The 250 mm footprint constraint makes fitting 6 legs difficult.
- **3-DOF per leg**: Provides full (x, y, z) foot positioning, enabling forward/backward walking, lateral movement, turning, and body height/attitude adjustment.

| Joint | Axis | Function | Range |
|-------|------|----------|-------|
| **Coxa** (Hip Yaw) | Vertical Z | Swings leg forward/backward and laterally | ±45° |
| **Femur** (Hip Pitch) | Horizontal Y | Lifts/lowers the thigh segment | ±60° |
| **Tibia** (Knee Pitch) | Horizontal Y | Extends/retracts the lower leg | 0° to 135° |

## 7.2 12-DOF Leg Design

### Link Lengths (Proposed — require physical validation)

| Link | Symbol | Proposed Length | Rationale |
|------|--------|----------------|-----------|
| **Coxa offset** | L_coxa | 25 mm | Minimum clearance to prevent leg-body collision |
| **Femur** | L_femur | 50 mm | Adequate stride within MG90S torque capability |
| **Tibia** | L_tibia | 55 mm | Reaches ground at comfortable ~70 mm standing height |

**Maximum leg reach**: L_coxa + L_femur + L_tibia = 130 mm horizontal from hip.

**Footprint compliance**: Design the body at **~160 × 140 mm**. Verify that the robot stands independently within 250 × 250 mm with legs in a neutral starting stance. During walking, legs will extend beyond this footprint, which is permitted ("Expansion during the run is allowed").

### Foot Design
- Small rounded tip with rubber O-ring or silicone for traction on matte arena surface
- Foot diameter: ~8–10 mm contact patch

## 7.3 Servo Selection — MG90S Torque Feasibility

### MG90S Specifications

| Parameter | Value |
|-----------|-------|
| Stall torque | 1.8 kg·cm @ 4.8V / **2.2 kg·cm @ 6.0V** |
| Speed | 0.10 sec/60° @ 4.8V / 0.08 sec/60° @ 6.0V |
| Weight | ~13.4 g |
| Stall current | ~700 mA @ 6.0V |
| Operating current | ~200 mA (typical, loaded) |

### Estimated Robot Mass

| Component | Estimated Mass (g) |
|-----------|-------------------|
| 15 × MG90S servos | 201 |
| STM32 + PCA9685 | 18 |
| Raspberry Pi 4B (2GB) | 46 |
| Camera module | 5 |
| MPU6050 + 3× VL53L0X | 9 |
| 2S LiPo 1300mAh | 75 |
| BEC + regulators | 23 |
| 3D printed body + legs | 120 |
| Ball mechanism + bumper | 35 |
| Wiring + fasteners | 45 |
| **TOTAL ESTIMATED** | **~577 g** |

### Torque Requirements

| Condition | Required (kg·cm) | MG90S @ 6V (kg·cm) | Margin |
|-----------|------------------|-----------------------|--------|
| Static standing (4 legs) | 0.58 | 2.2 | **3.8×** ✅ |
| **Crawl gait (3 legs stance)** | 1.53 | 2.2 | **1.4×** ⚠️ |
| Pushing (4 legs, low stance) | ~2.0 | 2.2 | **1.1×** ⚠️ |
| Trot gait (2 legs) | 2.3 | 2.2 | **0.96×** ❌ |

**Assessment**: MG90S is **marginally sufficient** for crawl gait and pushing, but **insufficient for trot gait**. The design must:
1. Use **crawl gait exclusively** — never trot
2. Keep total robot mass **under 550 g** — every gram matters
3. Operate servos at **6.0V** for maximum torque
4. During pushing, **lower the body** and widen stance

**Backup plan**: If MG90S proves insufficient after prototype testing, upgrade femur and tibia joints to **MG996R** (10 kg·cm, 55g each) while keeping MG90S for coxa joints — a hybrid approach adding ~340g but providing 5× torque margin.

## 7.4 Body Design

**Multi-deck construction:**
- **Top deck**: Raspberry Pi + camera mount (~15 mm)
- **Mid deck**: STM32 + PCA9685 + BEC (~12 mm)
- **Bottom deck**: 2S LiPo battery + ball storage (~13 mm)
- **Material**: PETG or PLA+, 3D printed, 25–30% infill

## 7.5 Centre of Mass

**Principle**: Battery (heaviest component) at bottom-centre. Electronics centred. Camera at front-top (light, needs view). Ball storage near centre.

When the ball (~30g) is picked up and stored at front-centre, the CoM shifts forward by approximately **+3 mm** — well within the support polygon margin. During pickup, the robot should lower its body and widen stance for maximum stability.

## 7.6 Ball Collection Mechanism

### Options Analysis

| Option | Can Reach 5cm Pedestal? | Servos | Weight | Reliability | Selected? |
|--------|------------------------|--------|--------|-------------|-----------|
| A: Servo arm + gripper | ✅ Yes | 2 | 30g | Medium | ✅ **PRIMARY** |
| B: Front scoop + trapdoor | ❌ No (ball on pedestal) | 1 | 15g | High | ❌ |
| C: 3-DOF arm + gripper | ✅ Yes | 3 | 45g | Lower | ❌ Over-complex |
| D: Roller intake | ❌ No (can't reach pedestal) | 1 motor | 25g | Medium | ❌ |
| E: Passive funnel + gate | ❌ No (can't reach pedestal) | 1 | 12g | Medium | ❌ |

**Decision: Option A — 2-Servo Arm + Gripper**

The ball sits on a **5 cm pedestal**. Only options with an active arm can reach it. Option A uses:
- 1× MG90S for arm pitch (raise/lower to pedestal height)
- 1× MG90S for gripper (open/close around 40 mm ball)
- Arm pivot at front of body, ~40–50 mm above ground
- Gripper opening: >40 mm

**Backup**: Modified scoop at pedestal height — use robot body motion to knock ball off pedestal into a funnel.

## 7.7 Ball Storage

**Recommended: Internal compartment with servo gate**

- Location: Bottom-centre of body, behind the arm
- Size: ~50 mm diameter, ~50 mm deep (40 mm ball + clearance)
- Gate: Servo-operated trapdoor at bottom
- Arm grabs ball → lifts over body → drops into top opening → gate locks ball inside
- Ball cannot escape during walking, turning, pushing

## 7.8 Ball Release

**Recommended: Bottom trapdoor (servo gate)**

- Open the compartment gate → ball drops by gravity
- Uses the **same servo** that locks the gate during storage
- Simplest and most reliable — no additional actuator needed
- Gate servo on PCA9685 channel CH14

## 7.9 Pushing Mechanism

**Recommended: Passive front bumper plate**

- 3D-printed flat plate at front-bottom of body
- Width: ~80–100 mm (wider than robot front for good contact)
- Height: ground to ~50 mm
- No moving parts, no servos, no failure modes
- PETG with 50%+ infill for rigidity

**Pushing strategy:**
1. Detect obstacle (front ToF < 150 mm)
2. Lower body height to ~45 mm
3. Widen leg stance
4. Crawl gait at 50 mm/s — push with front bumper
5. Monitor IMU pitch — if >15°, reduce speed
6. When front ToF > 300 mm or push duration > 8 sec → obstacle cleared
7. Execute left turn → resume navigation

---

# 8. Electronics Architecture

### I2C Bus Architecture

**I2C1 (PB6/PB7, 400 kHz):** PCA9685 (0x40) + MPU6050 (0x68)

**I2C2 (PB10/PB3, 400 kHz):** 3× VL53L0X via **XSHUT sequential init** (GPIO PA4/PA5/PA6) — reprogrammed to addresses 0x29, 0x30, 0x31. No TCA9548A mux needed.

### PCA9685 Channel Allocation

| CH | Assignment | CH | Assignment |
|----|-----------|-----|-----------|
| 0 | FL Coxa | 8 | BL Tibia |
| 1 | FL Femur | 9 | BR Coxa |
| 2 | FL Tibia | 10 | BR Femur |
| 3 | FR Coxa | 11 | BR Tibia |
| 4 | FR Femur | 12 | Ball Arm Pitch |
| 5 | FR Tibia | 13 | Gripper Open/Close |
| 6 | BL Coxa | 14 | Compartment Gate |
| 7 | BL Femur | 15 | *Spare* |

**One PCA9685 is sufficient** — 15/16 channels used.

---

# 9. Actuators

| Actuator | Qty | Purpose | Torque @ 6V |
|----------|-----|---------|-------------|
| MG90S (leg) | 12 | Quadruped locomotion | 2.2 kg·cm |
| MG90S (arm) | 1 | Ball arm pitch | 2.2 kg·cm |
| MG90S (gripper) | 1 | Ball grasp/release | 2.2 kg·cm |
| MG90S (gate) | 1 | Storage compartment gate | 2.2 kg·cm |
| **Total** | **15** | | |

---

# 10. Sensors

| Sensor | Qty | Interface | Purpose |
|--------|-----|-----------|---------|
| **MPU6050** | 1 | I2C (0x68) | Pitch/roll, stabilization, tilt detection |
| **VL53L0X** | 3 | I2C (XSHUT) | Wall distance, obstacle detection |
| **CSI Camera** | 1 | CSI-2 ribbon | Line/ball/colour/junction detection |
| **Start Button** | 1 | GPIO PA0 | Competition start |

**IMU Placement**: Centre of body, mid-deck, at centre of mass — minimises false angular readings from translational acceleration.

**ToF Configuration**: Front (0°), Left (90°), Right (90°). Mounted ~25 mm above ground. 25° FoV cone. Median filter (window 5) for noise rejection.

---

# 11. Raspberry Pi and Vision System

## Pi Selection — Comparison

| Feature | Pi Zero 2 W | **Pi 4B (2GB)** | Pi 5 |
|---------|------------|-----------------|------|
| CPU | 4× A53 @ 1.0 GHz | **4× A72 @ 1.5 GHz** | 4× A76 @ 2.4 GHz |
| RAM | 512 MB | **2 GB** | 4 GB |
| OpenCV FPS (640×480) | 8–12 | **25–35** | 50–70 |
| Power | ~1.0A | **~2.5A** | ~3.5A |
| Weight | 9g | **46g** | 47g |
| UART | Shared with BT | **Dedicated** | Dedicated |

**Decision: Raspberry Pi 4 Model B (2GB)**

Pi Zero 2 W is too slow (8–12 FPS for full pipeline). Pi 5 is overpowered with heat issues. Pi 4B provides 25+ FPS with comfortable headroom.

Wi-Fi and Bluetooth must be **disabled at OS level** before competition.

## Camera Mounting

- **Position**: Front-top, +65 mm forward from centre, 50 mm above ground
- **Tilt**: −35° to −45° downward
- **Ground coverage**: Line at ~100–250 mm ahead of body (adequate for line following and ball detection)
- **Adjustable bracket**: Slotted mount for tilt adjustment
- **Vibration isolation**: Rubber grommets between bracket and body

---

# 12. STM32 Control System

### STM32F401CCU6 Key Specs
- ARM Cortex-M4 @ 84 MHz, 256 KB Flash, 64 KB RAM
- 3× I2C, 3× UART, 12-bit ADC

### Real-Time Loop (50 Hz / 20 ms)
1. Read IMU → compute pitch/roll (complementary filter)
2. Read 3× ToF → median filter
3. Parse UART commands from Pi
4. Read battery ADC
5. Update gait phase → 4 foot targets
6. Apply stabilization corrections
7. Solve IK (all 4 legs) → 12 joint angles
8. Apply servo trim offsets
9. Write PWM to PCA9685
10. Send telemetry to Pi
11. Check safety (tilt, battery, watchdog)

---

# 13. Power System

**Battery: 2S LiPo, 1300–2200 mAh, 25C+** (7.4V nominal, 8.4V full, ≤24V compliant)

### Power Distribution

```
2S LiPo → Toggle Switch → 10A Fuse → ┬→ 5V/15A BEC → PCA9685 V+ → Servos
                                        │   (+ 4700µF cap across V+/GND)
                                        ├→ 5V/3A Buck → Pi 4B (GPIO 5V)
                                        └→ Voltage divider → STM32 ADC (PA7)
                                            └→ STM32 VBUS → 3.3V LDO → Sensors
```

### Power Budget

| Component | Typical (mA) | Peak (mA) |
|-----------|-------------|-----------|
| 12 MG90S legs | 2400 | 8400 |
| 3 MG90S mechanism | 300 | 2100 |
| Raspberry Pi 4B | 1500 | 2500 |
| STM32 + sensors + PCA9685 | 120 | 150 |
| **TOTAL** | **~4320** | **~13150** |

Realistic peak during walking: **4–6A** (not all servos stall simultaneously).

**Runtime**: 1300mAh / ~3.2A (at 7.4V) ≈ **24 minutes** — sufficient for 15-min competition.

---

# 14. Communication Architecture

### Pi ↔ STM32 UART Protocol (115200 baud, CRC8)

**Commands (Pi → STM32):**

| Command | ID | Payload |
|---------|----|---------|
| SET_VELOCITY | 0x01 | vx, vy, omega, gait_type |
| ARM_ACTION | 0x02 | action_id (HOME/GRAB/STORE/RELEASE/PUSH) |
| BODY_ATTITUDE | 0x03 | pitch, roll, height |
| ESTOP | 0x04 | (none) |

**Telemetry (STM32 → Pi, 50 Hz):**

| Telemetry | ID | Payload |
|-----------|----|---------|
| SENSOR_STREAM | 0x81 | tof_L, tof_R, tof_F, pitch, roll, batt_mV, status |

**No wireless communication during operation.**

---

# 15. Software Architecture

**Raspberry Pi**: Camera capture → OpenCV pipeline (HSV threshold, contours, centroid) → Mission state machine (HFSM) → UART commands to STM32

**STM32**: UART parser → Motion controller (gait generator + IK solver + IMU stabilization) → PCA9685 PWM → Servos. Sensor acquisition (IMU + ToF + battery) → Telemetry transmission.

---

# 16. Inverse Kinematics

### 3-DOF Leg IK

Given foot position (x, y, z) relative to hip frame:

**Step 1 — Coxa angle**: θ₁ = atan2(y, x)

**Step 2 — Knee angle**: Using law of cosines on triangle (L_femur, L_tibia, D):
- R = √(x²+y²) − L_coxa
- D = √(R²+z²)
- θ₃ = π − acos((L_femur² + L_tibia² − D²) / (2·L_femur·L_tibia))

**Step 3 — Hip pitch**: θ₂ = atan2(−z, R) − acos((L_femur² + D² − L_tibia²) / (2·L_femur·D))

Link lengths: L_coxa=25mm, L_femur=50mm, L_tibia=55mm (proposed).

---

# 17. Gait Generation

### Crawl Gait (Primary — Only)

| Parameter | Value |
|-----------|-------|
| Cycle time | 2.0 sec (full 4-leg cycle) |
| Swing time/leg | 0.25 sec |
| Stance duty | 75% (3 legs always in support) |
| Step height | 20–25 mm |
| Step length | 40 mm |
| Body height | 65–70 mm |
| Speed | ~80 mm/s |

**Sequence**: FL → BR → FR → BL (diagonal alternation).

**Foot trajectory**: Quadratic Bezier for smooth lift/land:
- z(t) = z_stand + 4 × H_step × t × (1−t)

**Trot gait is NOT used** — exceeds MG90S torque capability.

---

# 18. IMU Stabilization

**Complementary filter**: angle = 0.98 × (angle + gyro × dt) + 0.02 × accel_angle

**Pitch/Roll correction**: Apply proportional correction to foot Z positions:
- Front feet: z += Kp × pitch_error
- Rear feet: z −= Kp × pitch_error
- Left feet: z += Kp × roll_error
- Right feet: z −= Kp × roll_error

Start with Kp = 0.5 mm/degree.

---

# 19. Line Following

Camera → ROI (bottom 30%) → Grayscale → Threshold → Contours → Centroid → Heading error → PD controller → omega command to STM32

**Special cases**: Intersections (wide contour), missing line (hold last command), curves (PD tracks smoothly).

---

# 20. Ball Detection

At grid intersections: Full frame → HSV → Colour masks (R/G/B) → Contours → Filter by area (>500px) + circularity (>0.6) → Classify colour → Store `ball_color`.

---

# 21. Colour Detection

HSV thresholds: Red (H=0–10 ∪ 165–179), Green (H=40–85), Blue (H=95–135). All with S>100, V>70. Require 2× pixel dominance for confident classification.

---

# 22. Wall Following

Dual-ToF PD centering: error = d_left − d_right.

**Gap rejection**: If one sensor reads >400mm while the other <250mm → switch to single-wall following. Require 3+ consecutive gap readings before switching (temporal filter).

---

# 23. Obstacle Detection and Pushing

Front ToF < 150mm → stop → lower body to 45mm → widen stance → crawl at 50mm/s → push → monitor IMU pitch (stop if >15°) → front ToF > 300mm → cleared → left turn → resume.

---

# 24. Complete Task State Machine

```
START → CALIBRATE → LINE_FOLLOW(to grid) → GRID_SEARCH → BALL_APPROACH →
BALL_GRAB → COLOUR_ID → BALL_STORE → GRID_EXIT → CORRIDOR_1_WALL_FOLLOW →
CORRIDOR_2_WALL_FOLLOW → OBSTACLE_DETECT → PUSH_MODE → LEFT_TURN →
LINE_FOLLOW(to junction) → COLOUR_SORT → BALL_RELEASE → RETURN_LINE →
LINE_FOLLOW(to finish) → FINISH

Error states: SAFE_STOP (excessive tilt/low battery), ERROR_RECOVERY
```

---

# 25. Subtask-by-Subtask Strategy

| Subtask | Sensors | Algorithm | Speed | Key Risk |
|---------|---------|-----------|-------|----------|
| 1: Grid+Ball | Camera | Line follow + ball detect + grab | 60 mm/s | Ball alignment |
| 2: Curved Walls | 3× ToF, IMU | Dual-wall PD + gap rejection | 80 mm/s | False gap detect |
| 3: Push | Front ToF, IMU | Low stance + crawl push | 50 mm/s | Robot tipping |
| 4: Colour Sort | Camera | Line follow + junction detect + release | 60 mm/s | Wrong branch |

---

# 26. Failure Handling

| Failure | Detection | Response |
|---------|-----------|----------|
| Servo stall | No IMU motion | Reduce load; retry; SAFE_STOP if persistent |
| Camera fail | No frames | Halt; re-init; ToF-only degraded mode |
| Pi timeout | No UART for 500ms | STM32 → autonomous stop |
| Low battery | ADC < 6.8V | Flash LED; reduce speed; stop at 6.4V |
| Excessive tilt | IMU > 30° | Emergency stop all servos |
| Ball pickup fail | >10 sec timeout | Retry once; skip if still fails |
| Line lost | No centroid for 3s | Stop; rotate to search; hold last heading |

---

# 27. Sensor and Component Placement

| Component | Location | Height | Rationale |
|-----------|----------|--------|-----------|
| Battery | Bottom-centre | 5mm | Lowest CoM |
| STM32+PCA9685 | Mid-deck, centre | 25mm | Short servo wiring |
| Raspberry Pi 4B | Top-deck, centre-rear | 45mm | CSI/USB access |
| Camera | Front-top, tilted −40° | 50mm | Forward/downward view |
| IMU | Centre, mid-deck | 25mm | At CoM |
| ToF Front | Front-centre | 25mm | Obstacle detection |
| ToF Left/Right | Front corners | 25mm | Wall centering |
| Ball storage | Front-centre, bottom | 15mm | Near CoM |
| Arm+Gripper | Front of body | 40mm pivot | Reaches pedestal |
| Front bumper | Front-bottom | 0–50mm | Obstacle contact |

---

# 28. Weight and Power Budget

### Weight: ~577g estimated (target: <550g)
### Power: ~4.3A typical, ~6A realistic peak
### Runtime: ~24 min with 1300mAh (sufficient for 15-min competition)

---

# 29. Testing and Validation

| Stage | Test | Pass Criteria |
|-------|------|--------------|
| 1 | Single servo sweep | Full range, no jitter |
| 2 | One leg IK | Smooth foot positioning |
| 3 | One leg loaded | Supports 200g without stall |
| 4–5 | Standing (2 legs → 4 legs) | Stable, within footprint |
| 6–7 | Walking + turning | 1m forward; 90° turn |
| 8 | IMU stabilization | Level on ±5° surface |
| 9 | Wall following | Centre in 30cm corridor |
| 10 | Line following | Follow 2m curved line |
| 11 | Ball detect + colour | 95%+ accuracy |
| 12 | Ball pickup + store | Reliable grab from pedestal |
| 13 | Obstacle push | Push 30cm without tipping |
| 14 | Colour sorting | Correct branch selection |
| 15 | Full run | All 4 subtasks completed |
| 16 | Endurance | Full 15-min run |

---

# 30. Risk Analysis

| Risk | Prob. | Severity | Mitigation | Backup |
|------|-------|----------|-----------|--------|
| MG90S insufficient torque | High | Critical | Mass <550g; crawl only; 6V | Upgrade to MG996R hybrid |
| Servo overheating | Medium | High | Limit continuous load; rest between tasks | Duty-cycle servos |
| Servo brownout | Medium | High | 4700µF cap; separate rails; thick wires | Reduce simultaneous servo moves |
| Camera lighting | Medium | Medium | Lock exposure/WB | Adjust HSV in 2-min prep |
| ToF gap misread | Medium | High | 3-reading persistence filter | IMU heading hold |
| Ball pickup failure | Medium | High | Wide gripper; retry once | Skip ball (partial score) |
| Robot tipping (push) | Medium | High | Low stance; wide base; IMU monitor | Reduce push speed |
| Footprint violation | Medium | High | Measure with jig; 160×140 body | Adjust coxa starting angle |
| Pi crash | Low | Critical | STM32 watchdog → SAFE_STOP | Use restart allowance |
| 3D print failure | Medium | Medium | Print spares | Backup leg set |

---

# 31. Alternative Designs

| Decision Area | Option A | Option B | Option C | **Selected** |
|--------------|---------|---------|---------|-------------|
| Vision | Camera+OpenCV | IR sensor array | Colour sensors | **Camera** (replaces 3 sensor types) |
| Processing | Pi Zero 2 W (9g) | Pi 4B 2GB (46g) | Pi 5 (47g) | **Pi 4B** (best performance/weight) |
| Ball mechanism | Scoop | 2-servo arm+gripper | 3-DOF arm | **2-servo arm** (reaches pedestal) |
| Gait | Crawl | Trot | — | **Crawl** (within MG90S limits) |
| Servo control | 1× PCA9685 | 2× PCA9685 | — | **1× PCA9685** (15/16 channels) |
| Power | 2S LiPo | 3S LiPo | — | **2S LiPo** (lighter; adequate BEC margin) |

---

# 32. Final Recommended Design

| Area | Decision |
|------|----------|
| Robot type | 4-leg quadruped, 12-DOF |
| Servos | 15× MG90S (12 leg + 3 mechanism) |
| Servo driver | 1× PCA9685 |
| MCU | STM32F401CCU6 Black Pill |
| SBC | Raspberry Pi 4B (2GB) |
| Camera | Pi Camera Module (CSI) |
| IMU | MPU6050 |
| Distance | 3× VL53L0X (XSHUT addressing) |
| Battery | 2S LiPo, 1300–2200mAh |
| Servo BEC | 5V/15A switching regulator |
| Pi regulator | 5V/3A buck converter |
| Ball mechanism | 2-servo arm + gripper |
| Ball storage | Internal compartment + servo gate |
| Pushing | Passive front bumper |
| Gait | Crawl (only) |
| Vision | OpenCV HSV + contours |
| Communication | UART 115200 + CRC8 |

---

# 33. Task Delegation

| Team Area | Responsibilities |
|-----------|-----------------|
| **Mechanical** | Body CAD; leg design; 3D printing; ball mechanism; bumper |
| **Electronics** | Power distribution; BEC; wiring; PCA9685 setup |
| **STM32 Firmware** | IK; gait; IMU filter; ToF; UART; servo calibration |
| **Pi Software** | Camera pipeline; line/ball/colour detection; state machine |
| **Integration** | System testing; competition prep; calibration |

---

# 34. Development Roadmap

| Phase | Activities | Milestone |
|-------|-----------|-----------|
| 1: Design (Wk 1–2) | Finalise CAD; order parts | BOM ordered |
| 2: Mechanical (Wk 2–4) | 3D print; assemble; mount servos | Robot assembled |
| 3: Electronics (Wk 3–5) | Wire power; test BEC; connect PCA9685 | Power verified |
| 4: Firmware (Wk 4–7) | IK; single-leg test; gait; IMU | Walking robot |
| 5: Vision (Wk 5–8) | Camera pipeline; line following | Vision working |
| 6: Mechanisms (Wk 6–8) | Ball arm; gripper; gate; bumper | Ball pickup tested |
| 7: Integration (Wk 8–10) | Combine all; state machine | Full task completion |
| 8: Testing (Wk 10–12) | Endurance; tuning | Reliable runs |
| 9: Competition (Wk 12+) | Presentation; summary; spares | Competition ready |

---

# 35. Competition-Day Procedure

**Before**: Charge battery; verify Wi-Fi/BT disabled; pack spares.

**2-Min Prep**: Power on → check LED → verify footprint → adjust sensors → position on start line.

**Run**: Press start → hands off → restart if needed (up to 3).

---

# 36. Final Checklist

- [ ] Footprint ≤ 250×250mm at start
- [ ] Stands independently
- [ ] Single start switch works
- [ ] No wireless modules active
- [ ] Battery ≤ 24V DC
- [ ] All 15 servos respond
- [ ] Camera captures frames
- [ ] ToF sensors return valid distances
- [ ] Full 4-subtask run completed 3+ times
- [ ] Presentation prepared (10 min)
- [ ] 2-page summary printed

---

# 37. Conclusion

The RUNNER-4 design balances capability and simplicity. The highest risk is MG90S servo torque margin — the team must weigh every component, test a single leg under load before full assembly, and validate crawl gait early. If MG90S proves insufficient, the fallback to MG996R hybrid is well-defined.

---

# DECISIONS WE MUST MAKE BEFORE CAD/FINAL BUILD

| # | Decision | Impact |
|---|---------|--------|
| 1 | **Verify MG90S torque with physical prototype** | Determines entire servo strategy |
| 2 | **Confirm body dimensions** (160×140mm?) | Footprint compliance |
| 3 | **Select battery capacity** (1300 vs 2200mAh) | Weight vs runtime |
| 4 | **2S vs 3S LiPo** | Weight vs BEC headroom |
| 5 | **Ball arm geometry** (length, pivot height) | Must reach 5cm pedestal |
| 6 | **Camera module** (v2 vs v3) | Focus type; weight |
| 7 | **Foot material** (PETG vs rubber tip) | Arena traction |
| 8 | **XSHUT vs TCA9548A for ToF** | Weight vs complexity |
| 9 | **3D print material** (PLA+ vs PETG) | Weight vs strength |
| 10 | **Pi 4B vs Pi Zero 2 W** (validate FPS first) | Weight savings potential |
