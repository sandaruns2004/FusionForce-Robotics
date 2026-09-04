# RUNNER-4 — 10-Minute Presentation Plan

## Time Budget: 10:00 (strict — do NOT exceed)

---

## Slide Plan & Speaker Assignments

> Adjust speaker names as needed. Assumes 5 team members (A–E). All must present.

| Slide # | Section | Duration | Speaker | Content |
|---------|---------|----------|---------|---------|
| 1 | **Title + Team** | 0:20 | A | Team name, competition name, date, team members |
| 2 | **Problem Statement** | 0:30 | A | 4 subtasks overview; 250mm footprint; 15-min time; fully autonomous |
| 3 | **Overall Strategy** | 0:40 | A | Crawl gait for stability; single STM32 handles all perception + decision; 8-ch IR array replaces camera for line following; TCS34725 on arm tip for colour detection; passive pushing |
| 4 | **System Architecture** | 0:30 | B | Block diagram: STM32F411 ↔ I2C1 (PCA9685+MPU6050+TCS34725) ↔ I2C2 (3×VL53L0X); GPIO PA0–PA7 (line array); power tree (BEC + 3.3V LDO) |
| 5 | **Actuators** | 0:40 | C | 15× MG90S; torque analysis (2.2 kg·cm vs 1.53 required); why crawl not trot; PCA9685 single board |
| 6 | **Sensors** | 0:50 | C | **8-ch TCRT5000 line array** (line/intersection/junction); **TCS34725 on arm tip** (ball colour + floor zone, dual mode); 3× VL53L0X ToF (wall/obstacle); MPU6050 (stabilization); sensor placement diagram |
| 7 | **Mechanical Design** | 1:00 | B | 12-DOF quadruped; leg geometry; link lengths; body dimensions; 3D printed PETG; footprint compliance |
| 8 | **Ball Mechanism** | 0:40 | B | 2-servo arm + gripper; internal compartment; servo gate; why scoop doesn't work (5cm pedestal) |
| 9 | **Pushing Mechanism** | 0:20 | B | Passive front bumper; low stance strategy |
| 10 | **Embedded Perception** | 0:50 | C | **Why no camera/Pi?** Boot delay, weight, failure point; TCRT5000 weighted centroid algorithm; TCS34725 R/G/B ratio classification; arm dual-mode (MODE A 0° ball / MODE B −70° floor); temporal filter for intersection detection |
| 11 | **STM32F411 + Control** | 0:30 | D | F411CEU6 (512KB Flash needed for state machine + all drivers); 50Hz control loop; IK solver; gait generator; IMU filter; Flash EEPROM for ball colour persistence |
| 12 | **Algorithms** | 1:00 | D | IK equations; crawl gait sequence; PD line following; PD wall following; gap rejection filter |
| 13 | **Subtask Walkthrough** | 1:00 | D | Step through all 4 subtasks: grid→ball→corridor→push→sort→finish; state machine flow |
| 14 | **Power System** | 0:20 | E | 2S LiPo; 5V/15A BEC (servos); 3.3V LDO (STM32+sensors); separate rails; **~37 min runtime** (Pi removed saves 1.5A) |
| 15 | **Task Delegation** | 0:30 | E | 5 team areas; who does what; development timeline |
| 16 | **Risks + Mitigation** | 0:30 | E | Top 5 risks: servo torque, TCS34725 ambient light, intersection false trigger, ball colour memory loss on restart (Flash persistence), loop timing overflow; backup plans |
| 17 | **Conclusion** | 0:20 | E | Design philosophy (reliability > speed); confidence in approach; ready for build |
| | **TOTAL** | **10:00** | | |

---

## Key Presentation Tips

1. **Practice the timing** — each speaker rehearses their section individually, then run full team rehearsal
2. **Transitions**: Each speaker ends with "Now [Name] will cover..." for smooth handoffs
3. **Visually rich slides**: Use the CAD screenshots, system architecture diagram, state machine diagram, and component photos
4. **Avoid reading slides** — speak to the evaluators, not the screen
5. **Anticipate Q&A** — see the Q&A preparation document

## Must-Show Visuals

- [ ] System architecture block diagram
- [ ] CAD render / photo of robot design
- [ ] Leg kinematic diagram with link lengths
- [ ] Ball mechanism concept sketch
- [ ] Sensor placement diagram (top view)
- [ ] Power tree schematic
- [ ] State machine flow diagram
- [ ] Torque feasibility table
- [ ] Pi comparison table

---

## Detailed Speaker Script & Talking Points

### Slide 1: Title + Team (Speaker A - 0:20)
- **Greeting**: Welcome the evaluators and introduce the team (FusionForce).
- **Introduction**: State the project name (RUNNER-4).
- **Team**: Briefly introduce each team member and their core role.

### Slide 2: Problem Statement (Speaker A - 0:30)
- **Goal**: Build a fully autonomous robot to complete a circuit of 4 distinct subtasks.
- **Constraints**: Emphasize the strict 250mm footprint limit and the 15-minute maximum time limit.
- **Tasks Overview**: Mention the 4 phases: Grid search & object retrieval (Task 01), Corridor with gap (Task 02), Corridor with obstacle (Task 03), and Color sorting drop-off (Task 04).

### Slide 3: Overall Strategy (Speaker A - 0:40)
- **Movement**: We chose a crawl gait to maximize stability over speed.
- **Compute**: A single STM32 microcontroller handles both perception and decision-making for a lightweight, unified system.
- **Vision vs. IR**: We opted for an 8-channel IR array over a camera for line following, reducing computational overhead and complexity.
- **Color Detection**: A TCS34725 color sensor is mounted directly on the arm tip for close-range color detection of both the ball and the drop-off zones.
- **Pushing**: We are using a passive pushing mechanism (bumper) to save weight and complexity.

### Slide 4: System Architecture (Speaker B - 0:30)
- **Diagram Focus**: Point to the system block diagram.
- **Microcontroller**: Explain the central role of the STM32F411.
- **Buses**: 
  - I2C1 handles actuators (PCA9685), IMU (MPU6050), and the color sensor.
  - I2C2 is dedicated to the three VL53L0X Time-of-Flight sensors for wall detection.
- **Direct GPIO**: PA0-PA7 reads the line array directly.
- **Power Tree**: Highlight the split power strategy (logic via 3.3V LDO, actuators via 5V/15A BEC).

### Slide 5: Actuators (Speaker C - 0:40)
- **Hardware**: Mention the 15x MG90S servos.
- **Feasibility**: Present the torque analysis (2.2 kg·cm max torque vs 1.53 kg·cm required per joint).
- **Gait Choice**: Explain why we use a crawl gait instead of a trot—to ensure servos are not overstressed and to maintain static stability.
- **Driver**: Driven by a single PCA9685 board to simplify wiring.

### Slide 6: Sensors (Speaker C - 0:50)
- **Line Following**: 8-channel TCRT5000 array for precise line, intersection, and junction detection.
- **Color Sensing**: Dual-purpose TCS34725 sensor on the arm tip (used for ball color in Task 01 and floor zone color in Task 04).
- **Obstacle/Wall Detection**: 3x VL53L0X ToF sensors for wall and gap detection (Tasks 02 and 03).
- **Stabilization**: MPU6050 IMU.
- **Visual**: Point to the sensor placement diagram.

### Slide 7: Mechanical Design (Speaker B - 1:00)
- **Chassis**: Discuss the 12-DOF quadruped design.
- **Geometry**: Explain the leg geometry and link lengths chosen for the required stride and ground clearance.
- **Compliance**: Show body dimensions, emphasizing strict compliance with the 250mm footprint rule.
- **Materials**: Highlight the use of 3D-printed PETG for a balance of durability and weight savings.

### Slide 8: Ball Mechanism (Speaker B - 0:40)
- **Design**: Describe the 2-servo arm and gripper design.
- **Storage**: Explain how the ball is secured in the internal compartment using a servo gate.
- **Design Choices**: Detail why a simple scoop was rejected (e.g., wouldn't work with a 5cm pedestal or uneven terrain).

### Slide 9: Pushing Mechanism (Speaker B - 0:20)
- **Passive Design**: Explain the passive front bumper design.
- **Strategy**: The robot adopts a "low stance" strategy during Task 03 to maximize traction and push the obstacle efficiently without needing an active plow.

### Slide 10: Embedded Perception (Speaker C - 0:50)
- **No Pi/Camera**: Explain the decision to avoid a Raspberry Pi/camera setup (eliminates boot delays, reduces weight, removes a point of failure, saves power).
- **Line Algorithms**: Describe the weighted centroid algorithm for the TCRT5000 array.
- **Color Algorithms**: Detail the R/G/B ratio classification and the two modes (Mode A: 0° for ball, Mode B: -70° for floor).
- **Filtering**: Mention the temporal filter used for robust intersection detection.

### Slide 11: STM32F411 + Control (Speaker D - 0:30)
- **MCU Choice**: Why the F411CEU6? The 512KB Flash provides enough space for complex state machines and all sensor drivers.
- **Timing**: 50Hz control loop maintains responsiveness.
- **Kinematics**: Explain the integration of the Inverse Kinematics (IK) solver and gait generator.
- **Persistence**: Mention the use of Flash EEPROM to persist the detected ball color in case of a restart.

### Slide 12: Algorithms (Speaker D - 1:00)
- **IK**: Briefly touch on the IK equations enabling precise leg positioning.
- **Gait**: Explain the crawl gait sequence.
- **Control**: Detail the PD controllers used for both line following and wall following.
- **Gap Rejection**: Discuss the specific gap rejection filter used in Task 02 to ignore the gap and stay in the corridor.

### Slide 13: Subtask Walkthrough (Speaker D - 1:00)
- **Flow**: Walk the audience through the state machine flow for a complete run.
- **Task 01**: Grid search, detect ball color, store it, grip ball.
- **Task 02**: Enter corridor, use ToF sensors for wall following, filter out the gap.
- **Task 03**: Enter second corridor, detect obstacle, lower stance, push it.
- **Task 04**: Reach sorting junction, read floor colors, drop the ball on the matching color, proceed to finish.

### Slide 14: Power System (Speaker E - 0:20)
- **Source**: 2S LiPo battery.
- **Actuator Power**: 5V/15A BEC dedicated to the servos to prevent voltage drops.
- **Logic Power**: Separate 3.3V LDO for the STM32 and sensors for clean logic power.
- **Runtime**: Highlight the estimated 37-minute runtime, emphasizing the 1.5A power savings from removing the Raspberry Pi.

### Slide 15: Task Delegation (Speaker E - 0:30)
- **Team Breakdown**: Present how the workload was divided among the 5 team members (mechanical, electronics, control, software, testing).
- **Timeline**: Briefly show the development timeline and milestones achieved.

### Slide 16: Risks + Mitigation (Speaker E - 0:30)
- **Identify Risks**: Address the top 5 risks identified (servo torque limits, ambient light affecting the TCS34725, false intersection triggers, losing ball color on restart, and loop timing overflows).
- **Solutions**: Explain the backup plans (crawl gait for torque, physical shielding for the color sensor, temporal filtering for intersections, Flash persistence for color, and strict timing analysis).

### Slide 17: Conclusion (Speaker E - 0:20)
- **Philosophy**: Summarize the design philosophy (Reliability and simplicity over raw speed).
- **Confidence**: Reiterate confidence in the chosen approach.
- **Next Steps**: State readiness for the build and testing phase.
- **Close**: Thank the evaluators and open the floor for Q&A.
