# RUNNER-4 — 10-Minute Presentation Plan

## Time Budget: 10:00 (strict — do NOT exceed)

---

## Slide Plan & Speaker Assignments

> Adjust speaker names as needed. Assumes 5 team members (A–E). All must present.

| Slide # | Section | Duration | Speaker | Content |
|---------|---------|----------|---------|---------|
| 1 | **Title + Team** | 0:20 | A | Team name, competition name, date, team members |
| 2 | **Problem Statement** | 0:30 | A | 4 subtasks overview; 250mm footprint; 15-min time; fully autonomous |
| 3 | **Overall Strategy** | 0:40 | A | Crawl gait for stability; camera replaces IR/colour sensors; dual-processor; passive pushing |
| 4 | **System Architecture** | 0:30 | B | Block diagram: Pi ↔ STM32 ↔ PCA9685 ↔ Servos; sensor connections; power tree |
| 5 | **Mechanical Design** | 1:00 | B | 12-DOF quadruped; leg geometry; link lengths; body dimensions; 3D printed PETG; footprint compliance |
| 6 | **Ball Mechanism** | 0:40 | B | 2-servo arm + gripper; internal compartment; servo gate; why scoop doesn't work (5cm pedestal) |
| 7 | **Pushing Mechanism** | 0:20 | B | Passive front bumper; low stance strategy |
| 8 | **Actuators** | 0:40 | C | 15× MG90S; torque analysis (2.2 kg·cm vs 1.53 required); why crawl not trot; PCA9685 single board |
| 9 | **Sensors** | 0:50 | C | Camera (line/ball/colour/junction); 3× VL53L0X (wall/obstacle); MPU6050 (stabilization); placement diagram |
| 10 | **Raspberry Pi + Vision** | 0:40 | C | Pi 4B selection (vs Zero 2W vs Pi 5); OpenCV pipeline; HSV thresholds; camera mount position |
| 11 | **STM32 + Control** | 0:30 | D | F401CCU6; 50Hz control loop; IK solver; gait generator; IMU filter |
| 12 | **Algorithms** | 1:00 | D | IK equations; crawl gait sequence; PD line following; PD wall following; gap rejection filter |
| 13 | **Subtask Walkthrough** | 1:00 | D | Step through all 4 subtasks: grid→ball→corridor→push→sort→finish; state machine flow |
| 14 | **Power System** | 0:20 | E | 2S LiPo; 5V/15A BEC; separate rails; ~24 min runtime |
| 15 | **Task Delegation** | 0:30 | E | 5 team areas; who does what; development timeline |
| 16 | **Risks + Mitigation** | 0:30 | E | Top 5 risks: torque, overheating, ball pickup, tipping, camera; backup plans |
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
