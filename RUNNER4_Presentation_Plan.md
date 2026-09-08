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
- **Slide Content**: Project RUNNER-4, Team FusionForce, Event & Date.
- **Suggested Visual**: High-quality 3D CAD render of the RUNNER-4 robot on the title screen, with team and competition logos.
- **Speaker Script**: 
  - *"Good morning evaluators, and welcome. We are Team FusionForce, and today we are excited to present our final design for Project RUNNER-4. Before we begin, I'd like to quickly introduce the team: Minura and Thilishan lead our Mechanical Design, Pulina and Thushanthan handle Electronics and Power, Sandaru and Pulina are responsible for Control Systems and Software, while Minura and Thushanthan spearhead our Testing and Validation."*

### Slide 2: Problem Statement (Speaker A - 0:30)
- **Slide Content**: Fully autonomous navigation of 4 distinct subtasks, strict 250mm footprint limit, 15-minute maximum time limit.
- **Suggested Visual**: "Full Tasks Design Diagram" showing a zoomed-out, top-down view of the entire course (Grid $\rightarrow$ Corridors $\rightarrow$ Sorting Zone).
- **Speaker Script**: 
  - *"Our core challenge is to design a fully autonomous robot that can successfully navigate a circuit of four distinct subtasks. We must do all of this while adhering to a strict 250-millimeter footprint limit, and within a maximum time of 15 minutes. The four phases include: First, navigating a grid to retrieve a colored ball. Second, following a corridor while ignoring a gap in the wall. Third, pushing an obstacle out of our path in a second corridor. And finally, arriving at a junction to sort the ball into the correct zone based on the color we detected earlier."*

### Slide 3: Overall Strategy (Speaker A - 0:40)
- **Slide Content**: Crawl gait for maximum stability, centralized control via single STM32, 8-channel IR array for robust line following, arm-mounted sensor for direct color detection, passive pushing mechanism.
- **Suggested Visual**: Collage of key components (STM32 board, IR array, side-profile sketch of a quadruped walking).
- **Speaker Script**: 
  - *"Our overall strategy for RUNNER-4 prioritizes reliability and stability over raw speed. For locomotion, we've implemented a crawl gait to ensure three feet are always securely on the ground. Computationally, we've moved away from heavy processors like a Raspberry Pi, instead unifying perception, decision-making, and control onto a single STM32 microcontroller. To make this possible, we replaced complex camera vision with a fast 8-channel IR array for line following and a precision color sensor mounted directly on the gripper arm. Finally, to keep the design lightweight and simple, we're using a passive front bumper for the pushing task rather than adding extra motors."*

### Slide 4: System Architecture (Speaker B - 0:30)
- **Slide Content**: Central MCU: STM32F411, I2C1 Bus: Actuators, IMU, Color Sensor, I2C2 Bus: ToF Wall Sensors, Split Power Tree: 3.3V Logic / 5V Actuators.
- **Suggested Visual**: System Architecture Block Diagram showing the STM32 in the center, arrows pointing to I2C1, I2C2, and GPIO, and color-coded power delivery paths.
- **Speaker Script**: 
  - *"Taking a look at our system architecture diagram, everything centers around the STM32F411 microcontroller. We've carefully managed our data buses to prevent bottlenecks. The I2C1 bus handles our actuator driver, the IMU, and the color sensor, while we've dedicated the I2C2 bus entirely to our three Time-of-Flight wall sensors. Our 8-channel line array is read directly via fast GPIO pins. Crucially, as you can see at the bottom, our power tree is completely split: a 3.3-volt LDO provides clean power to the logic and sensors, while a heavy-duty 5-volt BEC provides dedicated, high-current power exclusively to the servos."*

### Slide 5: Actuators (Speaker C - 0:40)
- **Slide Content**: 15x MG90S Micro Servos, Torque Feasibility: 1.53 kg·cm required (2.2 kg·cm max), Driven by a single PCA9685 board.
- **Suggested Visual**: Photo of the MG90S servo and a simple bar chart comparing "Required Torque" vs "Max Torque".
- **Speaker Script**: 
  - *"For actuation, RUNNER-4 uses a total of 15 MG90S micro servos, all driven by a single PCA9685 board to simplify our wiring harness. During our design phase, we performed a strict torque analysis. We found that our joints require a maximum of 1.53 kilogram-centimeters of torque, which sits comfortably below the MG90S's 2.2 maximum rating. However, this safety margin is only valid for a crawl gait where only one leg lifts at a time. Attempting a trot gait would overstress these motors, which firmly validated our decision to use the slower, statically stable crawl gait."*

### Slide 6: Sensors (Speaker C - 0:50)
- **Slide Content**: 8-ch TCRT5000: Line & intersection tracking, TCS34725: Ball & floor zone color sensing, 3x VL53L0X ToF: Wall & gap detection, MPU6050 IMU: Body stabilization.
- **Suggested Visual**: Bottom-up or top-down drawing of the robot with bright markers pointing to where the IR array, ToF sensors, and Color sensor are mounted.
- **Speaker Script**: 
  - *"Here you can see our sensor placement diagram. Underneath the front chassis, we have an 8-channel TCRT5000 IR array which gives us high-resolution line and intersection tracking. On the tip of the arm is our TCS34725 color sensor; we cleverly use this one sensor for two purposes: reading the ball color when the arm is raised, and reading the floor sorting zones when the arm is angled down. For spatial awareness in the corridors, we mounted three Time-of-Flight sensors on the front left, right, and center. Lastly, an IMU sits directly at the center of mass to monitor body tilt during the pushing task."*

### Slide 7: Mechanical Design (Speaker B - 1:00)
- **Slide Content**: 12-DOF Quadruped chassis, 3D-printed PETG construction, optimized leg geometry, compliant with 250mm footprint.
- **Suggested Visual**: 2D kinematic diagram of a single leg (link lengths & joint angles) next to a top-down CAD view with a 250x250mm bounding box overlay.
- **Speaker Script**: 
  - *"For the mechanical design, we built a 12-DOF quadruped chassis from 3D-printed PETG. This material gives us the perfect balance of durability and weight savings, keeping our total mass under 550 grams. The leg geometry and specific link lengths were carefully optimized to provide sufficient ground clearance for obstacles while ensuring a stable stride. Crucially, as you can see in the footprint overlay, our design fits entirely within a 250 by 250-millimeter bounding box, maintaining strict compliance with the competition's size constraints."*

### Slide 8: Ball Mechanism (Speaker B - 0:40)
- **Slide Content**: 2-Servo arm & gripper assembly, internal compartment with secure servo gate, replaces scoop mechanisms.
- **Suggested Visual**: Zoomed-in CAD render of the 2-servo arm and gripper holding a ball, with a transparent view showing the internal storage compartment.
- **Speaker Script**: 
  - *"Handling the ball in Task 1 required a reliable mechanism. We rejected a simple scoop design because it wouldn't be able to retrieve a ball sitting on a 5-centimeter pedestal or handle uneven terrain. Instead, we designed an active 2-servo arm with a custom gripper. This arm reaches out, securely grasps the 40-millimeter ball, and pulls it into an internal belly compartment. Once inside, a third servo controls a gate that securely locks the ball in place for the remainder of the circuit until it's time for a gravity-assisted drop-off in Task 4."*

### Slide 9: Pushing Mechanism (Speaker B - 0:20)
- **Slide Content**: Passive front bumper design, utilizes "low stance" for maximum traction, eliminates active plow.
- **Suggested Visual**: Side-profile illustration of the robot in its "low stance" making contact with the block obstacle.
- **Speaker Script**: 
  - *"For the pushing task, we wanted to avoid the complexity and weight of an extra motor or active plow. Therefore, we integrated a passive flat bumper plate into the front chassis. When the ToF sensors detect the heavy obstacle in Task 3, the robot intentionally drops into a 'low stance'. This lowers our center of mass and maximizes leg traction, allowing the robot to use its own quadruped leg force to efficiently bulldoze the obstacle out of the way before standing back up."*

### Slide 10: Embedded Perception (Speaker C - 0:50)
- **Slide Content**: Weighted centroid algorithm for lines, R/G/B ratio classification for color, temporal filtering for gaps, no Raspberry Pi delays.
- **Suggested Visual**: Simple graphic showing a black line underneath an 8-sensor array, with a red dot showing the "calculated center" to explain the weighted centroid algorithm.
- **Speaker Script**: 
  - *"For perception, we made a crucial decision to omit a Raspberry Pi and camera setup. This eliminates boot delays, saves weight, and removes a major point of failure. Instead, we rely entirely on IR and color sensors. For line tracking, we use a weighted centroid algorithm on our 8-channel IR array to pinpoint the exact center of the line. For color, our TCS34725 sensor uses an R/G/B ratio classification and operates in two physical modes: zero degrees to look at the ball, and negative seventy degrees to look at the floor. Finally, to prevent false triggers at intersections, we implemented a robust temporal filter."*

### Slide 11: STM32F411 + Control (Speaker D - 0:30)
- **Slide Content**: 512KB Flash, responsive 50Hz control loop, Flash EEPROM for ball color persistence.
- **Suggested Visual**: Photo of the STM32F411 microcontroller and a simple loop graphic ("Read Sensors -> Process State -> Move Servos" at 50Hz).
- **Speaker Script**: 
  - *"At the heart of RUNNER-4 is the STM32F411 microcontroller. We specifically chose the CEU6 variant because its 512 kilobytes of flash memory provides ample space for our complex state machines and all sensor drivers. The entire system runs on a strict 50-Hertz control loop, ensuring the robot remains highly responsive. This loop handles everything, including integrating the inverse kinematics solver with our gait generator. An important safety feature we've added is using the Flash memory as an EEPROM to persist the detected ball color; if the robot ever loses power and restarts, it won't forget the ball's color for the final sorting task."*

### Slide 12: Algorithms (Speaker D - 1:00)
- **Slide Content**: Inverse Kinematics (IK), crawl gait sequence generator, PD controllers for line/wall, gap rejection filter.
- **Suggested Visual**: A 4-step graphic illustrating the leg lifting order for the crawl gait sequence.
- **Speaker Script**: 
  - *"Moving onto our algorithms, movement is governed by Inverse Kinematics equations that calculate the exact joint angles required to position the legs in 3D space. These feed into our crawl gait sequence generator, which ensures three legs are always on the ground for maximum static stability. For navigation, we use standard Proportional-Derivative, or PD, controllers for both line following on the grid and wall following in the corridors. Finally, to handle Task 2 successfully, we built a gap rejection filter into our wall-following logic—if the ToF sensor suddenly reads a large distance, the algorithm ignores the gap and maintains its current trajectory."*

### Slide 13: Subtask Walkthrough (Speaker D - 1:00)
- **Slide Content**: Task 01: Grid, Task 02: Corridor Gap, Task 03: Corridor Obstacle, Task 04: Color Drop-off.
- **Suggested Visual**: State machine flowchart linking Tasks 1 through 4 with their specific triggers and transitions.
- **Speaker Script**: 
  - *"Let's walk through the full circuit execution using our state machine flowchart. The run begins in Task 1 with a grid search; the robot finds the pedestal, reads and stores the ball's color in memory, and grips it. It then transitions to Task 2, entering the first corridor where it switches from line-following to ToF wall-following, carefully filtering out the large gap in the wall. In Task 3, it enters the second corridor, detects the obstacle, drops into a low stance, and pushes it clear. Finally, in Task 4, the robot reaches the sorting junction, angles its arm down to read the floor colors, and drops the ball in the zone that matches the color we stored back in Task 1, before proceeding to the finish line."*

### Slide 14: Power System (Speaker E - 0:20)
- **Slide Content**: 2S LiPo Battery, 5V / 15A BEC (Servos), 3.3V LDO (Logic), ~37 min runtime.
- **Suggested Visual**: Power tree schematic splitting the 2S LiPo into the 5V BEC and 3.3V LDO, emphasizing physical separation of rails.
- **Speaker Script**: 
  - *"Powering the entire system is a lightweight 2S LiPo battery. As mentioned earlier, keeping our power rails clean was a top priority. We use a heavy-duty 15-Amp BEC dedicated entirely to the servos to handle sudden current spikes without browning out the system. A completely separate 3.3-volt LDO provides smooth, isolated power to the STM32 and our sensors. Furthermore, by removing the Raspberry Pi from our architecture, we saved over 1.5 Amps of continuous current draw, giving RUNNER-4 an estimated runtime of 37 minutes—more than double the 15-minute competition limit."*

### Slide 15: Task Delegation
- **Slide Content**: Mechanical (Minura, Thilishan), Electronics & Power (Pulina, Thushanthan), Control/Software (Sandaru, Pulina), Testing (Minura, Thushanthan).
- **Suggested Visual**: A photo of the team with roles labeled, or a simple timeline/Gantt chart of development phases.
- **Speaker Script**: 
  - *"To achieve this within our timeframe, we divided the project into five core responsibilities based on our strengths. Minura and Thilishan handled the mechanical CAD and 3D printing. Pulina and Thushanthan designed the power distribution and wiring harnesses. Sandaru and Pulina developed the STM32 firmware, including the IK solver, state machine, and sensor algorithms. Finally, Minura and Thushanthan led our integration and testing phase. As you can see on our timeline, this parallel development allowed us to hit our milestones efficiently and leaves us with ample time for end-to-end testing."*

### Slide 16: Risks + Mitigation (Speaker E - 0:30)
- **Slide Content**: Torque limits vs. Crawl Gait, Ambient Light vs. Shielding, False Intersections vs. Temporal Filters, Reboot Data Loss vs. Flash EEPROM.
- **Suggested Visual**: Clean 2-column table with a "Warning" icon for risks and a "Checkmark" icon for mitigations.
- **Speaker Script**: 
  - *"We proactively identified five major risks and implemented strict mitigations for each. First, to avoid exceeding servo torque limits, we locked the robot into a crawl gait. Second, to prevent ambient light from throwing off our color sensor, we designed a custom physical shroud for the arm tip. Third, to prevent the IR array from misinterpreting noise as an intersection, we implemented temporal filtering in the code. Fourth, if the robot suffers a sudden power loss, we prevent data loss by actively saving the ball's color to the STM32's Flash EEPROM. Finally, to prevent control loop overflows, we rigorously profiled our code to guarantee it runs comfortably within our 50-Hertz window."*

### Slide 17: Conclusion (Speaker E - 0:20)
- **Slide Content**: Design Focus: Reliability and simplicity, Ready for build/testing, Q&A Session.
- **Suggested Visual**: Final action-shot photo or render of the robot gripping the ball, and large "Questions?" text.
- **Speaker Script**: 
  - *"In conclusion, RUNNER-4 is built on a philosophy of reliability and simplicity over raw speed. By streamlining our compute architecture, relying on robust embedded sensors, and prioritizing static stability, we have engineered a system that is highly resilient to the unpredictable nature of competition environments. We are fully confident in this design and are completely ready to move into the final build and testing phase. Thank you for your time, and we'd now like to open the floor to any questions."*
