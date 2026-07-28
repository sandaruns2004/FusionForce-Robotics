# FusionForce-Robotics

Welcome to the FusionForce-Robotics repository! This project aims to build a 3D-printed quadruped robot using a Raspberry Pi for high-level decision-making and an STM32 for low-level motor control and balancing.

## 🚀 Autonomous Competition Engineering Documentation Suite

We have created a complete, highly detailed engineering specification suite for the 12-servo quadruped robot engineered for the EN2533 Autonomous Robotics Competition (`Task/tasks_circuit_v1.md`):

👉 **[Go to the Master Engineering Documentation Suite](docs/README.md)**

The suite includes 6 comprehensive engineering documents covering:
1. **[Mechanical Design, Kinematics & Chassis Layout](docs/01_Mechanical_Design_and_Chassis_Layout.md)** (12-DOF CAD, Servo Torque Evaluation, CoG Math, Denavit-Hartenberg IK)
2. **[Electrical Architecture & Power Distribution](docs/02_Electrical_and_Power_Distribution.md)** (Power Tree, Pinouts, Star Grounding, I2C Sensor Mux)
3. **[Dual-Board Software Architecture & IPC Protocol](docs/03_Dual_Board_Software_Architecture_and_IPC.md)** (Raspberry Pi 4B + STM32F401 Black Pill UART CRC8 Protocol & Timing)
4. **[Autonomous Task State Machine (HFSM)](docs/04_Autonomous_Task_State_Machine.md)** (Competition Run State Chart, Wall Gap Rejection Filter, Color Memory)
5. **[Algorithmic Implementations & Control Logic](docs/05_Algorithmic_Implementations_and_Control_Logic.md)** (OpenCV Line/Color Math, PD Wall Follow, Bezier Gait IK, Python & C++ Code)
6. **[System Integration, Calibration & Testing Guide](docs/06_System_Integration_and_Testing_Guide.md)** (Servo/ToF/Camera Calibration & 4 Integration Test Checklists)

---

## 📚 Team Learning Curriculum

If you are a team member just getting started, please begin by reading our master syllabus. It contains everything you need to know about the math, physics, and code required for this project:

👉 **[Go to the Robot Curriculum Syllabus](Robot_Curriculum/README.md)**

The curriculum is broken down into modules for:
*   **The Mechanical Team** (Kinematics, 3D printing, Torque)
*   **The Embedded Team** (STM32, PID loops, PWM, IMU)
*   **The Software & AI Team** (Raspberry Pi, OpenCV, State Machines)