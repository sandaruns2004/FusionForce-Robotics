# 🤖 Legged Robot Curriculum: Master Syllabus

Welcome to the **FusionForce Robotics** legged robot curriculum! This folder contains everything you need to learn how to build, program, and balance a 3D-printed quadruped robot from scratch.

This project is split across two main processing units:
1.  **Raspberry Pi (The Brain)**: Handles High-Level vision, AI, and complex math.
2.  **STM32 Microcontroller (The Spinal Cord)**: Handles Low-Level motor control, sensor reading, and real-time balancing.

---

## 🗺️ Where Should I Start?

Find your team below and follow the recommended learning path.

### ⚙️ The Mechanical & Hardware Team
**Your Goal:** Design the physical legs, choose the right motors, calculate the required torque, and ensure the robot can physically support itself.
**Your Learning Path:**
1.  Read all files in `Module_1_Mechanics/` to understand linkages, kinematics, and torque.
2.  Read `Module_5_Integration/17_calibration_and_debugging.md` to learn how to assemble and test the legs safely.
3.  Collaborate with the STM32 team to test physical movements.

### ⚡ The Embedded Systems Team (STM32)
**Your Goal:** Write C++ code (Arduino IDE) to read the balancing sensors (IMU) and precisely control the 12 motors using PID loops and PWM signals.
**Your Learning Path:**
1.  Read all files in `Module_2_Low_Level_Control/`.
2.  Read `Module_5_Integration/16_uart_communication_protocol.md` to learn how you will receive commands from the Raspberry Pi.

### 🧠 The Software & AI Team (Raspberry Pi)
**Your Goal:** Write Python code to process camera feeds (OpenCV), calculate walking animations (Bezier curves), and make high-level decisions.
**Your Learning Path:**
1.  Read all files in `Module_3_Pi_Logic/` and `Module_4_Vision/`.
2.  Read `Module_1_Mechanics/04_inverse_kinematics.md` so you understand the math required to command the physical legs.
3.  Read `Module_5_Integration/16_uart_communication_protocol.md` to learn how to send your calculated angles to the STM32.

---

## 📚 Curriculum Index

### Module 1: Mechanics, Kinematics, & Power
*   `01_mechanical_design_principles.md`
*   `02_torque_and_statics.md`
*   `03_forward_kinematics.md`
*   `04_inverse_kinematics.md`
*   `05_3d_printing_for_robotics.md`
*   `06_power_distribution.md`

### Module 2: Low-Level Control (STM32)
*   `07_stm32_arduino_setup.md`
*   `08_actuators_pwm.md`
*   `09_pid_control_theory.md`
*   `10_imu_and_sensor_fusion.md`

### Module 3: Pi Setup & Core Logic
*   `11_pi_linux_and_python.md`
*   `12_state_machine_architecture.md`
*   `13_gait_generation_curves.md`

### Module 4: Computer Vision
*   `14_opencv_fundamentals.md`
*   `15_advanced_vision_apriltags.md`

### Module 5: Integration & Testing
*   `16_uart_communication_protocol.md`
*   `17_calibration_and_debugging.md`
