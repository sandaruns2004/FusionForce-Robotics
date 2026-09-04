# FusionForce-Robotics (BREACH PROTOCOL)

Welcome to the FusionForce-Robotics repository! This project engineers a fully autonomous, 12-DOF 3D-printed quadruped robot designed specifically to compete in the **IN24 EN2533 BREACH PROTOCOL** competition.

The architecture uses a **single embedded MCU design**: all perception, mission decision-making, and real-time motor control are handled by one STM32F411CEU6 microcontroller. An 8-channel TCRT5000 IR line array handles line following and intersection detection; a TCS34725 RGBC colour sensor (mounted on the gripper arm tip) handles ball colour identification and floor zone detection.

## 🚀 The Two Engineering Domains

The system is divided into two engineering domains to ensure real-time safety, mechanical simplicity, and clean separation of concerns.

1. **[STM32 + Embedded Sensors (The Brain + Spine)](docs/STM32_ARCHITECTURE.md)**: Bare-metal C controlling all perception (8-ch IR array + TCS34725), Mission State Machine (18 states), Inverse Kinematics, gait generation, and 50Hz PWM.
2. **[Mechanical & Hardware (The Body)](Mechanical/README.md)**: The 12-DOF quadruped chassis, 2-DOF frontal arm+gripper (with TCS34725 at tip), line array bracket, and integrated bumper.

---

## 📚 Master Engineering Documentation Suite

We have conducted a complete documentation audit and generated a highly detailed engineering specification suite tailored for `Task/tasks_circuit_v2.md`.

👉 **[Go to the Technical Blueprint](docs/TECHNICAL_BLUEPRINT.md)** (Start Here)

### Core Planning Documents
- [Technical Blueprint](docs/TECHNICAL_BLUEPRINT.md)
- [Technical Proposal](docs/TECHNICAL_PROPOSAL.md)
- [Execution Report](docs/EXECUTION_REPORT.md)
- [Master Implementation Checklist](docs/MASTER_IMPLEMENTATION_CHECKLIST.md)
- [Risk Register (FMEA)](docs/RISK_REGISTER.md)

### Technical Architecture
- [System Overview](docs/SYSTEM_OVERVIEW.md)
- [Hardware Architecture](docs/HARDWARE_ARCHITECTURE.md)
- [Power Architecture](docs/POWER_ARCHITECTURE.md)
- [Software Architecture](docs/SOFTWARE_ARCHITECTURE.md)
- [**Sensor Perception Strategy**](docs/SENSOR_PERCEPTION.md) ← replaces Computer Vision
- [**Embedded State Machine**](docs/EMBEDDED_STATE_MACHINE.md) ← replaces Pi HFSM
- [STM32 Embedded Architecture](docs/STM32_ARCHITECTURE.md)
- [Locomotion & Kinematics](docs/LOCOMOTION_AND_KINEMATICS.md)
- [Mechanical Design](docs/MECHANICAL_DESIGN.md)

### Interfaces & Safety
- **[Pinout and Connections (Authoritative)](docs/PINOUT_AND_CONNECTIONS.md)**
- [Communication Protocol (UART IPC)](docs/COMMUNICATION_PROTOCOL.md)
- [Safety and Failure Modes](docs/SAFETY_AND_FAILURE_MODES.md)
- [Troubleshooting Guide](docs/TROUBLESHOOTING.md)

---

## 🎓 Team Learning Curriculum

If you are a new team member, you MUST complete the curriculum before touching the codebase. 

👉 **[Master Robotics Curriculum Syllabus](Robot_Curriculum/README.md)**

The curriculum takes you systematically from Level 1 (Fundamentals) to Level 5 (Integration), ensuring you understand the mathematics of inverse kinematics, the physics of PID stabilization, and the computer science of OpenCV before writing competition logic.

---
*Note: This repository is currently in the Planning & Documentation Phase. Source code directories (`/src`, `/firmware`, `/hardware`) will be populated in subsequent development phases as per the Execution Report.*