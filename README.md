# FusionForce-Robotics (BREACH PROTOCOL)

Welcome to the FusionForce-Robotics repository! This project engineers a fully autonomous, 12-DOF 3D-printed quadruped robot designed specifically to compete in the **IN24 EN2533 BREACH PROTOCOL** competition. 

The architecture strictly separates high-level computer vision and decision-making (Raspberry Pi 4B) from hard real-time postural stabilization and locomotion (STM32 Black Pill).

## 🚀 The Three Engineering Domains

The system is rigorously divided into three engineering domains to ensure real-time safety, modularity, and a clean separation of concerns.

1. **[Raspberry Pi + Computer Vision (The Brain)](Raspberry_Pi/README.md)**: Python-based OpenCV pipelines and the High-Level Finite State Machine (HFSM).
2. **[STM32 Low-Level Control (The Spine)](docs/STM32_ARCHITECTURE.md)**: Bare-metal C controlling Inverse Kinematics (IK), IMU stabilization, and 50Hz PWM generation.
3. **[Mechanical & Hardware (The Body)](Mechanical/README.md)**: The authoritative assembly workflow for the 12-DOF Quadruped chassis, the 2-DOF frontal micro-gripper, and the integrated bumper.

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
- [Computer Vision Strategy](docs/COMPUTER_VISION.md)
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