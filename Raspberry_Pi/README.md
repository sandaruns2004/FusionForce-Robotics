# Raspberry Pi Workflow & Curriculum

> [!WARNING]
> **⚠️ ARCHIVED — Design Change Notice**
>
> The **Raspberry Pi 4B has been removed** from the RUNNER-4 design. All perception and decision-making has been moved to the **STM32F411CEU6** using embedded sensors:
> - **03_camera** and **04_opencv** → replaced by [docs/SENSOR_PERCEPTION.md](../docs/SENSOR_PERCEPTION.md) (TCS34725 + TCRT5000)
> - **05_perception** → replaced by [docs/SENSOR_PERCEPTION.md](../docs/SENSOR_PERCEPTION.md)
> - **06_decision_making** (HFSM) → replaced by [docs/EMBEDDED_STATE_MACHINE.md](../docs/EMBEDDED_STATE_MACHINE.md) (C firmware)
> - **07_uart** (Pi↔STM32 UART) → removed; no inter-CPU communication
>
> The modules in this directory remain here **for learning reference only**. They are not required for competition. The code and concepts are valid Python/OpenCV curriculum but are not part of the active RUNNER-4 codebase.

---

## Historical Context
The Raspberry Pi 4B serves as the central "Brain" of the FusionForce robot. It handles computationally heavy tasks like computer vision (OpenCV) and high-level decision making (Hierarchical State Machine), while offloading real-time motion and reflex control to the STM32 microcontroller.

## System Workflow & Architecture

```mermaid
flowchart TD
    %% Define styles
    classDef phase fill:#2b5876,stroke:#4e4376,color:#fff,stroke-width:2px;
    
    subgraph "Raspberry Pi Workflow Phases"
        A[01. Environment Setup\nLinux, Python, SSH]:::phase
        B[02. IPC Communication\nUART, CRC8, Serial Handler]:::phase
        C[03. Vision Pipeline\nOpenCV, HSV, Centroid Tracking]:::phase
        D[04. State Machine - HFSM\nMission States and Transitions]:::phase
        E[05. System Integration\nCombine all modules with STM32]:::phase
        
        A --> B
        B --> C
        C --> D
        D --> E
    end
```

## Learning Path & Modules

Follow these numbered directories for a complete learning path from basic setup to competition deployment.

1.  **[01 Setup](./01_setup/README.md)** - OS, SSH, VNC, Python Dependencies
2.  **[02 Linux Fundamentals](./02_linux/README.md)** - Terminal navigation, Permissions
3.  **[03 Camera Configuration](./03_camera/README.md)** - Pi Camera Module 3 Setup
4.  **[04 OpenCV Basics](./04_opencv/README.md)** - Grayscale, Thresholds, Line Tracking
5.  **[05 Perception Algorithms](./05_perception/README.md)** - HSV Color, Intersection Detection
6.  **[06 Decision Making & State Machines](./06_decision_making/README.md)** - HFSM, Mission Logic, PID
7.  **[07 UART Communication](./07_uart/README.md)** - IPC, Binary Packets, Telemetry
8.  **[08 System Integration](./08_integration/README.md)** - Main Loop, Error Recovery
9.  **[09 Testing Methodologies](./09_testing/README.md)** - Dry Runs, Tethered Testing
10. **[10 Final Deployment](./10_deployment/README.md)** - Systemd services for headless boot

---
🔙 **[Back to Main Repository README](../README.md)**
