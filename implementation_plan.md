# RUNNER-4 Full Engineering Report — Implementation Plan

## Repository Inspection Summary

After thorough inspection of the entire FusionForce-Robotics repository, here is what exists:

### Confirmed Implementation Status

| Component | Status | Key Findings |
|-----------|--------|-------------|
| **MCU** | ⚠️ **Inconsistency** | `Quadruped_Robot_Hardware_Plan.md` says **STM32F411CEU6**, but ALL `docs/` engineering specs reference **STM32F401CCEU**. The actual board needs physical verification. |
| **Raspberry Pi** | ✅ Documented | Repository consistently specifies **Raspberry Pi 4B** (not Zero 2 W) |
| **Servos** | ⚠️ **Inconsistency** | Hardware plan lists DS3218/DS3225; docs recommend **Hybrid MG996R + MG90**; CAD assembly (`quadruped-spider-2`) uses **12× SG90** servos. None use MG90S as the user prompt suggests. |
| **Battery** | ⚠️ **Inconsistency** | Hardware plan says **3S LiPo**; user prompt specifies **2S LiPo**. Docs use 3S (11.1V). |
| **PCA9685** | ✅ Confirmed | Single board, 14 channels used (12 leg + 2 arm), 2 spare |
| **IMU** | ✅ Confirmed | MPU6050, on I2C1 bus |
| **ToF Sensors** | ✅ Confirmed | 3× VL53L0X (Front/Left/Right), via TCA9548A mux or XSHUT |
| **Camera** | ✅ Confirmed | Pi Camera Module 3, CSI interface, 640×480 @ 60 FPS |
| **IPC Protocol** | ✅ Implemented | UART binary protocol with CRC8 — working Python code in `Raspberry_Pi/src/ipc_handler.py` |
| **IK Solver** | ✅ Documented | C++ reference code with link lengths: Coxa=30mm, Femur=55mm, Tibia=60mm |
| **Vision Pipeline** | ✅ Documented | OpenCV HSV segmentation, PD line following, Python reference code |
| **State Machine** | ✅ Documented | HFSM with 6 states (STATE_0 through STATE_5) |
| **Ball Mechanism** | ✅ Documented | 2-servo arm + gripper + ventral belly storage cage |
| **Pushing** | ✅ Documented | Front bumper + lowered arm as pushing face |
| **CAD Models** | ✅ Present | Two SolidWorks assemblies: spider-2 (SG90-based) and robot-7 (MG90-based) |
| **Testing** | ✅ Documented | 4 integration test checklists |
| **STM32 Firmware** | ❌ Not implemented | No actual C/C++ firmware code in repository |
| **Vision Code** | ❌ Not implemented | Only reference snippets, no runnable vision pipeline |
| **Gait Engine** | ❌ Not implemented | Only Bezier math documentation, no running code |

### Critical Issues Identified
1. **MCU mismatch**: F411 vs F401 — must be resolved
2. **Servo mismatch**: SG90 in CAD vs MG996R/MG90 hybrid in docs vs MG90S in user prompt vs DS3218 in hardware plan
3. **Battery mismatch**: 3S in docs vs 2S in user prompt
4. **Voltage mismatch**: 3S = 11.1V, but competition max is 24V DC (both compliant)
5. **No actual firmware or vision code** — only documentation and one IPC handler module

## Proposed Deliverables

I will create 5 output files:

1. **`RUNNER4_Full_Engineering_Report.md`** — Comprehensive ~37-section report
2. **`RUNNER4_2Page_Summary.md`** — Concise 2-page proposal
3. **`RUNNER4_Presentation_Plan.md`** — 10-minute presentation structure
4. **`RUNNER4_QA_Preparation.md`** — 25+ evaluator Q&A
5. **`RUNNER4_Decision_Table.md`** — Final engineering decision table

> [!IMPORTANT]
> The report will reconcile all inconsistencies found in the repository and align with the user's prompt (MG90S servos, 2S LiPo, STM32 Black Pill) while clearly noting where repository documentation differs.

## Open Questions

> [!WARNING]
> **STM32 Model**: The hardware plan says F411CEU6 but all docs say F401CCEU. The report will note both and recommend physical verification. Which chip do you actually have?

> [!IMPORTANT]
> **Servo Decision**: The CAD uses SG90 (plastic gear, ~1.8 kg·cm). The docs recommend MG996R+MG90 hybrid. Your prompt says MG90S (metal gear, ~2.2 kg·cm). The report will analyze all three against torque requirements and make a recommendation.

> [!IMPORTANT]
> **Battery**: Your prompt says 2S LiPo (7.4V) but all existing docs use 3S LiPo (11.1V). 2S is lighter but may require a different BEC. The report will analyze both.

Shall I proceed with generating all 5 deliverables?
