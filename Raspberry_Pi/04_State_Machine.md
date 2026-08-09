# 04. State Machine (HFSM)

## Objective
Implement the Hierarchical Finite State Machine (HFSM) that controls the robot's high-level mission logic, seamlessly switching between line following, wall following, obstacle clearing, and color sorting.

## Reference Documentation
- [04. Autonomous Task State Machine](../docs/04_Autonomous_Task_State_Machine.md)
- [12. State Machine Architecture](../Robot_Curriculum/Module_3_Pi_Logic/12_state_machine_architecture.md)

## Steps to Implement

1. **State Machine Framework Setup**
   - Import a state machine library like `smach` or create a robust custom Python class structure to handle states and transitions.
   - Define the 6 primary states outlined in the autonomous task documentation:
     - `STATE_0_START_TO_GRID`
     - `STATE_1_GRID_SEARCH_AND_PICKUP`
     - `STATE_2_CORRIDOR_WALL_FOLLOWING`
     - `STATE_3_CORRIDOR_OBSTACLE_CLEARING`
     - `STATE_4_COLOR_SORTING_JUNCTION`
     - `STATE_5_FINISH_LINE_RUN`

2. **Transition Logic**
   - Write the logic that checks entry and exit conditions for each state.
   - *Example:* Transition from `STATE_0` to `STATE_1` when the Vision Pipeline triggers the `grid_cross_detected` flag.

3. **Sub-State Handlers**
   - Within complex states like `STATE_1_GRID_SEARCH_AND_PICKUP`, implement sub-states for:
     - Halting locomotion (`CMD_SET_VELOCITY = 0`).
     - Arm Gripping Sequence (`CMD_ARM_ACTION = GRAB`).
     - Belly Storage Sequence (`CMD_ARM_ACTION = STORE`).

4. **PID Control Implementation**
   - In active movement states, implement a proportional controller mapping the Vision's centroid error to the `omega` (turn rate) parameter of the `CMD_SET_VELOCITY` packet.
   - Implement PD (Proportional-Derivative) control for Corridor Wall Following (Task #02) using the ToF sensor data ($d_{left} - d_{right}$) provided via UART telemetry.

## Next Step
Proceed to [05. Integration and Testing](./05_Integration_and_Testing.md) to bring the Vision, IPC, and HFSM together.
