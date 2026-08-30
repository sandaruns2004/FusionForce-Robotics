# 06 Decision Making (HFSM)

## Objective
Implement the Hierarchical Finite State Machine (HFSM) that controls the robot's high-level mission logic, seamlessly switching between line following, wall following, obstacle clearing, and color sorting.

## State Machine Framework Setup
Import a state machine library like `smach` or create a robust custom Python class. Define the primary mission states based on the competition tasks:
- `STATE_0_START_TO_GRID`
- `STATE_1_GRID_SEARCH_AND_PICKUP`
- `STATE_2_CORRIDOR_WALL_FOLLOWING`
- `STATE_3_CORRIDOR_OBSTACLE_CLEARING`
- `STATE_4_COLOR_SORTING_JUNCTION`
- `STATE_5_FINISH_LINE_RUN`

## Transition Logic
Write the logic that checks entry and exit conditions.
*Example:* Transition from `STATE_0` to `STATE_1` when the Vision Pipeline triggers the `grid_cross_detected` flag.

## Sub-State Handlers
Within complex states like `STATE_1_GRID_SEARCH_AND_PICKUP`, implement sub-states for hardware actions:
- **Halting locomotion**: Send `CMD_SET_VELOCITY = [0,0,0]`.
- **Arm Gripping Sequence**: Send `CMD_ARM_ACTION = GRAB` to the STM32.
- **Belly Storage Sequence**: Send `CMD_ARM_ACTION = STORE` to the STM32.

## PID Control Implementation
In active movement states, the FSM acts as a controller:
- **Line Following**: Implement a Proportional (P) controller mapping the Vision's centroid error to the `omega` (turn rate) parameter of the `CMD_SET_VELOCITY` UART packet.
- **Wall Following (Task 02)**: Implement a Proportional-Derivative (PD) controller mapping the ToF sensor data ($d_{left} - d_{right}$) received via UART telemetry to maintain a center path.

## Code Structure
```python
class RobotState:
    INIT = 0
    TASK_1_GRID = 1
    TASK_1_GRAB = 2

current_state = RobotState.INIT

def fsm_tick(vision_data, telemetry_data):
    global current_state
    
    if current_state == RobotState.INIT:
        if telemetry_data.switch_pressed:
            current_state = RobotState.TASK_1_GRID
            
    elif current_state == RobotState.TASK_1_GRID:
        # P-Controller for line following
        wz = Kp * vision_data.heading_error
        send_velocity(0.2, 0.0, wz)
        
        if vision_data.intersection_detected:
            current_state = RobotState.TASK_1_GRAB
```
