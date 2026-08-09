# 05. Integration and Testing

## Objective
Merge the Python Vision Pipeline, the IPC UART Handler, and the HFSM into a single, cohesive main loop. Conduct progressive testing to ensure hardware and software safety.

## Reference Curriculum
- [Module 5: Integration](../Robot_Curriculum/Module_5_Integration)

## Steps to Implement

1. **Main Loop Architecture**
   - Create a `main.py` script that instantiates the Vision Pipeline class, the UART IPC class, and the HFSM.
   - Structure the primary 20-30Hz loop:
     1. Read incoming STM32 Telemetry.
     2. Grab a camera frame and process Vision.
     3. Tick the HFSM (evaluate state and compute outputs).
     4. Build and transmit the `CMD_SET_VELOCITY` / `CMD_ARM_ACTION` packet.

2. **Error Recovery & Hardware Reflexes**
   - Integrate handling for the STM32's hardware reflex. If the UART telemetry reports `motion_status == REFLEX_STOP`, the FSM must gracefully transition to a recovery state (e.g., reversing or engaging the push stance).
   - Add timeout logic (e.g., if a grid cross is not seen after 10 seconds of line following, halt and pulse an error LED).

3. **Dry-Run Testing (Pi Only)**
   - Before connecting to the STM32, test the Pi script using a mocked Serial class that returns fake ToF distances. Verify that the FSM transitions states correctly based on fake sensor inputs and real camera feeds.

4. **Tethered Field Testing**
   - Connect the Pi and STM32. Place the robot on a test stand (legs off the ground).
   - Ensure that moving a line under the camera accurately alters the PWM servo angles on the legs (proving the entire Pi -> UART -> STM32 -> IK -> Servo chain).

5. **Untethered Track Runs**
   - Begin full track runs. Start by testing isolated tasks (e.g., just the Task 02 Corridor) before attempting a full run from Start to Finish.
