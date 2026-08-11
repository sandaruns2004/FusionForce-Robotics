# 08 System Integration

## Objective
Merge the Python Vision Pipeline, the IPC UART Handler, and the HFSM into a single, cohesive main loop. Conduct progressive testing to ensure hardware and software safety.

## Main Loop Architecture
Create a `main.py` script that instantiates the Vision Pipeline class, the UART IPC class, and the HFSM. You must use `threading` to prevent the heavy Vision processing from delaying the UART communications.

Structure the primary loop:
1. **UART Thread**: Read incoming STM32 Telemetry. Write outgoing Velocity packets at 20Hz.
2. **Vision Thread**: Grab a camera frame and process Vision.
3. **Logic Thread**: Tick the HFSM (evaluate state, compute outputs based on Vision and Telemetry, update shared Velocity variable).

## Error Recovery & Hardware Reflexes
Integrate handling for the STM32's hardware reflex. If the UART telemetry reports `motion_status == REFLEX_STOP` (e.g., bumper hit), the FSM must gracefully transition to a recovery state (e.g., reversing or engaging the push stance).

Add timeout logic: if a grid cross is not seen after 10 seconds of line following, halt and pulse an error LED.

## Code Example
```python
import threading
import time

robot_velocity = [0.0, 0.0, 0.0]
state_lock = threading.Lock()

def vision_logic_thread():
    while True:
        # Grab frame, detect lines
        # Tick HFSM
        with state_lock:
            robot_velocity[2] = 0.5 # Example turn
            
def uart_thread():
    while True:
        with state_lock:
            vx, vy, wz = robot_velocity
        # send_packet(vx, vy, wz)
        time.sleep(0.05) # Send at 20Hz

threading.Thread(target=vision_logic_thread, daemon=True).start()
threading.Thread(target=uart_thread, daemon=True).start()

while True: time.sleep(1)
```
