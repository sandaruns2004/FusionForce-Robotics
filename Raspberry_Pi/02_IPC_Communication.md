# 02. IPC Communication (UART)

## Objective
Establish a reliable, high-speed, and error-free communication link between the Raspberry Pi and the STM32F401 Black Pill using UART and binary packets.

## Reference Documentation
- [03. Dual-Board Software Architecture & IPC Protocol](../docs/03_Dual_Board_Software_Architecture_and_IPC.md)

## Steps to Implement

1. **Hardware Wiring**
   - Connect the Pi's UART TX/RX pins (usually GPIO 14/15) to the STM32's USART1 RX/TX pins.
   - **CRITICAL:** Ensure both boards share a common Ground (GND).

2. **Enable UART on the Pi**
   - Use `raspi-config` or edit `/boot/config.txt` to enable the serial port and disable the serial console.
   - The device will likely appear as `/dev/serial0` or `/dev/ttyAMA0`.

3. **Python Serial Handler Class**
   - Create a Python class utilizing the `pyserial` library.
   - Implement the **Binary Frame Structure**:
     - `SYNC 1 (0xAA)` | `SYNC 2 (0xBB)` | `CMD_ID` | `LENGTH` | `PAYLOAD` | `CRC-8`
   - Write a function `send_cmd_velocity(vx, vy, omega, gait_type)` mapping to `CMD_ID 0x01`.
   - Write a function `send_arm_action(action_id)` mapping to `CMD_ID 0x02`.

4. **Checksum Implementation**
   - Implement a MAXIM CRC-8 calculation function in Python to append the correct checksum to every outgoing packet.
   - Implement a validation check for all incoming telemetry packets to drop corrupted frames.

5. **Telemetry Polling Thread**
   - Create a background thread or asynchronous task that continuously reads `TEL_SENSOR_STREAM` (ID: `0x81`) from the STM32 at 50Hz.
   - Parse `tof_front_mm`, `tof_left_mm`, `tof_right_mm`, and `motion_status` so the State Machine has instant access to real-time sensor data.

## Next Step
Proceed to [03. Vision Pipeline](./03_Vision_Pipeline.md) to implement the OpenCV logic that will dictate the `vx` and `omega` commands sent over UART.
