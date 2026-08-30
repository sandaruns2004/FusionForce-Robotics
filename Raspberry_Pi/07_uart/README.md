# 07 IPC Communication (UART)

## Objective
Establish a reliable, high-speed, and error-free communication link between the Raspberry Pi and the STM32F411 Black Pill using UART and binary packets.

## Hardware Wiring
- Connect Pi UART TX (GPIO 14) to STM32 UART RX (PA10).
- Connect Pi UART RX (GPIO 15) to STM32 UART TX (PA9).
- **CRITICAL:** Ensure both boards share a common Ground (GND).

## Enable UART on the Pi
Use `raspi-config` to enable the serial port and **disable the serial console** so Linux doesn't print login prompts to the STM32.

## Python Serial Handler Class
Create a class utilizing `pyserial`. Implement the **Binary Frame Structure**:
`SYNC 1 (0xAA)` | `SYNC 2 (0x55)` | `CMD_ID` | `LENGTH` | `PAYLOAD` | `CRC-8`

Write mapping functions:
- `send_cmd_velocity(vx, vy, omega, gait_type)` mapping to `CMD_ID 0x01`.
- `send_arm_action(action_id)` mapping to `CMD_ID 0x02`.

## Checksum Implementation
Implement a MAXIM CRC-8 calculation function in Python to append the correct checksum to every outgoing packet. The STM32 will drop any packet where the CRC fails.

## Telemetry Polling Thread
Create a background thread that continuously reads the `TEL_SENSOR_STREAM` (ID: `0x10`) from the STM32 at 10Hz-50Hz. Parse `tof_front`, `tof_left`, `tof_right`, and `battery_voltage` so the State Machine has instant access to real-time sensor data.

## Code Implementation
```python
import serial
import struct

ser = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=0.1)

def send_velocity(vx, vy, wz):
    payload = struct.pack('<fff', vx, vy, wz) # 12 bytes
    crc = sum(payload) & 0xFF # Simplified checksum for example
    packet = bytes([0xAA, 0x55, 0x01, 12]) + payload + bytes([crc])
    ser.write(packet)
```
