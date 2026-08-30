# COMMUNICATION PROTOCOL

## 1. UART Specifications
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Hardware Flow Control**: None

## 2. Packet Structure
All communication between the Raspberry Pi and the STM32 follows a rigid, non-ASCII binary structure to ensure parsing speed and reliability.

| Byte | Name | Description |
|---|---|---|
| 0 | `HEADER_1` | Always `0xAA` |
| 1 | `HEADER_2` | Always `0x55` |
| 2 | `MSG_TYPE` | e.g., `0x01` (Command), `0x02` (Telemetry) |
| 3 | `LENGTH` | Number of payload bytes (`N`) |
| 4...4+N | `PAYLOAD` | The actual data |
| 5+N | `CRC8` | Checksum of `MSG_TYPE`, `LENGTH`, and `PAYLOAD` |

## 3. Pi -> STM32 Messages (Commands)

### MSG_TYPE: `0x01` (Set Velocity)
Commands the gait generator to move.
- **Payload Length**: 12 bytes
- **Payload**:
  - Float32 `Vx` (Forward velocity in m/s)
  - Float32 `Vy` (Lateral velocity in m/s)
  - Float32 `Wz` (Angular velocity in rad/s)

### MSG_TYPE: `0x02` (Set Posture)
Overrides the default stance height and pitch (e.g., when lowering to grab the ball).
- **Payload Length**: 8 bytes
- **Payload**:
  - Float32 `Z_Offset` (Height in mm)
  - Float32 `Pitch_Offset` (Angle in rad)

### MSG_TYPE: `0x03` (Action Command)
Triggers a mechanical action outside the main gait.
- **Payload Length**: 1 byte
- **Payload**:
  - `0x00`: Open Gripper
  - `0x01`: Close Gripper
  - `0x02`: Lower Gripper
  - `0x03`: Raise Gripper
  - `0xFF`: EMERGENCY STOP

## 4. STM32 -> Pi Messages (Telemetry)

### MSG_TYPE: `0x10` (Sensor Status)
Sent continuously at 10Hz to feed the Pi's logic layer.
- **Payload Length**: 8 bytes
- **Payload**:
  - Uint16 `ToF_Front` (Distance in mm)
  - Uint16 `ToF_Left` (Distance in mm)
  - Uint16 `ToF_Right` (Distance in mm)
  - Uint16 `Battery_Voltage` (mV)

### MSG_TYPE: `0x11` (Action ACK)
Sent immediately after receiving an Action Command to confirm execution.
- **Payload Length**: 1 byte
- **Payload**:
  - Echoes the Action Command byte.
