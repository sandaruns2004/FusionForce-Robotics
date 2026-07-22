# Module 5: UART Communication Protocol

The Raspberry Pi (Brain) and the STM32 (Spinal Cord) are physically connected by three wires:
1.  **GND** (Ground)
2.  **TX** (Transmit) -> connects to the other's **RX**
3.  **RX** (Receive) -> connects to the other's **TX**

They talk over **UART (Universal Asynchronous Receiver-Transmitter)**, also known as Serial. 

## The Problem: Data Corruption

Imagine the Pi wants to tell the STM32 to move Leg 1 to 45 degrees, Leg 2 to 90 degrees.
The Pi sends the string: `45,90`
The STM32 receives: `45,90` and moves the legs.

But what if a wire gets jiggled, or the Pi stutters?
The Pi sends: `45,90`
The STM32 receives: `5,9` -> It suddenly moves the legs to 5 and 9 degrees. The robot violently crashes to the floor.

## The Solution: A Packet Structure

We must wrap our data in a strict **Packet Protocol** with headers, footers, and a checksum.

### Example Packet Structure

```text
[Header] [Command Type] [Payload Length] [Payload Data...] [Checksum] [Footer]
  0xAA       0x01            0x02           0x2D, 0x5A        0x88      0x55
```

1.  **Header (`0xAA`)**: The STM32 ignores all data until it sees the byte `0xAA`. This means a new message is starting.
2.  **Command Type (`0x01`)**: Tells the STM32 what this data means (e.g., `0x01` = Motor Angles, `0x02` = Sleep Mode).
3.  **Payload Length (`0x02`)**: Tells the STM32 to expect exactly 2 more bytes of data before the checksum.
4.  **Payload (`0x2D`, `0x5A`)**: The actual data (45 and 90 in hex).
5.  **Checksum (`0x88`)**: A mathematical verification. (e.g., add all previous bytes together. The receiver does the same math. If the answers don't match, the data was corrupted and the packet is ignored).
6.  **Footer (`0x55`)**: Confirms the end of the message.

## Python (Sender) Example

```python
import serial
import struct

# Open the serial port at 115200 baud
ser = serial.Serial('/dev/serial0', 115200)

def send_angles(angle1, angle2):
    header = 0xAA
    cmd = 0x01
    length = 2
    
    # Calculate simple checksum
    checksum = (header + cmd + length + angle1 + angle2) % 256
    footer = 0x55
    
    # Pack the integers into a binary bytearray
    packet = struct.pack('BBBBBBB', header, cmd, length, angle1, angle2, checksum, footer)
    ser.write(packet)
```

## STM32 C++ (Receiver) Concept

On the STM32 (Arduino IDE), you write a state machine in the `Serial.available()` loop.

1.  Wait for `0xAA`.
2.  Read Command and Length.
3.  Read Payload.
4.  Read Checksum. 
5.  Calculate your own checksum. Do they match?
6.  If YES -> Apply angles to motors.
7.  If NO -> Discard packet.

By implementing this, the communication bridge between the two boards becomes rock-solid, even in electrically noisy environments.

## 📺 Recommended Viewing
*   Search YouTube for: `"Serial Communication Protocol Design Arduino"`
*   Search YouTube for: `"Python struct library tutorial"`
