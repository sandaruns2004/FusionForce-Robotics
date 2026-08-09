import serial
import struct
import threading
import time

# Protocol Constants
SYNC_1 = 0xAA
SYNC_2 = 0xBB

# Command IDs (Pi -> STM32)
CMD_SET_VELOCITY = 0x01
CMD_ARM_ACTION = 0x02
CMD_BODY_ATTITUDE = 0x03
CMD_ESTOP = 0x04

# Telemetry IDs (STM32 -> Pi)
TEL_SENSOR_STREAM = 0x81

def calc_crc8_maxim(data: bytes) -> int:
    """
    Calculates the CRC-8/MAXIM (Polynomial 0x31, reversed 0x8C)
    Initial value: 0x00, Final XOR: 0x00
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

class SerialHandler:
    def __init__(self, port='/dev/serial0', baudrate=115200, mock_mode=False):
        self.mock_mode = mock_mode
        self.telemetry_data = {}
        self.running = False
        
        if not self.mock_mode:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
        else:
            print(f"[MOCK] Serial opened on {port} at {baudrate} baud.")
            
    def _send_packet(self, cmd_id: int, payload: bytes):
        length = len(payload)
        
        # We calculate the CRC over the CMD_ID, LENGTH, and PAYLOAD
        crc_data = bytes([cmd_id, length]) + payload
        crc = calc_crc8_maxim(crc_data)
        
        packet = bytes([SYNC_1, SYNC_2]) + crc_data + bytes([crc])
        
        if self.mock_mode:
            hex_str = ' '.join(f"{b:02X}" for b in packet)
            print(f"[MOCK TX] {hex_str}")
        else:
            self.ser.write(packet)
            
    def send_set_velocity(self, vx: int, vy: int, omega: int, gait_type: int):
        """
        vx, vy (int16 mm/s)
        omega (int16 deg/s)
        gait_type (uint8: 0=Stop, 1=Trot, 2=Crawl)
        Format: <hhhB (Little Endian, 3x 16-bit int, 1x 8-bit unsigned)
        """
        payload = struct.pack('<hhhB', vx, vy, omega, gait_type)
        self._send_packet(CMD_SET_VELOCITY, payload)
        
    def send_arm_action(self, action_id: int):
        """
        action_id (uint8: 0=HOME, 1=GRAB, 2=STORE, 3=RELEASE, 4=PUSH_READY)
        """
        payload = struct.pack('<B', action_id)
        self._send_packet(CMD_ARM_ACTION, payload)
        
    def send_body_attitude(self, pitch: int, roll: int, height: int = 80):
        """
        pitch, roll (int8 deg)
        height (uint8 mm)
        """
        payload = struct.pack('<bbB', pitch, roll, height)
        self._send_packet(CMD_BODY_ATTITUDE, payload)
        
    def send_estop(self):
        """Emergency Stop - No payload"""
        self._send_packet(CMD_ESTOP, b'')

    def _read_telemetry_loop(self):
        """Background thread function to poll for incoming 50Hz telemetry."""
        while self.running:
            if self.mock_mode:
                time.sleep(0.02) # 50Hz simulated delay
                continue
                
            # Wait for SYNC_1
            if self.ser.read(1) != bytes([SYNC_1]):
                continue
            # Wait for SYNC_2
            if self.ser.read(1) != bytes([SYNC_2]):
                continue
                
            header = self.ser.read(2)
            if len(header) < 2:
                continue
                
            cmd_id, length = header[0], header[1]
            payload = self.ser.read(length)
            crc_byte = self.ser.read(1)
            
            if len(payload) != length or len(crc_byte) != 1:
                continue
                
            # Validate CRC
            crc_data = header + payload
            if calc_crc8_maxim(crc_data) != crc_byte[0]:
                print("[WARNING] Serial Checksum Mismatch. Dropping packet.")
                continue
                
            # Parse Payload if it's the 50Hz sensor stream
            if cmd_id == TEL_SENSOR_STREAM and length == 11:
                # Format: <HHHbbHB (uint16, uint16, uint16, int8, int8, uint16, uint8)
                unpacked = struct.unpack('<HHHbbHB', payload)
                self.telemetry_data = {
                    'tof_left': unpacked[0],
                    'tof_right': unpacked[1],
                    'tof_front': unpacked[2],
                    'imu_pitch': unpacked[3],
                    'imu_roll': unpacked[4],
                    'battery_mv': unpacked[5],
                    'motion_status': unpacked[6]
                }
                
    def start_telemetry_thread(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_telemetry_loop, daemon=True)
        self.thread.start()
        
    def stop(self):
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        if not self.mock_mode:
            self.ser.close()

if __name__ == "__main__":
    # Quick Test in Mock Mode
    ipc = SerialHandler(mock_mode=True)
    
    print("Testing CMD_SET_VELOCITY (vx=150, vy=0, omega=-15, gait=1):")
    ipc.send_set_velocity(150, 0, -15, 1)
    
    print("\nTesting CMD_ARM_ACTION (GRAB):")
    ipc.send_arm_action(1)
    
    print("\nTesting CMD_ESTOP:")
    ipc.send_estop()
