import serial
import time
import struct
import threading
from typing import Dict, List, Optional

class FeetechBus:
    """
    Feetech STS3215 serial bus driver.
    Implemented from scratch based on reverse engineering of standard Feetech protocols.
    """

    # Standard instruction definitions
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_READ = 0x82
    INST_SYNC_WRITE = 0x83

    # Register definitions
    REG_OPERATING_MODE = 33
    REG_TORQUE_ENABLE = 40
    REG_GOAL_POSITION = 42
    REG_GOAL_TIME = 44
    REG_GOAL_SPEED = 46
    REG_PRESENT_POSITION = 56
    REG_PRESENT_SPEED = 58
    REG_PRESENT_LOAD = 60

    def __init__(self, port: str, baudrate: int = 1000000, timeout: float = 0.05):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )
        self._lock = threading.Lock()

    def connect(self):
        if not self._serial.is_open:
            self._serial.open()

    def disconnect(self):
        if self._serial.is_open:
            self._serial.close()

    def is_connected(self) -> bool:
        return self._serial.is_open

    def _write_packet(self, motor_id: int, instruction: int, parameters: bytes) -> None:
        """Constructs and sends a command packet."""
        length = len(parameters) + 2
        packet = bytearray([0xFF, 0xFF, motor_id, length, instruction])
        packet.extend(parameters)
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)

        with self._lock:
            self._serial.write(packet)
            self._serial.flush()

    def _read_packet(self) -> Optional[bytes]:
        """Reads a status packet."""
        with self._lock:
            # Wait for header 0xFF, 0xFF
            while True:
                header = self._serial.read(2)
                if len(header) < 2:
                    return None
                if header == b'\xff\xff':
                    break

            # Read ID, Length, Error
            id_len_err = self._serial.read(3)
            if len(id_len_err) < 3:
                return None

            motor_id, length, error = id_len_err

            # Read parameters + checksum
            params_checksum = self._serial.read(length - 1)
            if len(params_checksum) < length - 1:
                return None

            parameters = params_checksum[:-1]
            checksum_received = params_checksum[-1]

            # Verify checksum
            computed_checksum = ~(motor_id + length + error + sum(parameters)) & 0xFF
            if computed_checksum != checksum_received:
                # Checksum error
                return None

            return parameters

    def write_register(self, motor_id: int, address: int, data: bytes) -> None:
        """Write a value to a specific register."""
        params = bytearray([address])
        params.extend(data)
        self._write_packet(motor_id, self.INST_WRITE, params)
        self._read_packet() # Consume status packet

    def read_register(self, motor_id: int, address: int, length: int) -> Optional[bytes]:
        """Read a value from a specific register."""
        params = bytearray([address, length])
        self._write_packet(motor_id, self.INST_READ, params)
        return self._read_packet()

    def sync_read_present_positions(self, motor_ids: List[int]) -> Dict[int, int]:
        """Synchronously read present positions from multiple motors."""
        # For simplicity, if SYNCREAD is not fully supported, fallback to individual reads.
        positions = {}
        for mid in motor_ids:
            res = self.read_register(mid, self.REG_PRESENT_POSITION, 2)
            if res and len(res) == 2:
                positions[mid] = struct.unpack('<h', res)[0]
        return positions

    def sync_read_present_states(self, motor_ids: List[int]) -> Dict[int, Dict[str, int]]:
        """Synchronously read present position, speed, and load from multiple motors."""
        states = {}
        for mid in motor_ids:
            # Address 56 (Position), length 6 covers Pos (2), Speed (2), Load (2)
            res = self.read_register(mid, self.REG_PRESENT_POSITION, 6)
            if res and len(res) == 6:
                pos, speed, load = struct.unpack('<hhh', res)
                states[mid] = {'position': pos, 'speed': speed, 'load': load}
        return states

    def sync_write_goal_positions(self, motor_goal_map: Dict[int, int]) -> None:
        """Synchronously write goal positions to multiple motors."""
        params = bytearray([self.REG_GOAL_POSITION, 2]) # Addr, Data Length
        for mid, pos in motor_goal_map.items():
            params.append(mid)
            params.extend(struct.pack('<h', pos))

        self._write_packet(0xFE, self.INST_SYNC_WRITE, params) # Broadcast ID 0xFE

    def sync_write_goal_states(self, motor_states: Dict[int, Dict[str, int]]) -> None:
        """Synchronously write goal position, time, and speed to multiple motors."""
        # Address 42, length 6 (Pos 2B, Time 2B, Speed 2B)
        params = bytearray([self.REG_GOAL_POSITION, 6])
        for mid, state in motor_states.items():
            params.append(mid)
            pos = state.get('position', 0)
            time = state.get('time', 0)
            speed = state.get('speed', 0)
            params.extend(struct.pack('<hhh', pos, time, speed))

        self._write_packet(0xFE, self.INST_SYNC_WRITE, params)

    def enable_torque(self, motor_ids: List[int], enable: bool = True):
        for mid in motor_ids:
            self.write_register(mid, self.REG_TORQUE_ENABLE, bytes([1 if enable else 0]))
