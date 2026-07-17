"""Serial bus helpers for Feetech STS3215 servos."""

from dataclasses import dataclass
import struct
import threading
import time
from typing import Dict, List, Optional

import serial


@dataclass(frozen=True)
class FeetechStatusPacket:
    """Decoded Feetech status packet."""

    motor_id: int
    error: int
    parameters: bytes


class FeetechBus:
    """
    Feetech STS3215 serial bus driver.

    Implemented from scratch based on reverse engineering of standard
    Feetech protocols.
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
    REG_MODEL = 3
    REG_ID = 5
    REG_BAUD_RATE = 6
    REG_MIN_POSITION_LIMIT = 9
    REG_MAX_POSITION_LIMIT = 11
    REG_PHASE = 18
    REG_HOMING_OFFSET = 31
    REG_OPERATING_MODE = 33
    REG_TORQUE_ENABLE = 40
    REG_GOAL_POSITION = 42
    REG_GOAL_TIME = 44
    REG_GOAL_SPEED = 46
    REG_LOCK = 55
    REG_PRESENT_POSITION = 56
    REG_PRESENT_SPEED = 58
    REG_PRESENT_LOAD = 60

    BROADCAST_ID = 0xFE
    HOMING_OFFSET_SIGN_BIT = 11
    STS3215_RESOLUTION = 4096

    BAUDRATE_TO_INDEX = {
        1000000: 0,
        500000: 1,
        250000: 2,
        128000: 3,
        115200: 4,
        57600: 5,
        38400: 6,
        19200: 7,
    }
    INDEX_TO_BAUDRATE = {
        value: key for key, value in BAUDRATE_TO_INDEX.items()
    }

    def __init__(
        self,
        port: str,
        baudrate: int = 1000000,
        timeout: float = 0.05,
    ):
        """Initialize the bus and open the serial port."""
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
        """Open the serial port if it is not already open."""
        if not self._serial.is_open:
            self._serial.open()

    def disconnect(self):
        """Close the serial port if it is open."""
        if self._serial.is_open:
            self._serial.close()

    def is_connected(self) -> bool:
        """Return whether the serial port is open."""
        return self._serial.is_open

    def set_baudrate(self, baudrate: int) -> None:
        """Update the serial baudrate used by this bus."""
        self._baudrate = baudrate
        self._serial.baudrate = baudrate

    @classmethod
    def baudrate_to_register(cls, baudrate: int) -> int:
        """Return the STS register value for a baudrate."""
        if baudrate not in cls.BAUDRATE_TO_INDEX:
            raise ValueError(f'Unsupported Feetech baudrate: {baudrate}')
        return cls.BAUDRATE_TO_INDEX[baudrate]

    @classmethod
    def register_to_baudrate(cls, value: int) -> Optional[int]:
        """Return the baudrate represented by a STS baudrate register."""
        return cls.INDEX_TO_BAUDRATE.get(value)

    def clear_input_buffer(self) -> None:
        """Discard stale bytes from the serial input buffer when supported."""
        reset_input_buffer = getattr(self._serial, 'reset_input_buffer', None)
        if reset_input_buffer is None:
            return
        with self._lock:
            reset_input_buffer()

    def _write_packet(
        self,
        motor_id: int,
        instruction: int,
        parameters: bytes,
    ) -> None:
        """Construct and send a command packet."""
        length = len(parameters) + 2
        packet = bytearray([0xFF, 0xFF, motor_id, length, instruction])
        packet.extend(parameters)
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)

        with self._lock:
            self._serial.write(packet)
            self._serial.flush()

    def _read_packet(self) -> Optional[bytes]:
        """Read a status packet."""
        status = self.read_status_packet()
        if status is None:
            return None
        return status.parameters

    def read_status_packet(self) -> Optional[FeetechStatusPacket]:
        """Read a status packet including motor id and error byte."""
        with self._lock:
            # Wait for header 0xFF, 0xFF
            header_count = 0
            while True:
                byte = self._serial.read(1)
                if len(byte) < 1:
                    return None
                if byte[0] == 0xFF:
                    header_count += 1
                else:
                    header_count = 0
                if header_count == 2:
                    break

            # Read ID, Length, Error
            id_len_err = self._serial.read(3)
            if len(id_len_err) < 3:
                return None

            motor_id, length, error = id_len_err
            if length < 2:
                return None

            # Read parameters + checksum
            params_checksum = self._serial.read(length - 1)
            if len(params_checksum) < length - 1:
                return None

            parameters = bytes(params_checksum[:-1])
            checksum_received = params_checksum[-1]

            # Verify checksum
            checksum_sum = motor_id + length + error + sum(parameters)
            computed_checksum = ~(checksum_sum & 0xFF) & 0xFF
            if computed_checksum != checksum_received:
                # Checksum error
                return None

            return FeetechStatusPacket(
                motor_id=motor_id,
                error=error,
                parameters=parameters,
            )

    def write_register(
        self,
        motor_id: int,
        address: int,
        data: bytes,
    ) -> Optional[FeetechStatusPacket]:
        """Write a value to a specific register."""
        params = bytearray([address])
        params.extend(data)
        self._write_packet(motor_id, self.INST_WRITE, params)
        return self.read_status_packet()

    def read_register(
        self,
        motor_id: int,
        address: int,
        length: int,
    ) -> Optional[bytes]:
        """Read a value from a specific register."""
        status = self.read_register_status(motor_id, address, length)
        if status is None or status.error != 0:
            return None
        return status.parameters

    def read_register_status(
        self,
        motor_id: int,
        address: int,
        length: int,
    ) -> Optional[FeetechStatusPacket]:
        """Read a register and return the complete status packet."""
        params = bytes([address, length])
        self._write_packet(motor_id, self.INST_READ, params)
        return self.read_status_packet()

    def read_u8(self, motor_id: int, address: int) -> Optional[int]:
        """Read one byte from a register."""
        data = self.read_register(motor_id, address, 1)
        if data is None or len(data) != 1:
            return None
        return data[0]

    def read_u16(self, motor_id: int, address: int) -> Optional[int]:
        """Read an unsigned little-endian 16-bit register."""
        data = self.read_register(motor_id, address, 2)
        if data is None or len(data) != 2:
            return None
        return struct.unpack('<H', data)[0]

    def write_u8(
        self,
        motor_id: int,
        address: int,
        value: int,
    ) -> Optional[FeetechStatusPacket]:
        """Write one byte to a register."""
        return self.write_register(motor_id, address, bytes([value & 0xFF]))

    def write_u16(
        self,
        motor_id: int,
        address: int,
        value: int,
    ) -> Optional[FeetechStatusPacket]:
        """Write an unsigned little-endian 16-bit register."""
        data = struct.pack('<H', value & 0xFFFF)
        return self.write_register(motor_id, address, data)

    @classmethod
    def encode_homing_offset(cls, value: int) -> int:
        """Encode a signed homing offset as Feetech sign-magnitude."""
        sign_bit = 1 << cls.HOMING_OFFSET_SIGN_BIT
        magnitude_mask = sign_bit - 1
        magnitude = abs(int(value))
        if magnitude > magnitude_mask:
            raise ValueError(
                f'Homing offset {value} exceeds sign-magnitude range '
                f'+/-{magnitude_mask}'
            )
        if value < 0:
            return sign_bit | magnitude
        return magnitude

    @classmethod
    def decode_homing_offset(cls, value: int) -> int:
        """Decode a Feetech sign-magnitude homing offset."""
        sign_bit = 1 << cls.HOMING_OFFSET_SIGN_BIT
        magnitude_mask = sign_bit - 1
        magnitude = value & magnitude_mask
        if value & sign_bit:
            return -magnitude
        return magnitude

    def read_homing_offset(self, motor_id: int) -> Optional[int]:
        """Read a signed homing offset from one motor."""
        raw_value = self.read_u16(motor_id, self.REG_HOMING_OFFSET)
        if raw_value is None:
            return None
        return self.decode_homing_offset(raw_value)

    def write_homing_offset(
        self,
        motor_id: int,
        value: int,
    ) -> Optional[FeetechStatusPacket]:
        """Write a signed homing offset to one motor."""
        raw_value = self.encode_homing_offset(value)
        return self.write_u16(motor_id, self.REG_HOMING_OFFSET, raw_value)

    def ping(self, motor_id: int) -> Optional[FeetechStatusPacket]:
        """Ping one motor ID and return its status packet."""
        self._write_packet(motor_id, self.INST_PING, b'')
        return self.read_status_packet()

    def broadcast_ping(
        self,
        read_timeout: float = 0.2,
    ) -> Dict[int, Optional[int]]:
        """Ping the broadcast ID and collect responding motor IDs."""
        found: Dict[int, Optional[int]] = {}
        self.clear_input_buffer()
        self._write_packet(self.BROADCAST_ID, self.INST_PING, b'')

        deadline = time.monotonic() + read_timeout
        while time.monotonic() < deadline:
            status = self.read_status_packet()
            if status is None:
                break
            found[status.motor_id] = self._model_number_from_ping(status)
        return found

    def ping_motor_ids(self, motor_ids: List[int]) -> Dict[int, Optional[int]]:
        """Ping selected motors and return responding model numbers."""
        found: Dict[int, Optional[int]] = {}
        for motor_id in motor_ids:
            status = self.ping(motor_id)
            if status is not None and status.error == 0:
                found[motor_id] = self._model_number_from_ping(status)
        return found

    def verify_motor_ids(self, motor_ids: List[int]) -> None:
        """Raise if any expected motor ID does not respond."""
        found = self.ping_motor_ids(motor_ids)
        missing = [motor_id for motor_id in motor_ids if motor_id not in found]
        if missing:
            raise RuntimeError(f'Missing motor IDs: {missing}')

    @staticmethod
    def _model_number_from_ping(
        status: FeetechStatusPacket,
    ) -> Optional[int]:
        if len(status.parameters) < 2:
            return None
        return struct.unpack('<H', status.parameters[:2])[0]

    def sync_read_present_positions(
        self,
        motor_ids: List[int],
    ) -> Dict[int, int]:
        """Read present positions from multiple motors."""
        # Fall back to individual reads when SYNC_READ is unavailable.
        positions = {}
        for mid in motor_ids:
            res = self.read_register(mid, self.REG_PRESENT_POSITION, 2)
            if res and len(res) == 2:
                positions[mid] = struct.unpack('<h', res)[0]
        return positions

    def sync_read_present_states(
        self,
        motor_ids: List[int],
    ) -> Dict[int, Dict[str, int]]:
        """Read present position, speed, and load from multiple motors."""
        states = {}
        for mid in motor_ids:
            # Address 56 covers position, speed, and load.
            res = self.read_register(mid, self.REG_PRESENT_POSITION, 6)
            if res and len(res) == 6:
                pos, speed, load = struct.unpack('<hhh', res)
                states[mid] = {'position': pos, 'speed': speed, 'load': load}
        return states

    def sync_write_goal_positions(
        self,
        motor_goal_map: Dict[int, int],
    ) -> None:
        """Write goal positions to multiple motors."""
        params = bytearray([self.REG_GOAL_POSITION, 2])
        for mid, pos in motor_goal_map.items():
            params.append(mid)
            params.extend(struct.pack('<h', pos))

        self._write_packet(0xFE, self.INST_SYNC_WRITE, params)

    def sync_write_goal_states(
        self,
        motor_states: Dict[int, Dict[str, int]],
    ) -> None:
        """Write goal position, time, and speed to multiple motors."""
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
        """Enable or disable torque for the given motor IDs."""
        for mid in motor_ids:
            value = 1 if enable else 0
            self.write_register(mid, self.REG_TORQUE_ENABLE, bytes([value]))

    def disable_torque(self, motor_ids: List[int]) -> None:
        """Disable torque for the given motor IDs."""
        self.enable_torque(motor_ids, False)

    def unlock_eeprom(self, motor_ids: List[int]) -> None:
        """Unlock EEPROM writes for the given motor IDs."""
        for motor_id in motor_ids:
            self.write_u8(motor_id, self.REG_LOCK, 0)

    def lock_eeprom(self, motor_ids: List[int]) -> None:
        """Lock EEPROM writes for the given motor IDs."""
        for motor_id in motor_ids:
            self.write_u8(motor_id, self.REG_LOCK, 1)
