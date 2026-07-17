"""Tests for the Feetech serial bus helper."""

import struct

from nakalab_so101_py import feetech_bus
from nakalab_so101_py.feetech_bus import FeetechBus


class FakeSerial:
    """Small pyserial fake with scripted read responses."""

    def __init__(self):
        """Initialize empty fake serial state."""
        self.read_buffer = bytearray()
        self.writes = []
        self.is_open = True
        self.timeout = None
        self.port = None
        self.on_write = None
        self.open_count = 0
        self.close_count = 0
        self.baudrate_history = []
        self._baudrate = None

    @property
    def baudrate(self):
        """Return the current fake baudrate."""
        return self._baudrate

    @baudrate.setter
    def baudrate(self, value):
        self._baudrate = value
        self.baudrate_history.append(value)

    def queue(self, data):
        """Queue bytes to be returned by read."""
        self.read_buffer.extend(data)

    def write(self, data):
        """Record a write and optionally enqueue a response."""
        packet = bytes(data)
        self.writes.append(packet)
        if self.on_write is not None:
            self.on_write(self, packet)
        return len(packet)

    def read(self, size):
        """Read queued response bytes."""
        if not self.read_buffer:
            return b''
        data = bytes(self.read_buffer[:size])
        del self.read_buffer[:size]
        return data

    def flush(self):
        """Match pyserial flush."""

    def reset_input_buffer(self):
        """Clear queued input bytes."""
        self.read_buffer.clear()

    def open(self):  # noqa: A003
        """Open the fake port."""
        self.open_count += 1
        self.is_open = True

    def close(self):
        """Close the fake port."""
        self.close_count += 1
        self.is_open = False


def install_serial(monkeypatch, fake):
    """Install one FakeSerial instance as serial.Serial."""
    def factory(port, baudrate, timeout):
        fake.port = port
        fake.baudrate = baudrate
        fake.timeout = timeout
        return fake

    monkeypatch.setattr(feetech_bus.serial, 'Serial', factory)


def status_packet(motor_id, error=0, parameters=b''):
    """Build a valid Feetech status packet."""
    length = len(parameters) + 2
    packet = bytearray([0xFF, 0xFF, motor_id, length, error])
    packet.extend(parameters)
    checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
    packet.append(checksum)
    return bytes(packet)


def decode_command(packet):
    """Decode a command packet written by FeetechBus."""
    return packet[2], packet[4], packet[5:-1]


def test_write_packet_checksum(monkeypatch):
    """Build the expected Feetech command checksum."""
    fake = FakeSerial()
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    bus._write_packet(1, FeetechBus.INST_PING, b'')

    assert fake.writes == [b'\xff\xff\x01\x02\x01\xfb']


def test_read_status_packet(monkeypatch):
    """Decode status packet id, error byte, and parameters."""
    fake = FakeSerial()
    fake.queue(status_packet(7, 3, b'\x09\x03'))
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    status = bus.read_status_packet()

    assert status.motor_id == 7
    assert status.error == 3
    assert status.parameters == b'\x09\x03'


def test_broadcast_ping_no_motors(monkeypatch):
    """Return no detections when no status packet arrives."""
    fake = FakeSerial()
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    assert bus.broadcast_ping() == {}


def test_broadcast_ping_one_motor(monkeypatch):
    """Collect one broadcast ping response with a model number."""
    fake = FakeSerial()

    def on_write(serial_obj, packet):
        motor_id, instruction, _ = decode_command(packet)
        if motor_id == FeetechBus.BROADCAST_ID:
            if instruction == FeetechBus.INST_PING:
                serial_obj.queue(status_packet(1, 0, struct.pack('<H', 777)))

    fake.on_write = on_write
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    assert bus.broadcast_ping() == {1: 777}


def test_broadcast_ping_multiple_motors(monkeypatch):
    """Collect all available broadcast ping responses."""
    fake = FakeSerial()

    def on_write(serial_obj, packet):
        motor_id, instruction, _ = decode_command(packet)
        if motor_id == FeetechBus.BROADCAST_ID:
            if instruction == FeetechBus.INST_PING:
                serial_obj.queue(status_packet(1, 0, struct.pack('<H', 777)))
                serial_obj.queue(status_packet(6, 0, b''))

    fake.on_write = on_write
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    assert bus.broadcast_ping() == {1: 777, 6: None}


def test_write_u16_uses_little_endian(monkeypatch):
    """Write unsigned 16-bit registers with little-endian payloads."""
    fake = FakeSerial()

    def on_write(serial_obj, packet):
        motor_id, instruction, _ = decode_command(packet)
        if instruction == FeetechBus.INST_WRITE:
            serial_obj.queue(status_packet(motor_id))

    fake.on_write = on_write
    install_serial(monkeypatch, fake)
    bus = FeetechBus('/dev/fake')

    bus.write_u16(1, FeetechBus.REG_MIN_POSITION_LIMIT, 0x1234)

    motor_id, instruction, parameters = decode_command(fake.writes[0])
    assert motor_id == 1
    assert instruction == FeetechBus.INST_WRITE
    assert parameters == bytes([FeetechBus.REG_MIN_POSITION_LIMIT, 0x34, 0x12])


def test_homing_offset_sign_magnitude_encoding():
    """Encode and decode signed homing offsets."""
    assert FeetechBus.encode_homing_offset(0) == 0
    assert FeetechBus.encode_homing_offset(2047) == 2047
    assert FeetechBus.encode_homing_offset(-1) == 2049
    assert FeetechBus.encode_homing_offset(-2047) == 4095

    assert FeetechBus.decode_homing_offset(0) == 0
    assert FeetechBus.decode_homing_offset(2047) == 2047
    assert FeetechBus.decode_homing_offset(2049) == -1
    assert FeetechBus.decode_homing_offset(4095) == -2047
