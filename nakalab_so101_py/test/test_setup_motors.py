"""Tests for the SO-101 motor setup command."""

import struct

from nakalab_so101_py.feetech_bus import FeetechBus
from nakalab_so101_py.setup_motors import (
    MotorDetection,
    MotorSetup,
    SetupConfig,
    validate_config,
)

import pytest

from test_feetech_bus import decode_command, FakeSerial, install_serial
from test_feetech_bus import status_packet


def test_setup_single_motor_writes_id_baudrate_and_verifies(monkeypatch):
    """Write lock, ID, baudrate, lock again, and verify registers."""
    fake = FakeSerial()
    target_id = 6
    target_baudrate = 1000000
    target_baudrate_raw = FeetechBus.baudrate_to_register(target_baudrate)

    def on_write(serial_obj, packet):
        motor_id, instruction, parameters = decode_command(packet)

        if instruction == FeetechBus.INST_WRITE:
            serial_obj.queue(status_packet(motor_id))
            return

        if instruction != FeetechBus.INST_READ:
            return

        address = parameters[0]
        length = parameters[1]
        if address == FeetechBus.REG_ID:
            payload = bytes([target_id])
        elif address == FeetechBus.REG_BAUD_RATE:
            payload = bytes([target_baudrate_raw])
        elif address == FeetechBus.REG_MODEL:
            payload = struct.pack('<H', 777)
        else:
            payload = bytes(length)
        serial_obj.queue(status_packet(motor_id, parameters=payload[:length]))

    fake.on_write = on_write
    install_serial(monkeypatch, fake)
    outputs = []
    setup = MotorSetup(
        SetupConfig('/dev/fake', baudrate=target_baudrate),
        output=outputs.append,
        sleep_func=lambda _: None,
    )

    setup.setup_single_motor(
        MotorDetection(baudrate=500000, motor_id=1, model_number=None),
        target_id,
    )

    write_ops = []
    read_ops = []
    for packet in fake.writes:
        motor_id, instruction, parameters = decode_command(packet)
        if instruction == FeetechBus.INST_WRITE:
            write_ops.append((motor_id, parameters[0], list(parameters[1:])))
        if instruction == FeetechBus.INST_READ:
            read_ops.append((motor_id, parameters[0], parameters[1]))

    assert write_ops[:4] == [
        (1, FeetechBus.REG_LOCK, [0]),
        (1, FeetechBus.REG_ID, [target_id]),
        (target_id, FeetechBus.REG_BAUD_RATE, [target_baudrate_raw]),
        (target_id, FeetechBus.REG_LOCK, [1]),
    ]
    assert read_ops == [
        (target_id, FeetechBus.REG_ID, 1),
        (target_id, FeetechBus.REG_BAUD_RATE, 1),
        (target_id, FeetechBus.REG_MODEL, 2),
    ]
    assert fake.baudrate_history == [500000, target_baudrate]
    assert fake.close_count >= 1
    assert outputs == ['Verified motor 6 (model 777).']


@pytest.mark.parametrize(
    'config, expected_message',
    [
        (SetupConfig(''), "Parameter 'device' must be set."),
        (
            SetupConfig('/dev/fake', arm_type='bad_arm'),
            "Parameter 'arm_type' must be one of:",
        ),
        (
            SetupConfig('/dev/fake', baudrate=9600),
            'Unsupported Feetech baudrate: 9600',
        ),
    ],
)
def test_validate_config_rejects_invalid_parameters(
    config,
    expected_message,
):
    """Reject missing device, invalid arm type, and unsupported baudrate."""
    with pytest.raises(ValueError, match=expected_message):
        validate_config(config)
