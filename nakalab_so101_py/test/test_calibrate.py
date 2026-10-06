"""Tests for the SO-101 calibration command."""

import json

from nakalab_so101_py.calibrate import (
    CalibrateConfig,
    MOTOR_IDS,
    SO101Calibrator,
    validate_config,
)
from nakalab_so101_py.feetech_bus import FeetechBus

import pytest


class FakeCalibrationBus:
    """Fake Feetech bus used by calibration tests."""

    def __init__(self, samples):
        """Initialize scripted position samples."""
        self.samples = list(samples)
        self.last_sample = self.samples[-1]
        self.operations = []
        self.position_reads = 0
        self.connected = False
        self.homing_offsets = {}
        self.u16_values = {}
        self.u8_values = {}

    def connect(self):
        """Record a bus connection."""
        self.operations.append(('connect',))
        self.connected = True

    def disconnect(self):
        """Record a bus disconnection."""
        self.operations.append(('disconnect',))
        self.connected = False

    def verify_motor_ids(self, motor_ids):
        """Record expected motor verification."""
        self.operations.append(('verify_motor_ids', list(motor_ids)))

    def disable_torque(self, motor_ids):
        """Record torque disable."""
        self.operations.append(('disable_torque', list(motor_ids)))

    def unlock_eeprom(self, motor_ids):
        """Record EEPROM unlock."""
        self.operations.append(('unlock_eeprom', list(motor_ids)))

    def lock_eeprom(self, motor_ids):
        """Record EEPROM lock."""
        self.operations.append(('lock_eeprom', list(motor_ids)))

    def write_u8(self, motor_id, address, value):
        """Record an unsigned 8-bit write."""
        self.operations.append(('write_u8', motor_id, address, value))
        self.u8_values[(motor_id, address)] = value
        return None

    def read_u8(self, motor_id, address):
        """Read a fake unsigned 8-bit register."""
        if address == FeetechBus.REG_PHASE:
            return self.u8_values.get((motor_id, address), 0x10)
        return self.u8_values.get((motor_id, address), 0)

    def sync_read_present_positions(self, motor_ids):
        """Return the next scripted position sample."""
        self.operations.append(('read_positions', list(motor_ids)))
        self.position_reads += 1
        if self.samples:
            self.last_sample = self.samples.pop(0)
        return {
            motor_id: self.last_sample[motor_id]
            for motor_id in motor_ids
        }

    def write_homing_offset(self, motor_id, value):
        """Record a homing offset write."""
        self.operations.append(('write_homing_offset', motor_id, value))
        self.homing_offsets[motor_id] = value
        return None

    def read_homing_offset(self, motor_id):
        """Read a written homing offset."""
        self.operations.append(('read_homing_offset', motor_id))
        return self.homing_offsets[motor_id]

    def write_u16(self, motor_id, address, value):
        """Record an unsigned 16-bit write."""
        self.operations.append(('write_u16', motor_id, address, value))
        self.u16_values[(motor_id, address)] = value
        return None

    def read_u16(self, motor_id, address):
        """Read a written unsigned 16-bit value."""
        self.operations.append(('read_u16', motor_id, address))
        return self.u16_values[(motor_id, address)]


class OffsetAwareCalibrationBus(FakeCalibrationBus):
    """実位置と EEPROM オフセットから現在位置を返す模擬サーボ．"""

    def __init__(self, homing_offsets):
        """校正前のオフセットと固定した実位置を用意する．"""
        super().__init__([{motor_id: 2047 for motor_id in MOTOR_IDS.values()}])
        self.actual_positions = dict(enumerate(
            [3000, 1300, 2700, 1000, 2500, 1900], start=1,
        ))
        self.homing_offsets = dict(homing_offsets)

    def sync_read_present_positions(self, motor_ids):
        """Feetech の補正済み位置を模擬する．"""
        self.operations.append(('read_positions', list(motor_ids)))
        self.position_reads += 1
        return {
            motor_id: (
                self.actual_positions[motor_id] - self.homing_offsets[motor_id]
            ) % FeetechBus.STS3215_RESOLUTION
            for motor_id in motor_ids
        }


@pytest.mark.parametrize('initial_offsets', [
    dict.fromkeys(MOTOR_IDS.values(), 0),
    dict(enumerate([1000, -500, 750, -1200, 500, -250], start=1)),
])
def test_recalibration_keeps_the_same_physical_pose_centered(
    tmp_path, initial_offsets,
):
    """初回・再校正とも，同じ実姿勢を中央に維持する．"""
    bus = OffsetAwareCalibrationBus(initial_offsets)
    calibrator = SO101Calibrator(
        CalibrateConfig('/dev/fake', 'so101_follower', 'repeat', overwrite=True),
        bus_factory=lambda *_, **__: bus,
        input_func=lambda _: '',
        output=lambda _: None,
        sleep_func=lambda _: None,
        calibration_root=tmp_path,
        range_stop_checker=lambda: True,
        display_ranges=False,
    )

    for _ in range(2):
        calibrator.run()
        positions = bus.sync_read_present_positions(list(MOTOR_IDS.values()))
        assert positions == dict.fromkeys(MOTOR_IDS.values(), 2047)
        assert bus.homing_offsets == {
            motor_id: position - 2047
            for motor_id, position in bus.actual_positions.items()
        }
        assert not bus.connected


@pytest.mark.parametrize('failure', ['reset', 'offset', 'feedback', 'missing'])
def test_invalid_homing_aborts_without_saving(tmp_path, monkeypatch, failure):
    """書き込み・読み戻し・基準位置の異常時は JSON を保存しない．"""
    bus = OffsetAwareCalibrationBus(dict.fromkeys(MOTOR_IDS.values(), 500))
    write_offset = bus.write_homing_offset
    read_positions = bus.sync_read_present_positions

    def faulty_write(motor_id, value):
        if failure == 'reset' and value == 0:
            return None
        if failure == 'offset' and value != 0:
            return None
        return write_offset(motor_id, value)

    def faulty_read(motor_ids):
        if failure == 'feedback':
            return {mid: bus.actual_positions[mid] for mid in motor_ids}
        positions = read_positions(motor_ids)
        if failure == 'missing':
            positions.pop(1)
        return positions

    monkeypatch.setattr(bus, 'write_homing_offset', faulty_write)
    monkeypatch.setattr(bus, 'sync_read_present_positions', faulty_read)
    calibrator = SO101Calibrator(
        CalibrateConfig('/dev/fake', 'so101_follower', 'invalid'),
        bus_factory=lambda *_, **__: bus,
        input_func=lambda _: '',
        output=lambda _: None,
        calibration_root=tmp_path,
        range_stop_checker=lambda: True,
        display_ranges=False,
    )

    with pytest.raises(RuntimeError):
        calibrator.run()

    assert not calibrator.calibration_path().exists()
    assert ('lock_eeprom', list(MOTOR_IDS.values())) in bus.operations
    assert not bus.connected


def sample(**positions):
    """Build a complete position sample keyed by motor id."""
    return {
        motor_id: positions[name]
        for name, motor_id in MOTOR_IDS.items()
    }


def first_index(operations, operation_name):
    """Return the first index for an operation name."""
    for index, operation in enumerate(operations):
        if operation[0] == operation_name:
            return index
    raise AssertionError(f'{operation_name} not found')


@pytest.mark.parametrize(
    'config, expected_message',
    [
        (
            CalibrateConfig('', 'so101_follower', 'arm'),
            "Parameter 'device' must be set.",
        ),
        (
            CalibrateConfig('/dev/fake', 'bad_arm', 'arm'),
            "Parameter 'arm_type' must be one of:",
        ),
        (
            CalibrateConfig('/dev/fake', 'so101_follower', ''),
            "Parameter 'calibration_id' must be set.",
        ),
        (
            CalibrateConfig('/dev/fake', 'so101_follower', '../arm'),
            "Parameter 'calibration_id' must be a file stem.",
        ),
        (
            CalibrateConfig('/dev/fake', 'so101_follower', 'arm', 9600),
            'Unsupported Feetech baudrate: 9600',
        ),
    ],
)
def test_validate_config_rejects_invalid_parameters(
    config,
    expected_message,
):
    """Reject invalid calibration parameters."""
    with pytest.raises(ValueError, match=expected_message):
        validate_config(config)


def test_calibration_path_and_overwrite_confirmation(tmp_path):
    """Resolve calibration paths and confirm overwrites."""
    config = CalibrateConfig('/dev/fake', 'so101_follower', 'arm_a')
    calibrator = SO101Calibrator(config, calibration_root=tmp_path)
    output_path = tmp_path / 'so101_follower' / 'arm_a.json'

    assert calibrator.calibration_path() == output_path

    output_path.parent.mkdir(parents=True)
    output_path.write_text('{}\n', encoding='utf-8')

    refusing_calibrator = SO101Calibrator(
        config,
        calibration_root=tmp_path,
        input_func=lambda _: 'n',
    )
    with pytest.raises(RuntimeError, match='avoid overwrite'):
        refusing_calibrator.confirm_overwrite(output_path)

    accepting_calibrator = SO101Calibrator(
        config,
        calibration_root=tmp_path,
        input_func=lambda _: 'yes',
    )
    accepting_calibrator.confirm_overwrite(output_path)


def test_compute_homing_offsets_from_center_pose():
    """Compute homing offsets from the middle pose."""
    offsets = SO101Calibrator.compute_homing_offsets({
        'shoulder_pan': 2047,
        'shoulder_lift': 2050,
        'elbow_flex': 2000,
        'wrist_flex': 2048,
        'wrist_roll': 2100,
        'gripper': 1900,
    })

    assert offsets == {
        'shoulder_pan': 0,
        'shoulder_lift': 3,
        'elbow_flex': -47,
        'wrist_flex': 1,
        'wrist_roll': 53,
        'gripper': -147,
    }


def test_record_ranges_of_motion_updates_min_max():
    """Update range minima and maxima from scripted samples."""
    fake_bus = FakeCalibrationBus([
        sample(
            shoulder_pan=1000,
            shoulder_lift=1100,
            elbow_flex=1200,
            wrist_flex=1300,
            wrist_roll=1400,
            gripper=1500,
        ),
        sample(
            shoulder_pan=3000,
            shoulder_lift=3100,
            elbow_flex=3200,
            wrist_flex=3300,
            wrist_roll=3400,
            gripper=3500,
        ),
        sample(
            shoulder_pan=2000,
            shoulder_lift=1000,
            elbow_flex=3400,
            wrist_flex=1250,
            wrist_roll=3600,
            gripper=1450,
        ),
    ])
    calibrator = SO101Calibrator(
        CalibrateConfig('/dev/fake', 'so101_follower', 'arm'),
        sleep_func=lambda _: None,
    )

    ranges = calibrator.record_ranges_of_motion(
        fake_bus,
        lambda: fake_bus.position_reads >= 3,
        display_ranges=False,
    )

    assert ranges['shoulder_pan'] == {'min': 1000, 'max': 3000}
    assert ranges['shoulder_lift'] == {'min': 1000, 'max': 3100}
    assert ranges['elbow_flex'] == {'min': 1200, 'max': 3400}
    assert ranges['wrist_flex'] == {'min': 1250, 'max': 3300}
    assert ranges['wrist_roll'] == {'min': 1400, 'max': 3600}
    assert ranges['gripper'] == {'min': 1450, 'max': 3500}


def test_run_writes_eeprom_in_order_and_saves_json(tmp_path):
    """Run calibration and save a LeRobot-style JSON file."""
    fake_bus = FakeCalibrationBus([
        sample(
            shoulder_pan=2047,
            shoulder_lift=2050,
            elbow_flex=2000,
            wrist_flex=2048,
            wrist_roll=2100,
            gripper=1900,
        ),
        dict.fromkeys(MOTOR_IDS.values(), 2047),
        sample(
            shoulder_pan=1000,
            shoulder_lift=1100,
            elbow_flex=1200,
            wrist_flex=1300,
            wrist_roll=1400,
            gripper=1500,
        ),
        sample(
            shoulder_pan=3000,
            shoulder_lift=3100,
            elbow_flex=3200,
            wrist_flex=3300,
            wrist_roll=3400,
            gripper=3500,
        ),
        sample(
            shoulder_pan=2000,
            shoulder_lift=1000,
            elbow_flex=3400,
            wrist_flex=1250,
            wrist_roll=3600,
            gripper=1450,
        ),
    ])
    config = CalibrateConfig(
        '/dev/fake',
        'so101_follower',
        'arm_a',
        overwrite=True,
    )
    calibrator = SO101Calibrator(
        config,
        bus_factory=lambda *_, **__: fake_bus,
        input_func=lambda _: '',
        output=lambda _: None,
        sleep_func=lambda _: None,
        calibration_root=tmp_path,
        range_stop_checker=lambda: fake_bus.position_reads >= 5,
        display_ranges=False,
    )

    calibrator.run()

    operations = fake_bus.operations
    assert first_index(operations, 'disable_torque') < first_index(
        operations,
        'write_homing_offset',
    )
    assert first_index(operations, 'write_homing_offset') < first_index(
        operations,
        'write_u16',
    )
    assert first_index(operations, 'write_u16') < first_index(
        operations,
        'lock_eeprom',
    )
    assert first_index(operations, 'lock_eeprom') < max(
        index for index, operation in enumerate(operations)
        if operation[0] == 'read_homing_offset'
    )

    output_path = tmp_path / 'so101_follower' / 'arm_a.json'
    data = json.loads(output_path.read_text(encoding='utf-8'))
    assert data['shoulder_pan'] == {
        'id': 1,
        'drive_mode': 0,
        'homing_offset': 0,
        'range_min': 1000,
        'range_max': 3000,
    }
    assert data['gripper'] == {
        'id': 6,
        'drive_mode': 0,
        'homing_offset': -147,
        'range_min': 1450,
        'range_max': 3500,
    }
