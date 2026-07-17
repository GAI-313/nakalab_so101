"""Interactive ROS2 command for calibrating SO-101 Feetech motors."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import threading
import time
from typing import Callable, Dict, Optional, Sequence

from nakalab_so101_py.feetech_bus import FeetechBus


VALID_ARM_TYPES = ('so101_leader', 'so101_follower')
MOTOR_IDS = {
    'shoulder_pan': 1,
    'shoulder_lift': 2,
    'elbow_flex': 3,
    'wrist_flex': 4,
    'wrist_roll': 5,
    'gripper': 6,
}
HALF_TURN_POSITION = 2047
POSITION_MODE = 0
PHASE_FEEDBACK_BIT = 0x10


@dataclass(frozen=True)
class CalibrateConfig:
    """Configuration for a calibration run."""

    device: str
    arm_type: str
    calibration_id: str
    baudrate: int = 1000000
    overwrite: bool = False


@dataclass(frozen=True)
class MotorCalibration:
    """Calibration values for one motor."""

    motor_id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    def to_dict(self) -> Dict[str, int]:
        """Return a LeRobot-style calibration dictionary."""
        return {
            'id': self.motor_id,
            'drive_mode': self.drive_mode,
            'homing_offset': self.homing_offset,
            'range_min': self.range_min,
            'range_max': self.range_max,
        }


def validate_config(config: CalibrateConfig) -> None:
    """Validate user-provided calibration parameters."""
    if not config.device:
        raise ValueError("Parameter 'device' must be set.")

    if config.arm_type not in VALID_ARM_TYPES:
        valid_types = ', '.join(VALID_ARM_TYPES)
        raise ValueError(
            f"Parameter 'arm_type' must be one of: {valid_types}"
        )

    if not config.calibration_id:
        raise ValueError("Parameter 'calibration_id' must be set.")

    calibration_id_path = Path(config.calibration_id)
    separators = [os.sep]
    if os.altsep:
        separators.append(os.altsep)
    has_separator = any(
        separator in config.calibration_id for separator in separators
    )
    if (
        calibration_id_path.name != config.calibration_id or
        config.calibration_id in ('.', '..') or
        has_separator
    ):
        raise ValueError("Parameter 'calibration_id' must be a file stem.")

    FeetechBus.baudrate_to_register(config.baudrate)


def default_calibration_root() -> Path:
    """Return the preferred calibration directory."""
    candidates = []
    pixi_root = os.environ.get('PIXI_PROJECT_ROOT')
    if pixi_root:
        candidates.append(
            Path(pixi_root) / 'nakalab_so101_description' / 'calibration'
        )

    workspace_root = Path(__file__).resolve().parents[2]
    candidates.append(
        workspace_root / 'nakalab_so101_description' / 'calibration'
    )

    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = Path(
            get_package_share_directory('nakalab_so101_description')
        )
        candidates.append(package_share / 'calibration')
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


class SO101Calibrator:
    """Calibrate SO-101 motor offsets and range limits."""

    def __init__(
        self,
        config: CalibrateConfig,
        bus_factory: Callable[..., FeetechBus] = FeetechBus,
        input_func: Callable[[str], str] = input,
        output: Callable[[str], None] = print,
        sleep_func: Callable[[float], None] = time.sleep,
        calibration_root: Optional[Path] = None,
        range_stop_checker: Optional[Callable[[], bool]] = None,
        display_ranges: bool = True,
    ):
        """Initialize calibration dependencies."""
        self.config = config
        self.bus_factory = bus_factory
        self.input_func = input_func
        self.output = output
        self.sleep_func = sleep_func
        self.calibration_root = calibration_root
        self.range_stop_checker = range_stop_checker
        self.display_ranges = display_ranges

    def run(self) -> None:
        """Run the full interactive calibration sequence."""
        validate_config(self.config)
        output_path = self.calibration_path()
        self.confirm_overwrite(output_path)

        motor_ids = list(MOTOR_IDS.values())
        bus = self.bus_factory(
            self.config.device,
            baudrate=self.config.baudrate,
        )
        bus.connect()
        locked = False

        try:
            bus.verify_motor_ids(motor_ids)
            self.prepare_motors(bus, motor_ids)
            self.input_func(
                'Move the arm to the middle of its range of motion, '
                'then press Enter.'
            )
            center_positions = self.read_required_positions(bus)
            homing_offsets = self.compute_homing_offsets(center_positions)
            self.write_homing_offsets(bus, homing_offsets)

            self.output(
                'Move each joint through its full range of motion. '
                'Press Enter when finished.'
            )
            ranges = self.record_ranges_of_motion(
                bus,
                self._range_stop_checker(),
                self.display_ranges,
            )

            calibration = self.build_calibration(homing_offsets, ranges)
            self.write_range_limits(bus, calibration)
            bus.lock_eeprom(motor_ids)
            locked = True
            self.verify_calibration(bus, calibration)
            self.save_calibration(output_path, calibration)
            self.output(f'Saved calibration to {output_path}')
        finally:
            if not locked:
                try:
                    bus.lock_eeprom(motor_ids)
                except Exception:
                    pass
            bus.disconnect()

    def calibration_path(self) -> Path:
        """Return the output JSON path for the calibration run."""
        root = self.calibration_root
        if root is None:
            root = default_calibration_root()
        return root / self.config.arm_type / f'{self.config.calibration_id}.json'

    def confirm_overwrite(self, output_path: Path) -> None:
        """Confirm overwriting an existing calibration file."""
        if self.config.overwrite or not output_path.exists():
            return

        answer = self.input_func(
            f'Calibration file exists at {output_path}. Overwrite? [y/N] '
        )
        if answer.strip().lower() not in ('y', 'yes'):
            raise RuntimeError('Calibration cancelled to avoid overwrite.')

    def prepare_motors(self, bus: FeetechBus, motor_ids: Sequence[int]) -> None:
        """Prepare motors for EEPROM calibration writes."""
        bus.disable_torque(list(motor_ids))
        bus.unlock_eeprom(list(motor_ids))

        for motor_id in motor_ids:
            self._write_u8(
                bus,
                motor_id,
                FeetechBus.REG_OPERATING_MODE,
                POSITION_MODE,
                'write operating mode',
            )
            phase = bus.read_u8(motor_id, FeetechBus.REG_PHASE)
            if phase is not None:
                phase_without_feedback = phase & ~PHASE_FEEDBACK_BIT
                if phase_without_feedback != phase:
                    self._write_u8(
                        bus,
                        motor_id,
                        FeetechBus.REG_PHASE,
                        phase_without_feedback,
                        'clear phase feedback bit',
                    )

    def read_required_positions(self, bus: FeetechBus) -> Dict[str, int]:
        """Read all expected motor positions by joint name."""
        positions_by_id = bus.sync_read_present_positions(
            list(MOTOR_IDS.values())
        )
        missing = [
            name for name, motor_id in MOTOR_IDS.items()
            if motor_id not in positions_by_id
        ]
        if missing:
            raise RuntimeError(f'Missing position reads for joints: {missing}')

        return {
            name: int(positions_by_id[motor_id])
            for name, motor_id in MOTOR_IDS.items()
        }

    @staticmethod
    def compute_homing_offsets(
        center_positions: Dict[str, int],
    ) -> Dict[str, int]:
        """Compute homing offsets from middle-pose positions."""
        return {
            name: int(position) - HALF_TURN_POSITION
            for name, position in center_positions.items()
        }

    def write_homing_offsets(
        self,
        bus: FeetechBus,
        homing_offsets: Dict[str, int],
    ) -> None:
        """Write computed homing offsets to motor EEPROM."""
        for name, homing_offset in homing_offsets.items():
            motor_id = MOTOR_IDS[name]
            status = bus.write_homing_offset(motor_id, homing_offset)
            self._check_status(status, motor_id, 'write homing offset')

    def record_ranges_of_motion(
        self,
        bus: FeetechBus,
        stop_checker: Callable[[], bool],
        display_ranges: bool = True,
    ) -> Dict[str, Dict[str, int]]:
        """Record minimum and maximum observed positions for each joint."""
        ranges = {
            name: {'min': None, 'max': None}
            for name in MOTOR_IDS
        }
        last_display_time = 0.0

        while True:
            positions = self.read_required_positions(bus)
            for name, position in positions.items():
                current_range = ranges[name]
                if current_range['min'] is None:
                    current_range['min'] = position
                if current_range['max'] is None:
                    current_range['max'] = position
                current_range['min'] = min(current_range['min'], position)
                current_range['max'] = max(current_range['max'], position)

            now = time.monotonic()
            if display_ranges and now - last_display_time >= 0.25:
                self.output(self.format_range_table(positions, ranges))
                last_display_time = now

            if stop_checker():
                break
            self.sleep_func(0.05)

        return {
            name: {
                'min': int(values['min']),
                'max': int(values['max']),
            }
            for name, values in ranges.items()
        }

    @staticmethod
    def format_range_table(
        positions: Dict[str, int],
        ranges: Dict[str, Dict[str, Optional[int]]],
    ) -> str:
        """Format the live min/position/max table."""
        lines = ['joint             MIN | POS | MAX']
        for name in MOTOR_IDS:
            values = ranges[name]
            min_value = values['min']
            max_value = values['max']
            lines.append(
                f'{name:<16} {min_value:>4} | '
                f'{positions[name]:>3} | {max_value:>3}'
            )
        return '\n'.join(lines)

    def build_calibration(
        self,
        homing_offsets: Dict[str, int],
        ranges: Dict[str, Dict[str, int]],
    ) -> Dict[str, MotorCalibration]:
        """Build motor calibration objects."""
        calibration = {}
        for name, motor_id in MOTOR_IDS.items():
            calibration[name] = MotorCalibration(
                motor_id=motor_id,
                drive_mode=0,
                homing_offset=homing_offsets[name],
                range_min=ranges[name]['min'],
                range_max=ranges[name]['max'],
            )
        return calibration

    def write_range_limits(
        self,
        bus: FeetechBus,
        calibration: Dict[str, MotorCalibration],
    ) -> None:
        """Write range limits to motor EEPROM."""
        for values in calibration.values():
            status = bus.write_u16(
                values.motor_id,
                FeetechBus.REG_MIN_POSITION_LIMIT,
                values.range_min,
            )
            self._check_status(status, values.motor_id, 'write range min')
            status = bus.write_u16(
                values.motor_id,
                FeetechBus.REG_MAX_POSITION_LIMIT,
                values.range_max,
            )
            self._check_status(status, values.motor_id, 'write range max')

    def verify_calibration(
        self,
        bus: FeetechBus,
        calibration: Dict[str, MotorCalibration],
    ) -> None:
        """Verify calibration values can be read back."""
        for name, values in calibration.items():
            homing_offset = bus.read_homing_offset(values.motor_id)
            if homing_offset != values.homing_offset:
                raise RuntimeError(
                    f'{name}: expected homing offset '
                    f'{values.homing_offset}, read {homing_offset}.'
                )

            range_min = bus.read_u16(
                values.motor_id,
                FeetechBus.REG_MIN_POSITION_LIMIT,
            )
            range_max = bus.read_u16(
                values.motor_id,
                FeetechBus.REG_MAX_POSITION_LIMIT,
            )
            if range_min != values.range_min or range_max != values.range_max:
                raise RuntimeError(
                    f'{name}: expected range '
                    f'{values.range_min}-{values.range_max}, '
                    f'read {range_min}-{range_max}.'
                )

    @staticmethod
    def save_calibration(
        output_path: Path,
        calibration: Dict[str, MotorCalibration],
    ) -> None:
        """Save calibration values as JSON."""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            name: values.to_dict()
            for name, values in calibration.items()
        }
        with output_path.open('w', encoding='utf-8') as file:
            json.dump(data, file, indent=4)
            file.write('\n')

    @staticmethod
    def _write_u8(
        bus: FeetechBus,
        motor_id: int,
        address: int,
        value: int,
        label: str,
    ) -> None:
        status = bus.write_u8(motor_id, address, value)
        SO101Calibrator._check_status(status, motor_id, label)

    @staticmethod
    def _check_status(status, motor_id: int, label: str) -> None:
        if status is not None and status.error != 0:
            raise RuntimeError(
                f'{label} failed for motor {motor_id}: '
                f'0x{status.error:02x}'
            )

    def _range_stop_checker(self) -> Callable[[], bool]:
        if self.range_stop_checker is not None:
            return self.range_stop_checker

        stop_event = threading.Event()

        def wait_for_enter():
            self.input_func('Press Enter to stop range recording.')
            stop_event.set()

        thread = threading.Thread(target=wait_for_enter, daemon=True)
        thread.start()
        return stop_event.is_set


def _config_from_node(node) -> CalibrateConfig:
    node.declare_parameter('device', '')
    node.declare_parameter('arm_type', '')
    node.declare_parameter('calibration_id', '')
    node.declare_parameter('baudrate', 1000000)
    node.declare_parameter('overwrite', False)

    return CalibrateConfig(
        device=node.get_parameter('device').value,
        arm_type=node.get_parameter('arm_type').value,
        calibration_id=node.get_parameter('calibration_id').value,
        baudrate=node.get_parameter('baudrate').value,
        overwrite=node.get_parameter('overwrite').value,
    )


def main(args=None):
    """Run the ROS2 console script entry point."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node('calibrate')
    exit_code = 0

    try:
        config = _config_from_node(node)
        SO101Calibrator(config).run()
    except KeyboardInterrupt:
        node.get_logger().info('Calibration cancelled.')
        exit_code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code
