"""Interactive ROS2 command for setting SO-101 Feetech motor IDs."""

from dataclasses import dataclass
import time
from typing import Callable, List, Optional, Sequence, Tuple

from nakalab_so101_py.feetech_bus import FeetechBus


VALID_ARM_TYPES = ('so101_leader', 'so101_follower')

SETUP_SEQUENCE: Sequence[Tuple[str, int]] = (
    ('gripper', 6),
    ('wrist_roll', 5),
    ('wrist_flex', 4),
    ('elbow_flex', 3),
    ('shoulder_lift', 2),
    ('shoulder_pan', 1),
)

DEFAULT_SCAN_BAUDRATES: Sequence[int] = (
    1000000,
    500000,
    250000,
    128000,
    115200,
    57600,
    38400,
    19200,
)


@dataclass(frozen=True)
class SetupConfig:
    """Configuration for a motor setup run."""

    device: str
    arm_type: str = 'so101_follower'
    baudrate: int = 1000000


@dataclass(frozen=True)
class MotorDetection:
    """One motor detected on the serial bus."""

    baudrate: int
    motor_id: int
    model_number: Optional[int]


def validate_config(config: SetupConfig) -> None:
    """Validate user-provided setup parameters."""
    if not config.device:
        raise ValueError("Parameter 'device' must be set.")

    if config.arm_type not in VALID_ARM_TYPES:
        valid_types = ', '.join(VALID_ARM_TYPES)
        raise ValueError(
            f"Parameter 'arm_type' must be one of: {valid_types}"
        )

    FeetechBus.baudrate_to_register(config.baudrate)


class MotorSetup:
    """Set SO-101 motor IDs and baudrate one motor at a time."""

    def __init__(
        self,
        config: SetupConfig,
        scan_baudrates: Sequence[int] = DEFAULT_SCAN_BAUDRATES,
        bus_factory: Callable[..., FeetechBus] = FeetechBus,
        input_func: Callable[[str], str] = input,
        output: Callable[[str], None] = print,
        sleep_func: Callable[[float], None] = time.sleep,
    ):
        """Initialize setup dependencies."""
        self.config = config
        self.scan_baudrates = scan_baudrates
        self.bus_factory = bus_factory
        self.input_func = input_func
        self.output = output
        self.sleep_func = sleep_func

    def run(self) -> None:
        """Run the interactive setup sequence."""
        validate_config(self.config)
        self.output(
            f'Setting up {self.config.arm_type} motors on '
            f'{self.config.device}.'
        )
        self.output('Connect exactly one motor at each step.')

        for motor_name, target_id in SETUP_SEQUENCE:
            self.input_func(
                'Connect the controller board to the '
                f"'{motor_name}' motor only and press enter."
            )
            detection = self.find_single_motor()
            self.setup_single_motor(detection, target_id)
            self.output(f"'{motor_name}' motor id set to {target_id}")

    def find_single_motor(self) -> MotorDetection:
        """Scan configured baudrates and require exactly one motor."""
        detections: List[MotorDetection] = []

        for baudrate in self.scan_baudrates:
            bus = self._open_bus(baudrate)
            try:
                found = bus.broadcast_ping()
            finally:
                bus.disconnect()

            for motor_id, model_number in sorted(found.items()):
                detections.append(
                    MotorDetection(
                        baudrate=baudrate,
                        motor_id=motor_id,
                        model_number=model_number,
                    )
                )

        if len(detections) == 1:
            return detections[0]

        if not detections:
            raise RuntimeError(
                'No motor was found. Check power, wiring, the serial port, '
                'and that exactly one motor is connected.'
            )

        raise RuntimeError(
            'Expected exactly one connected motor, but found: '
            f'{self._format_detections(detections)}'
        )

    def setup_single_motor(
        self,
        detection: MotorDetection,
        target_id: int,
    ) -> None:
        """Write one motor's ID and baudrate, then verify the result."""
        target_baudrate = self.config.baudrate
        baudrate_raw = FeetechBus.baudrate_to_register(target_baudrate)
        bus = self._open_bus(detection.baudrate)

        try:
            self._write_u8(
                bus,
                detection.motor_id,
                FeetechBus.REG_LOCK,
                0,
                'unlock EEPROM',
            )
            self.sleep_func(0.05)
            self._write_u8(
                bus,
                detection.motor_id,
                FeetechBus.REG_ID,
                target_id,
                'write ID',
            )
            self.sleep_func(0.05)
            self._write_u8(
                bus,
                target_id,
                FeetechBus.REG_BAUD_RATE,
                baudrate_raw,
                'write baudrate',
            )
            self.sleep_func(0.1)

            bus.disconnect()
            bus.set_baudrate(target_baudrate)
            bus.connect()
            self.sleep_func(0.05)

            self._write_u8(
                bus,
                target_id,
                FeetechBus.REG_LOCK,
                1,
                'lock EEPROM',
            )
            self.verify_motor(bus, target_id, baudrate_raw)
        finally:
            bus.disconnect()

    def verify_motor(
        self,
        bus: FeetechBus,
        target_id: int,
        expected_baudrate_raw: int,
    ) -> None:
        """Verify the target ID and baudrate can be read back."""
        read_id = bus.read_u8(target_id, FeetechBus.REG_ID)
        if read_id != target_id:
            raise RuntimeError(
                f'Verification failed: expected motor ID {target_id}, '
                f'read {read_id}.'
            )

        read_baudrate = bus.read_u8(target_id, FeetechBus.REG_BAUD_RATE)
        if read_baudrate != expected_baudrate_raw:
            raise RuntimeError(
                'Verification failed: expected baudrate register '
                f'{expected_baudrate_raw}, read {read_baudrate}.'
            )

        model_number = bus.read_u16(target_id, FeetechBus.REG_MODEL)
        if model_number is None:
            self.output(f'Verified motor {target_id}.')
        else:
            self.output(
                f'Verified motor {target_id} '
                f'(model {model_number}).'
            )

    def _open_bus(self, baudrate: int) -> FeetechBus:
        bus = self.bus_factory(self.config.device, baudrate=baudrate)
        bus.connect()
        return bus

    @staticmethod
    def _write_u8(
        bus: FeetechBus,
        motor_id: int,
        address: int,
        value: int,
        label: str,
    ) -> None:
        status = bus.write_u8(motor_id, address, value)
        if status is not None and status.error != 0:
            raise RuntimeError(
                f'{label} failed for motor {motor_id}: '
                f'status error 0x{status.error:02x}'
            )

    @staticmethod
    def _format_detections(detections: Sequence[MotorDetection]) -> str:
        parts = []
        for detection in detections:
            parts.append(
                'id='
                f'{detection.motor_id}, '
                f'baudrate={detection.baudrate}, '
                f'model={detection.model_number}'
            )
        return '; '.join(parts)


def _config_from_node(node) -> SetupConfig:
    node.declare_parameter('device', '')
    node.declare_parameter('arm_type', 'so101_follower')
    node.declare_parameter('baudrate', 1000000)

    return SetupConfig(
        device=node.get_parameter('device').value,
        arm_type=node.get_parameter('arm_type').value,
        baudrate=node.get_parameter('baudrate').value,
    )


def main(args=None):
    """Run the ROS2 console script entry point."""
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node('setup_motors')
    exit_code = 0

    try:
        config = _config_from_node(node)
        MotorSetup(config).run()
    except KeyboardInterrupt:
        node.get_logger().info('Motor setup cancelled.')
        exit_code = 130
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code
