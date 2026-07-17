#!/usr/bin/env python3
"""ROS2 Python drivers for SO-101 leader and follower arms."""

import math
from typing import Callable, Dict, Optional

from nakalab_so101_py.feetech_bus import FeetechBus

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool


MOTOR_IDS = {
    'shoulder_pan': 1,
    'shoulder_lift': 2,
    'elbow_flex': 3,
    'wrist_flex': 4,
    'wrist_roll': 5,
    'gripper': 6,
}
ID_TO_NAME = {value: key for key, value in MOTOR_IDS.items()}

POSITION_CENTER = 2048.0
POSITION_STEPS = 4096.0
SPEED_STEPS_PER_RAD_S = 651.0
DEFAULT_COMMAND_DEADBAND_STEPS = 1

JOINT_LIMITS = {
    'shoulder_pan': (-1.91986, 1.91986),
    'shoulder_lift': (-1.74533, 1.74533),
    'elbow_flex': (-1.69, 1.69),
    'wrist_flex': (-1.65806, 1.65806),
    'wrist_roll': (-2.74385, 2.84121),
    'gripper': (-0.174533, 1.74533),
}

REG_P_COEFFICIENT = 21
REG_D_COEFFICIENT = 22
REG_I_COEFFICIENT = 23


def all_motor_ids():
    """Return all SO-101 motor IDs."""
    return list(MOTOR_IDS.values())


def pos_to_rad(pos: int) -> float:
    """Convert a Feetech raw position to radians."""
    return (float(pos) - POSITION_CENTER) * (2.0 * math.pi) / POSITION_STEPS


def rad_to_pos(rad: float) -> int:
    """Convert radians to a clamped Feetech raw position."""
    raw_position = int(rad * POSITION_STEPS / (2.0 * math.pi) + POSITION_CENTER)
    return max(0, min(4095, raw_position))


def clamp_joint_position(name: str, position: float) -> float:
    """Clamp a joint command to the SO-101 URDF joint limit."""
    lower, upper = JOINT_LIMITS[name]
    return max(lower, min(upper, float(position)))


def decode_signed(raw_value: int, sign_bit: int, magnitude_mask: int) -> int:
    """Decode a sign-bit plus magnitude value."""
    if raw_value & sign_bit:
        return -(raw_value & magnitude_mask)
    return raw_value & magnitude_mask


def decode_speed(raw_speed: int) -> float:
    """Convert Feetech raw speed to rad/s."""
    speed = decode_signed(int(raw_speed), 0x8000, 0x7FFF)
    return float(speed) / SPEED_STEPS_PER_RAD_S


def decode_load(raw_load: int) -> float:
    """Convert Feetech raw load to normalized effort."""
    load = decode_signed(int(raw_load), 0x0400, 0x03FF)
    return float(load) / 1000.0


class LeaderArmDriverNode(Node):
    """Publish leader arm joint states."""

    def __init__(
        self,
        bus_factory: Callable[..., FeetechBus] = FeetechBus,
        parameter_overrides: Optional[list] = None,
    ):
        """Initialize the leader arm driver."""
        super().__init__(
            'leader_arm_driver_node',
            parameter_overrides=parameter_overrides,
        )
        self.bus = None
        self.motor_ids = MOTOR_IDS
        self.id_to_name = ID_TO_NAME

        self.declare_parameter('device', '')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('enable_teleop', False)

        device_port = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self.teleop_enabled = self.get_parameter('enable_teleop').value

        if not device_port:
            self.get_logger().error("Parameter 'device' must be set!")
            return

        self.bus = bus_factory(port=device_port, baudrate=baudrate)
        self.bus.connect()
        self.bus.enable_torque(all_motor_ids(), False)

        self.joint_state_pub = self.create_publisher(
            JointState,
            'leader/joint_states',
            10,
        )
        self.teleop_service = self.create_service(
            SetBool,
            'teleop',
            self.teleop_service_callback,
        )
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f'Leader Arm Driver started on {device_port}')

    def close(self) -> None:
        """Disable torque and close the bus."""
        if self.bus is not None and self.bus.is_connected():
            self.bus.enable_torque(all_motor_ids(), False)
            self.bus.disconnect()

    def teleop_service_callback(self, request, response):
        """Enable or disable publishing leader joint states."""
        self.teleop_enabled = request.data
        response.success = True
        response.message = f'Teleop enabled: {self.teleop_enabled}'
        self.get_logger().info(response.message)
        return response

    def timer_callback(self) -> None:
        """Read leader positions and publish joint states."""
        if (
            not self.teleop_enabled or
            self.bus is None or
            not self.bus.is_connected()
        ):
            return

        positions = self.bus.sync_read_present_positions(all_motor_ids())
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        for motor_id, position in positions.items():
            if motor_id in self.id_to_name:
                msg.name.append(self.id_to_name[motor_id])
                msg.position.append(pos_to_rad(position))

        self.joint_state_pub.publish(msg)


class FollowerArmDriverNode(Node):
    """Receive joint commands and drive the follower arm."""

    def __init__(
        self,
        bus_factory: Callable[..., FeetechBus] = FeetechBus,
        parameter_overrides: Optional[list] = None,
    ):
        """Initialize the follower arm driver."""
        super().__init__(
            'follower_arm_driver_node',
            parameter_overrides=parameter_overrides,
        )
        self.bus = None
        self.motor_ids = MOTOR_IDS
        self.id_to_name = ID_TO_NAME

        self.declare_parameter('device', '')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('max_speed', 2048)
        self.declare_parameter('command_deadband_steps', 1)
        self.declare_parameter('configure_motors', True)

        device_port = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.command_deadband_steps = int(
            self.get_parameter('command_deadband_steps').value
        )
        self.last_goal_states: Dict[int, Dict[str, int]] = {}

        if not device_port:
            self.get_logger().error("Parameter 'device' must be set!")
            return

        self.bus = bus_factory(port=device_port, baudrate=baudrate)
        self.bus.connect()
        if self.get_parameter('configure_motors').value:
            self.configure_motors()
        self.bus.enable_torque(all_motor_ids(), True)

        self.joint_state_pub = self.create_publisher(
            JointState,
            'follower/joint_states',
            10,
        )
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            'follower/joint_commands',
            self.joint_cmd_callback,
            10,
        )
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(f'Follower Arm Driver started on {device_port}')

    def close(self) -> None:
        """Disable torque and close the bus."""
        if self.bus is not None and self.bus.is_connected():
            self.bus.enable_torque(all_motor_ids(), False)
            self.bus.disconnect()

    def configure_motors(self) -> None:
        """Apply SO-101 follower motor settings used by LeRobot."""
        if self.bus is None or not self.bus.is_connected():
            return

        motor_ids = all_motor_ids()
        self.bus.disable_torque(motor_ids)
        self.bus.unlock_eeprom(motor_ids)
        for motor_id in motor_ids:
            self.bus.write_u8(
                motor_id,
                FeetechBus.REG_OPERATING_MODE,
                0,
            )
            phase = self.bus.read_u8(motor_id, FeetechBus.REG_PHASE)
            if phase is not None and phase & 0x10:
                self.bus.write_u8(
                    motor_id,
                    FeetechBus.REG_PHASE,
                    phase & ~0x10,
                )

            # LeRobot lowers P from the default 32 to reduce holding jitter.
            self.bus.write_u8(motor_id, REG_P_COEFFICIENT, 16)
            self.bus.write_u8(motor_id, REG_I_COEFFICIENT, 0)
            self.bus.write_u8(motor_id, REG_D_COEFFICIENT, 32)

        self.bus.lock_eeprom(motor_ids)

    def joint_cmd_callback(self, msg: JointState) -> None:
        """Convert joint command messages to Feetech goal states."""
        if self.bus is None or not self.bus.is_connected():
            return

        self.max_speed = self.get_parameter('max_speed').value
        self.command_deadband_steps = int(
            self.get_parameter('command_deadband_steps').value
        )
        use_velocity = len(msg.velocity) == len(msg.name)
        goal_states: Dict[int, Dict[str, int]] = {}

        for index, name in enumerate(msg.name):
            if index >= len(msg.position) or name not in self.motor_ids:
                continue

            speed = int(self.max_speed)
            if use_velocity:
                raw_speed = abs(float(msg.velocity[index])) * SPEED_STEPS_PER_RAD_S
                speed = min(32767, int(raw_speed))

            motor_id = self.motor_ids[name]
            goal_states[motor_id] = {
                'position': rad_to_pos(
                    clamp_joint_position(name, msg.position[index])
                ),
                'speed': speed,
            }

        changed_goal_states = self.filter_changed_goal_states(goal_states)
        if changed_goal_states:
            self.bus.sync_write_goal_states(changed_goal_states)
            self.last_goal_states.update(changed_goal_states)

    def filter_changed_goal_states(
        self,
        goal_states: Dict[int, Dict[str, int]],
    ) -> Dict[int, Dict[str, int]]:
        """Return only goals that changed enough to warrant a servo write."""
        changed_goal_states = {}
        for motor_id, goal_state in goal_states.items():
            last_goal_state = self.last_goal_states.get(motor_id)
            if last_goal_state is None:
                changed_goal_states[motor_id] = goal_state
                continue

            position_delta = abs(
                goal_state['position'] - last_goal_state['position']
            )
            speed_delta = goal_state['speed'] != last_goal_state['speed']
            if (
                position_delta > self.command_deadband_steps or
                speed_delta
            ):
                changed_goal_states[motor_id] = goal_state

        return changed_goal_states

    def timer_callback(self) -> None:
        """Read follower states and publish joint states."""
        if self.bus is None or not self.bus.is_connected():
            return

        states = self.bus.sync_read_present_states(all_motor_ids())
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        for motor_id, state in states.items():
            if motor_id not in self.id_to_name:
                continue

            msg.name.append(self.id_to_name[motor_id])
            msg.position.append(pos_to_rad(state['position']))
            msg.velocity.append(decode_speed(state.get('speed', 0)))
            msg.effort.append(decode_load(state.get('load', 0)))

        self.joint_state_pub.publish(msg)


def leader(args=None):
    """Run the leader arm driver entry point."""
    rclpy.init(args=args)
    node = LeaderArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def follower(args=None):
    """Run the follower arm driver entry point."""
    rclpy.init(args=args)
    node = FollowerArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
