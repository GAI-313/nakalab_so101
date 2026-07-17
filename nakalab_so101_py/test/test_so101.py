"""Tests for the Python SO-101 ROS2 drivers."""

import math
from types import SimpleNamespace

from nakalab_so101_py.feetech_bus import FeetechBus
from nakalab_so101_py.so101 import (
    all_motor_ids,
    clamp_joint_position,
    decode_load,
    decode_speed,
    FollowerArmDriverNode,
    MOTOR_IDS,
    rad_to_pos,
    REG_D_COEFFICIENT,
    REG_I_COEFFICIENT,
    REG_P_COEFFICIENT,
)

import pytest
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


class FakeBus:
    """Fake Feetech bus used by SO-101 driver tests."""

    def __init__(self):
        """Initialize recorded bus operations."""
        self.port = None
        self.baudrate = None
        self.connected = False
        self.torque_calls = []
        self.goal_states = []
        self.unlock_calls = []
        self.lock_calls = []
        self.u8_writes = []
        self.u8_reads = {}
        self.present_states = {
            motor_id: {
                'position': 2048,
                'speed': 0,
                'load': 0,
            }
            for motor_id in all_motor_ids()
        }

    def connect(self):
        """Record a bus connection."""
        self.connected = True

    def disconnect(self):
        """Record a bus disconnection."""
        self.connected = False

    def is_connected(self):
        """Return whether the fake bus is connected."""
        return self.connected

    def enable_torque(self, motor_ids, enable=True):
        """Record torque enable/disable requests."""
        self.torque_calls.append((list(motor_ids), enable))

    def disable_torque(self, motor_ids):
        """Record torque disable requests."""
        self.enable_torque(motor_ids, False)

    def unlock_eeprom(self, motor_ids):
        """Record EEPROM unlock requests."""
        self.unlock_calls.append(list(motor_ids))

    def lock_eeprom(self, motor_ids):
        """Record EEPROM lock requests."""
        self.lock_calls.append(list(motor_ids))

    def read_u8(self, motor_id, address):
        """Read a fake 8-bit register."""
        return self.u8_reads.get((motor_id, address), 0)

    def write_u8(self, motor_id, address, value):
        """Record an unsigned 8-bit register write."""
        self.u8_writes.append((motor_id, address, value))

    def sync_write_goal_states(self, goal_states):
        """Record follower goal state writes."""
        self.goal_states.append(goal_states)

    def sync_read_present_states(self, motor_ids):
        """Read fake present states."""
        return {
            motor_id: self.present_states[motor_id]
            for motor_id in motor_ids
            if motor_id in self.present_states
        }


class CapturePublisher:
    """Capture messages published by a ROS node."""

    def __init__(self):
        """Initialize the published message list."""
        self.messages = []

    def publish(self, msg):
        """Record one published message."""
        self.messages.append(msg)


@pytest.fixture
def fake_ros_node(monkeypatch):
    """Patch rclpy Node methods so tests do not create a DDS participant."""
    publishers = []

    class FakeLogger:
        """Collect log messages."""

        def __init__(self):
            """Initialize log buffers."""
            self.errors = []
            self.infos = []

        def error(self, message):
            """Record an error log."""
            self.errors.append(message)

        def info(self, message):
            """Record an info log."""
            self.infos.append(message)

    class FakeClock:
        """Return a stable ROS timestamp."""

        def now(self):
            """Return an object with a to_msg method."""
            return SimpleNamespace(to_msg=lambda: SimpleNamespace())

    def node_init(self, node_name, parameter_overrides=None):
        self.fake_node_name = node_name
        self.fake_logger = FakeLogger()
        self.fake_parameters = {
            parameter.name: parameter.value
            for parameter in (parameter_overrides or [])
        }

    def declare_parameter(self, name, default_value):
        self.fake_parameters.setdefault(name, default_value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.fake_parameters[name])

    def create_publisher(self, *args, **kwargs):
        publisher = CapturePublisher()
        publishers.append(publisher)
        return publisher

    def create_subscription(self, *args, **kwargs):
        return SimpleNamespace(args=args, kwargs=kwargs)

    def create_timer(self, *args, **kwargs):
        return SimpleNamespace(args=args, kwargs=kwargs)

    def get_logger(self):
        return self.fake_logger

    def get_clock(self):
        return FakeClock()

    monkeypatch.setattr(Node, '__init__', node_init)
    monkeypatch.setattr(Node, 'declare_parameter', declare_parameter)
    monkeypatch.setattr(Node, 'get_parameter', get_parameter)
    monkeypatch.setattr(Node, 'create_publisher', create_publisher)
    monkeypatch.setattr(Node, 'create_subscription', create_subscription)
    monkeypatch.setattr(Node, 'create_timer', create_timer)
    monkeypatch.setattr(Node, 'get_logger', get_logger)
    monkeypatch.setattr(Node, 'get_clock', get_clock)

    return publishers


@pytest.fixture
def fake_follower(fake_ros_node):
    """Create a follower node backed by a fake bus."""
    fake_bus = FakeBus()

    def bus_factory(port, baudrate):
        fake_bus.port = port
        fake_bus.baudrate = baudrate
        return fake_bus

    node = FollowerArmDriverNode(
        bus_factory=bus_factory,
        parameter_overrides=[
            Parameter('device', value='/dev/fake'),
            Parameter('max_speed', value=1234),
        ],
    )
    try:
        yield node, fake_bus
    finally:
        node.close()


def make_command(names, positions, velocities=None):
    """Create a JointState command message."""
    msg = JointState()
    msg.name = list(names)
    msg.position = list(positions)
    if velocities is not None:
        msg.velocity = list(velocities)
    return msg


def test_follower_init_enables_torque(fake_follower):
    """Enable torque for all motors when the follower starts."""
    _node, fake_bus = fake_follower

    assert fake_bus.port == '/dev/fake'
    assert fake_bus.baudrate == 1000000
    assert fake_bus.connected is True
    assert fake_bus.torque_calls[0] == (all_motor_ids(), False)
    assert fake_bus.torque_calls[-1] == (all_motor_ids(), True)


def test_follower_init_configures_position_pid_before_torque_enable(
    fake_follower,
):
    """Configure follower motors before enabling torque."""
    _node, fake_bus = fake_follower

    assert fake_bus.unlock_calls == [all_motor_ids()]
    assert fake_bus.lock_calls == [all_motor_ids()]
    assert (
        MOTOR_IDS['shoulder_pan'],
        FeetechBus.REG_OPERATING_MODE,
        0,
    ) in fake_bus.u8_writes
    assert (
        MOTOR_IDS['shoulder_pan'],
        REG_P_COEFFICIENT,
        16,
    ) in fake_bus.u8_writes
    assert (
        MOTOR_IDS['shoulder_pan'],
        REG_I_COEFFICIENT,
        0,
    ) in fake_bus.u8_writes
    assert (
        MOTOR_IDS['shoulder_pan'],
        REG_D_COEFFICIENT,
        32,
    ) in fake_bus.u8_writes
    assert fake_bus.torque_calls[0] == (all_motor_ids(), False)
    assert fake_bus.torque_calls[-1] == (all_motor_ids(), True)


def test_follower_close_disables_torque(fake_follower):
    """Disable torque and disconnect the bus during shutdown."""
    node, fake_bus = fake_follower

    node.close()

    assert fake_bus.torque_calls[-1] == (all_motor_ids(), False)
    assert fake_bus.connected is False


def test_follower_command_uses_default_speed_and_ignores_unknown_joint(
    fake_follower,
):
    """Convert known joints to raw positions and ignore unknown joints."""
    node, fake_bus = fake_follower
    command = make_command(
        ['shoulder_pan', 'unknown_joint', 'gripper'],
        [0.0, 1.0, math.pi / 2.0],
    )

    node.joint_cmd_callback(command)

    assert len(fake_bus.goal_states) == 1
    goal_states = fake_bus.goal_states[0]
    assert sorted(goal_states) == [
        MOTOR_IDS['shoulder_pan'],
        MOTOR_IDS['gripper'],
    ]
    assert goal_states[MOTOR_IDS['shoulder_pan']] == {
        'position': rad_to_pos(0.0),
        'speed': 1234,
    }
    assert goal_states[MOTOR_IDS['gripper']] == {
        'position': rad_to_pos(math.pi / 2.0),
        'speed': 1234,
    }


def test_follower_command_clamps_to_joint_limits(fake_follower):
    """Clamp incoming radian commands to URDF joint limits."""
    node, fake_bus = fake_follower
    command = make_command(['gripper'], [100.0])

    node.joint_cmd_callback(command)

    goal_state = fake_bus.goal_states[0][MOTOR_IDS['gripper']]
    assert goal_state['position'] == rad_to_pos(
        clamp_joint_position('gripper', 100.0)
    )


def test_follower_command_skips_duplicate_goal_writes(fake_follower):
    """Do not rewrite identical periodic JointState commands."""
    node, fake_bus = fake_follower
    command = make_command(['shoulder_pan'], [0.0])

    node.joint_cmd_callback(command)
    node.joint_cmd_callback(command)

    assert len(fake_bus.goal_states) == 1


def test_follower_command_uses_velocity_for_speed(fake_follower):
    """Use JointState velocity to set raw servo speed when present."""
    node, fake_bus = fake_follower
    command = make_command(
        ['wrist_roll'],
        [-math.pi / 4.0],
        velocities=[2.0],
    )

    node.joint_cmd_callback(command)

    goal_state = fake_bus.goal_states[0][MOTOR_IDS['wrist_roll']]
    assert goal_state['position'] == rad_to_pos(-math.pi / 4.0)
    assert goal_state['speed'] == 1302


def test_follower_command_uses_default_speed_when_velocity_is_partial(
    fake_follower,
):
    """Ignore velocity if it does not cover every command name."""
    node, fake_bus = fake_follower
    command = make_command(
        ['shoulder_lift', 'elbow_flex'],
        [0.25, -0.25],
        velocities=[9.0],
    )

    node.joint_cmd_callback(command)

    goal_states = fake_bus.goal_states[0]
    assert goal_states[MOTOR_IDS['shoulder_lift']]['speed'] == 1234
    assert goal_states[MOTOR_IDS['elbow_flex']]['speed'] == 1234


def test_follower_timer_publishes_present_joint_states(fake_follower):
    """Publish present position, velocity, and load as follower joint states."""
    node, fake_bus = fake_follower
    capture = CapturePublisher()
    node.joint_state_pub = capture
    fake_bus.present_states[MOTOR_IDS['shoulder_pan']] = {
        'position': 2048,
        'speed': 651,
        'load': 100,
    }
    fake_bus.present_states[MOTOR_IDS['gripper']] = {
        'position': 3072,
        'speed': -32767,
        'load': 0x0401,
    }

    node.timer_callback()

    assert len(capture.messages) == 1
    msg = capture.messages[0]
    shoulder_index = msg.name.index('shoulder_pan')
    gripper_index = msg.name.index('gripper')

    assert msg.position[shoulder_index] == pytest.approx(0.0)
    assert msg.velocity[shoulder_index] == pytest.approx(decode_speed(651))
    assert msg.effort[shoulder_index] == pytest.approx(decode_load(100))
    assert msg.position[gripper_index] == pytest.approx(math.pi / 2.0)
    assert msg.velocity[gripper_index] == pytest.approx(
        decode_speed(-32767),
    )
    assert msg.effort[gripper_index] == pytest.approx(decode_load(0x0401))
