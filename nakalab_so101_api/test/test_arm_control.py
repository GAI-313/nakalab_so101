"""Unit tests for the SO-101 ArmControl API."""

from types import SimpleNamespace
import time
from unittest.mock import Mock, patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
import pytest

from nakalab_so101_api import ArmControl, ArmControlStatus
from nakalab_so101_api.nakalab_so101_api import IkResult, IkStatus


class DoneFuture:
    """Minimal completed future used by action tests."""

    def __init__(self, value):
        self._value = value

    def add_done_callback(self, callback):
        callback(self)

    def done(self):
        return True

    def result(self):
        return self._value


class PendingFuture:
    """Minimal future that completes after cancellation is requested."""

    def __init__(self):
        self._callbacks = []
        self._value = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def complete(self, value):
        self._value = value
        for callback in self._callbacks:
            callback(self)

    def done(self):
        return self._value is not None

    def result(self):
        return self._value


class FakeGoalHandle:
    """Goal handle that records cancellation requests."""

    accepted = True

    def __init__(self, result_future=None):
        self.cancel_calls = 0
        self.goal_id = SimpleNamespace(uuid=[1] * 16)
        self._result_future = result_future or DoneFuture(Mock())

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return DoneFuture(SimpleNamespace(goals_canceling=[Mock()]))

    def get_result_async(self):
        return self._result_future


class FakeActionClient:
    """Capture MoveGroup goals without a ROS action server."""

    def __init__(self, *args):
        self.goals = []
        self.future = PendingFuture()

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.future


class FakeIkClient:
    """Stand-in for the IK service client."""

    def wait_for_service(self, timeout_sec):
        return True


def make_node():
    """Create the narrow node mock required by ArmControl."""
    node = Mock()
    node.context.ok.return_value = True
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    return node


def make_arm(tmp_path):
    """Create ArmControl with a local SRDF and fake ROS clients."""
    srdf = tmp_path / 'so101.srdf'
    srdf.write_text(
        '''<robot name="so101">
  <group_state name="home" group="so101_follower">
    <joint name="shoulder_pan" value="0.1"/>
    <joint name="shoulder_lift" value="-0.2"/>
    <joint name="elbow_flex" value="0.3"/>
    <joint name="wrist_flex" value="0.4"/>
    <joint name="wrist_roll" value="0.5"/>
  </group_state>
  <group_state name="open" group="so101_gripper">
    <joint name="gripper" value="1.57"/>
  </group_state>
</robot>''',
        encoding='utf-8',
    )
    with patch(
        'nakalab_so101_api.nakalab_so101_api.ActionClient', FakeActionClient
    ):
        arm = ArmControl(make_node(), tf_buffer=Mock(), srdf_path=str(srdf))
    arm._ik_client = FakeIkClient()
    arm._joint_states = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0,
    }
    arm._joint_state_updated_at = dict.fromkeys(arm._joint_states, time.monotonic())
    return arm


def test_joint_control_detailed_submits_plan_only_goal(tmp_path):
    """Detailed control must return SUBMITTED and preserve held joints."""
    arm = make_arm(tmp_path)

    result = arm.joint_control_detailed(
        shoulder_pan=0.2,
        execute=False,
        wait=False,
    )

    assert result.status == ArmControlStatus.SUBMITTED
    goal = arm._move_group_client.goals[-1]
    assert goal.planning_options.plan_only
    positions = {
        item.joint_name: item.position
        for item in goal.request.goal_constraints[0].joint_constraints
    }
    assert positions['shoulder_pan'] == 0.2
    assert positions['wrist_roll'] == 0.0


def test_joint_control_rejects_stale_relative_state(tmp_path):
    """Relative control must not use a stale JointState."""
    arm = make_arm(tmp_path)
    arm._joint_state_updated_at['elbow_flex'] -= arm.joint_state_max_age + 0.1

    result = arm.joint_control_detailed(rel=True, elbow_flex=0.2, wait=False)

    assert result.status == ArmControlStatus.STATE_UNAVAILABLE
    assert not arm._move_group_client.goals


def test_complete_absolute_goal_does_not_require_joint_state(tmp_path):
    """All-joint absolute control must work before JointState is received."""
    arm = make_arm(tmp_path)
    arm._joint_states.clear()
    arm._joint_state_updated_at.clear()

    result = arm.joint_control_detailed(
        shoulder_pan=0.0,
        shoulder_lift=0.0,
        elbow_flex=0.0,
        wrist_flex=0.0,
        wrist_roll=0.0,
        wait=False,
    )

    assert result.status == ArmControlStatus.SUBMITTED


def test_scaling_and_pose_tolerance_are_applied(tmp_path):
    """Motion scaling and half-width tolerance must reach MoveIt messages."""
    arm = make_arm(tmp_path)
    arm.set_motion_scaling(velocity_scale=0.25, acceleration_scale=0.4)
    pose = PoseStamped()
    pose.header.frame_id = 'arm_base_link'
    pose.pose.orientation.w = 1.0

    goal = arm._pose_goal(pose, 'so101_follower', 'gripper_frame_link', 1, 1.0, True)

    assert goal.request.max_velocity_scaling_factor == pytest.approx(0.25)
    assert goal.request.max_acceleration_scaling_factor == pytest.approx(0.4)
    box = goal.request.goal_constraints[0].position_constraints[0]
    dimensions = box.constraint_region.primitives[0].dimensions
    assert list(dimensions) == pytest.approx([0.01] * 3)


def test_group_state_introspection_returns_copies(tmp_path):
    """Blockly metadata access must list and copy SRDF states."""
    arm = make_arm(tmp_path)

    assert arm.list_group_states() == ('home',)
    state = arm.get_group_state('home')
    state['shoulder_pan'] = 99.0

    assert arm.get_group_state('home')['shoulder_pan'] == 0.1
    assert arm.get_supported_joints() == arm._ARM_JOINTS


def test_current_pose_copies_tf_translation_into_point(tmp_path):
    """simple=False must return a PoseStamped with copied Point values."""
    arm = make_arm(tmp_path)
    arm.tf_buffer.lookup_transform.return_value = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.1, y=-0.2, z=0.3),
            rotation=Quaternion(w=2.0),
        )
    )

    with patch('nakalab_so101_api.nakalab_so101_api.rclpy.ok', return_value=True):
        pose = arm.get_current_pose(simple=False)

    assert pose.header.frame_id == 'arm_base_link'
    assert (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) == (
        0.1,
        -0.2,
        0.3,
    )
    assert pose.pose.orientation.w == pytest.approx(1.0)


def test_detailed_pose_fallback_is_opt_in_and_degraded(tmp_path):
    """Position-only fallback must be explicit and reported as DEGRADED."""
    arm = make_arm(tmp_path)
    pose = PoseStamped()
    pose.header.frame_id = 'arm_base_link'
    pose.pose.orientation.w = 1.0
    arm._solve_ik_detailed = Mock(return_value=IkResult(IkStatus.NO_SOLUTION))
    failed = arm._result(ArmControlStatus.MOVEIT_FAILED, 'full pose failed')
    succeeded = arm._result(ArmControlStatus.SUCCEEDED, 'position succeeded')
    arm._send_move_group_goal_detailed = Mock(
        side_effect=[failed, failed, succeeded]
    )

    strict = arm.move_to_pose_detailed(pose)
    relaxed = arm.move_to_pose_detailed(pose, allow_position_only_fallback=True)

    assert strict.status == ArmControlStatus.MOVEIT_FAILED
    assert relaxed.status == ArmControlStatus.DEGRADED
    assert relaxed.used_position_only_fallback


def test_wait_false_rejects_automatic_pose_fallback(tmp_path):
    """Async pose commands cannot determine whether fallback is necessary."""
    arm = make_arm(tmp_path)
    pose = PoseStamped()
    pose.header.frame_id = 'arm_base_link'
    pose.pose.orientation.w = 1.0

    result = arm.move_to_pose_detailed(
        pose,
        wait=False,
        allow_position_only_fallback=True,
    )

    assert result.status == ArmControlStatus.INVALID_ARGUMENT


def test_cancel_current_goal_cancels_active_handle(tmp_path):
    """Explicit cancellation must target the active goal handle."""
    arm = make_arm(tmp_path)
    handle = FakeGoalHandle()
    arm._track_active_goal(handle)

    result = arm.cancel_current_goal()

    assert result.status == ArmControlStatus.CANCELLED
    assert handle.cancel_calls == 1


def test_pending_goal_is_cancelled_once_accepted(tmp_path):
    """Cancellation before goal acceptance must be deferred safely."""
    arm = make_arm(tmp_path)
    pending = arm._move_group_client.future
    result = arm.joint_control_detailed(shoulder_pan=0.1, wait=False)

    cancel = arm.cancel_current_goal()
    handle = FakeGoalHandle()
    pending.complete(handle)

    assert result.status == ArmControlStatus.SUBMITTED
    assert cancel.status == ArmControlStatus.CANCELLED
    assert handle.cancel_calls == 1


def test_preexisting_cancel_prevents_goal_send(tmp_path):
    """A Stop arriving immediately before send must not be cleared or submitted."""
    arm = make_arm(tmp_path)
    arm.request_cancel()

    result = arm.joint_control_detailed(shoulder_pan=0.1, wait=False)

    assert result.status == ArmControlStatus.CANCELLED
    assert not arm._move_group_client.goals


def test_keyboard_interrupt_cancels_accepted_goal(tmp_path):
    """KeyboardInterrupt must cancel the goal and reach the lifecycle owner."""
    arm = make_arm(tmp_path)
    result_future = PendingFuture()
    handle = FakeGoalHandle(result_future)
    arm._move_group_client.future = DoneFuture(handle)
    spin = 'nakalab_so101_api.nakalab_so101_api.rclpy.spin_once'

    with patch(spin, side_effect=KeyboardInterrupt()), pytest.raises(KeyboardInterrupt):
        arm._send_move_group_goal_detailed(
            MoveGroup.Goal(), wait=True, execute=True
        )

    assert handle.cancel_calls == 1


def test_readiness_does_not_touch_ros_entities_after_shutdown(tmp_path):
    """Readiness checks must short-circuit after context shutdown."""
    arm = make_arm(tmp_path)
    arm.node.context.ok.return_value = False
    arm._move_group_client.wait_for_server = Mock(side_effect=AssertionError)
    arm._ik_client.wait_for_service = Mock(side_effect=AssertionError)

    assert not arm.is_move_group_ready()
    assert not arm.is_ik_ready()


def test_cancel_does_not_call_ros_after_shutdown(tmp_path):
    """Cancellation must not call a goal handle on an invalid context."""
    arm = make_arm(tmp_path)
    handle = FakeGoalHandle()
    arm._track_active_goal(handle)
    arm.node.context.ok.return_value = False

    result = arm.cancel_current_goal()

    assert result.status == ArmControlStatus.ROS_SHUTDOWN
    assert handle.cancel_calls == 0


def test_wait_for_future_reports_shutdown_without_spin(tmp_path):
    """Future waiting must stop before spin when the context is invalid."""
    arm = make_arm(tmp_path)
    arm.node.context.ok.return_value = False

    with patch('nakalab_so101_api.nakalab_so101_api.rclpy.spin_once') as spin:
        status = arm._wait_for_future(PendingFuture(), 1.0)

    assert status.value == 'ros_shutdown'
    spin.assert_not_called()
