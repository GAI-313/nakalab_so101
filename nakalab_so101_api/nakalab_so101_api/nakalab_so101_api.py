"""MoveIt-based control API for a single SO-101 arm."""

from dataclasses import dataclass, replace
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Any, Optional
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import InvalidHandle, RCLError
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


class ArmControlStatus(str, Enum):
    """ArmControl の実行状態。"""

    SUCCEEDED = 'succeeded'
    DEGRADED = 'degraded'
    SUBMITTED = 'submitted'
    INVALID_ARGUMENT = 'invalid_argument'
    NOT_READY = 'not_ready'
    STATE_UNAVAILABLE = 'state_unavailable'
    TF_UNAVAILABLE = 'tf_unavailable'
    GOAL_REJECTED = 'goal_rejected'
    MOVEIT_FAILED = 'moveit_failed'
    CANCELLED = 'cancelled'
    TIMED_OUT = 'timed_out'
    ROS_SHUTDOWN = 'ros_shutdown'
    INTERNAL_ERROR = 'internal_error'


@dataclass(frozen=True)
class ArmControlResult:
    """ArmControl の詳細な実行結果。"""

    status: ArmControlStatus
    message: str
    moveit_error_code: Optional[int] = None
    moveit_error_name: Optional[str] = None
    action_status: Optional[int] = None
    plan_only: bool = False
    used_ik: bool = False
    used_position_only_fallback: bool = False

    @property
    def succeeded(self) -> bool:
        """通常成功または縮退成功かを返す。"""
        return self.status in {
            ArmControlStatus.SUCCEEDED,
            ArmControlStatus.DEGRADED,
        }

    @property
    def strict_success(self) -> bool:
        """縮退を含まない完全成功かを返す。"""
        return self.status == ArmControlStatus.SUCCEEDED


class IkStatus(str, Enum):
    """MoveIt inverse kinematics の内部結果。"""

    SUCCEEDED = 'succeeded'
    NO_SOLUTION = 'no_solution'
    SERVICE_UNAVAILABLE = 'service_unavailable'
    TIMED_OUT = 'timed_out'
    INVALID_REQUEST = 'invalid_request'
    INVALID_RESPONSE = 'invalid_response'
    STATE_UNAVAILABLE = 'state_unavailable'
    ROS_SHUTDOWN = 'ros_shutdown'


@dataclass(frozen=True)
class IkResult:
    """Inverse kinematics の詳細な結果。"""

    status: IkStatus
    joints: Optional[dict[str, float]] = None
    message: str = ''
    moveit_error_code: Optional[int] = None


class _FutureWaitStatus(str, Enum):
    """内部 future 待機状態。"""

    COMPLETED = 'completed'
    CANCELLED = 'cancelled'
    TIMED_OUT = 'timed_out'
    ROS_SHUTDOWN = 'ros_shutdown'


class ArmControl:
    """MoveIt を介して SO-101 を制御する API。

    既存の移動メソッドは bool を返す。Blockly などの外部ツールでは
    対応する ``*_detailed`` メソッドで ``ArmControlResult`` を取得する。

    Parameters
    ----------
    node : rclpy.node.Node
        API が利用する ROS 2 ノード。
    wait_time : float, optional
        action server および service の初期接続待機時間（秒）。
    move_action_name, ik_service_name, joint_state_topic : str, optional
        MoveIt action、IK service、JointState topic の名前。
    planning_group, gripper_group : str, optional
        アームおよびグリッパーの planning group 名。
    base_frame, tip_link : str, optional
        既定の基準 frame とエンドエフェクタ link。
    tf_buffer : tf2_ros.Buffer, optional
        利用済みの TF buffer。省略時は listener を作成する。
    srdf_path : str, optional
        group state を読む SRDF のパス。
    position_tolerance, orientation_tolerance, joint_tolerance : float, optional
        pose および関節 constraint の許容値。
    joint_state_max_age : float, optional
        現在状態として利用可能な JointState の最大経過時間（秒）。
    goal_response_timeout, result_timeout, cancel_timeout : float, optional
        goal 応答、MoveGroup 結果、cancel response の待機時間（秒）。
    velocity_scale, acceleration_scale : float, optional
        MoveIt の最大速度・加速度スケール。省略時はクラス既定値を使う。
    """

    _ARM_JOINTS = (
        'shoulder_pan',
        'shoulder_lift',
        'elbow_flex',
        'wrist_flex',
        'wrist_roll',
    )
    _GRIPPER_JOINTS = ('gripper',)
    DEFAULT_VELOCITY_SCALE = 1.0
    DEFAULT_ACCELERATION_SCALE = 0.8
    _SPIN_INTERVAL = 0.05

    def __init__(
        self,
        node: Any,
        wait_time: float = 5.0,
        move_action_name: str = '/move_action',
        ik_service_name: str = '/compute_ik',
        joint_state_topic: str = '/joint_states',
        planning_group: str = 'so101_follower',
        gripper_group: str = 'so101_gripper',
        base_frame: str = 'arm_base_link',
        tip_link: str = 'gripper_frame_link',
        tf_buffer: Optional[Buffer] = None,
        srdf_path: Optional[str] = None,
        position_tolerance: float = 0.005,
        velocity_scale: Optional[float] = None,
        acceleration_scale: Optional[float] = None,
        orientation_tolerance: float = 0.1,
        joint_tolerance: float = 0.01,
        joint_state_max_age: float = 1.0,
        goal_response_timeout: float = 5.0,
        result_timeout: float = 30.0,
        cancel_timeout: float = 2.0,
    ) -> None:
        self.node = node
        self.planning_group = planning_group
        self.gripper_group = gripper_group
        self.base_frame = base_frame
        self.tip_link = tip_link
        self.position_tolerance = self._validate_positive(
            'position_tolerance', position_tolerance
        )
        self.orientation_tolerance = self._validate_positive(
            'orientation_tolerance', orientation_tolerance
        )
        self.joint_tolerance = self._validate_positive(
            'joint_tolerance', joint_tolerance
        )
        self.joint_state_max_age = self._validate_positive(
            'joint_state_max_age', joint_state_max_age
        )
        self.goal_response_timeout = self._validate_positive(
            'goal_response_timeout', goal_response_timeout
        )
        self.result_timeout = self._validate_positive('result_timeout', result_timeout)
        self.cancel_timeout = self._validate_positive('cancel_timeout', cancel_timeout)
        self._velocity_scale = self.DEFAULT_VELOCITY_SCALE
        self._acceleration_scale = self.DEFAULT_ACCELERATION_SCALE
        self.set_motion_scaling(velocity_scale, acceleration_scale)

        self._joint_states: dict[str, float] = {}
        self._joint_state_updated_at: dict[str, float] = {}
        self._joint_state_lock = threading.RLock()
        self._srdf_group_states: Optional[dict[tuple[str, str], dict[str, float]]] = None
        self._srdf_path = Path(srdf_path) if srdf_path else self._default_srdf_path()
        self._goal_lock = threading.RLock()
        self._pending_goal_futures: set[Any] = set()
        self._active_goal_handles: dict[Any, Any] = {}
        self._cancel_requested = threading.Event()

        self.tf_buffer = tf_buffer or Buffer()
        self._tf_listener = (
            None if tf_buffer is not None else TransformListener(self.tf_buffer, node)
        )
        self._move_group_client = ActionClient(node, MoveGroup, move_action_name)
        self._ik_client = node.create_client(GetPositionIK, ik_service_name)
        self._joint_sub = node.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            10,
        )
        if not self._context_ok():
            self._log_error('ROS context is not available')
        elif not self._safe_action_ready(wait_time):
            self._log_error(f'MoveGroup action server not available: {move_action_name}')
        if self._context_ok() and not self._safe_service_ready(wait_time):
            self._log_error(f'IK service not available: {ik_service_name}')

    @property
    def velocity_scale(self) -> float:
        """現在の MoveIt 最大速度スケールを返す。"""
        return self._velocity_scale

    @velocity_scale.setter
    def velocity_scale(self, value: float) -> None:
        self._velocity_scale = self._validate_scale('velocity_scale', value)

    @property
    def acceleration_scale(self) -> float:
        """現在の MoveIt 最大加速度スケールを返す。"""
        return self._acceleration_scale

    @acceleration_scale.setter
    def acceleration_scale(self, value: float) -> None:
        self._acceleration_scale = self._validate_scale('acceleration_scale', value)

    def set_motion_scaling(
        self,
        velocity_scale: Optional[float] = None,
        acceleration_scale: Optional[float] = None,
    ) -> None:
        """MoveIt の速度・加速度スケールを動的に更新する。"""
        if velocity_scale is not None:
            self.velocity_scale = velocity_scale
        if acceleration_scale is not None:
            self.acceleration_scale = acceleration_scale

    def is_move_group_ready(self) -> bool:
        """MoveGroup action server が利用可能かを返す。"""
        return self._safe_action_ready(0.0)

    def is_ik_ready(self) -> bool:
        """MoveIt IK service が利用可能かを返す。"""
        return self._safe_service_ready(0.0)

    def wait_until_ready(
        self,
        timeout: float = 5.0,
        require_ik: bool = True,
    ) -> ArmControlResult:
        """必要な MoveIt action/service が利用可能になるまで待つ。"""
        timeout = self._validate_positive('timeout', timeout)
        deadline = time.monotonic() + timeout
        while self._context_ok() and time.monotonic() < deadline:
            if self.is_move_group_ready() and (not require_ik or self.is_ik_ready()):
                return self._result(ArmControlStatus.SUCCEEDED, 'MoveIt is ready')
            if not self._spin_once(self._SPIN_INTERVAL):
                break
        if not self._context_ok():
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        return self._result(ArmControlStatus.NOT_READY, 'MoveIt is not ready')

    def request_cancel(self) -> None:
        """現在の待機処理へキャンセル要求を通知する。"""
        self._cancel_requested.set()

    def reset_cancel_request(self) -> None:
        """前回のキャンセル要求を解除する。"""
        self._cancel_requested.clear()

    def cancel_current_goal(
        self,
        timeout: Optional[float] = None,
    ) -> ArmControlResult:
        """現在実行中の MoveGroup goal をキャンセルする。"""
        cancel_timeout = self.cancel_timeout if timeout is None else timeout
        try:
            cancel_timeout = self._validate_positive('timeout', cancel_timeout)
        except ValueError as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))
        self.request_cancel()
        if not self._context_ok():
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        with self._goal_lock:
            handles = list(self._active_goal_handles.values())
            has_pending = bool(self._pending_goal_futures)
        if not handles and not has_pending:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'No active goal')
        accepted = has_pending
        for handle in handles:
            try:
                future = handle.cancel_goal_async()
            except (RCLError, InvalidHandle):
                return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
            wait_status = self._wait_for_future(
                future, cancel_timeout, observe_cancel=False
            )
            if wait_status == _FutureWaitStatus.TIMED_OUT:
                return self._result(ArmControlStatus.TIMED_OUT, 'Cancel request timed out')
            if wait_status == _FutureWaitStatus.ROS_SHUTDOWN:
                return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
            response = future.result()
            if response is not None and response.goals_canceling:
                accepted = True
        if not accepted:
            return self._result(ArmControlStatus.GOAL_REJECTED, 'Cancel request rejected')
        return self._result(ArmControlStatus.CANCELLED, 'Cancellation requested')

    def cancel(self, wait: bool = True, timeout: float = 5.0) -> bool:
        """後方互換のため bool でキャンセル結果を返す。"""
        del wait
        result = self.cancel_current_goal(timeout)
        return result.status == ArmControlStatus.CANCELLED

    def get_current_joints_pose(self) -> dict[str, float]:
        """現在の関節角度のコピーを返す。"""
        with self._joint_state_lock:
            return self._joint_states.copy()

    def get_joint_state_age(self, joint_name: Optional[str] = None) -> Optional[float]:
        """指定 JointState の受信からの経過時間を返す。"""
        with self._joint_state_lock:
            if joint_name is not None:
                updated_at = self._joint_state_updated_at.get(joint_name)
                return None if updated_at is None else time.monotonic() - updated_at
            if not self._joint_state_updated_at:
                return None
            return max(time.monotonic() - value
                       for value in self._joint_state_updated_at.values())

    def wait_for_joint_states(
        self,
        planning_group: Optional[str] = None,
        timeout: float = 2.0,
        max_age: Optional[float] = None,
    ) -> ArmControlResult:
        """group に必要な新しい JointState が揃うまで待つ。"""
        group = planning_group or self.planning_group
        joints = self._joints_for_group(group)
        if joints is None:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Unsupported group')
        try:
            timeout = self._validate_positive('timeout', timeout)
            max_age = self.joint_state_max_age if max_age is None else max_age
            max_age = self._validate_positive('max_age', max_age)
        except ValueError as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))
        deadline = time.monotonic() + timeout
        while self._context_ok() and time.monotonic() < deadline:
            if self._fresh_joint_snapshot(joints, max_age)[0] is not None:
                return self._result(ArmControlStatus.SUCCEEDED, 'JointState is ready')
            if not self._spin_once(self._SPIN_INTERVAL):
                break
        if not self._context_ok():
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        _, missing = self._fresh_joint_snapshot(joints, max_age)
        return self._result(
            ArmControlStatus.STATE_UNAVAILABLE,
            f'JointState is missing or stale: {", ".join(missing)}',
        )

    def get_current_pose(
        self,
        simple: bool = True,
        reference_frame: Optional[str] = None,
        tip_link: Optional[str] = None,
        timeout: float = 2.0,
    ) -> Optional[PoseStamped | list[float]]:
        """現在のエンドエフェクタ pose を TF から取得する。"""
        if timeout < 0.0:
            self._log_error('timeout must not be negative')
            return None
        target_frame = reference_frame or self.base_frame
        source_frame = tip_link or self.tip_link
        if not target_frame or not source_frame:
            self._log_error('reference_frame and tip_link must not be empty')
            return None
        deadline = time.monotonic() + timeout
        while self._context_ok() and time.monotonic() <= deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                if not all(self._finite(value) for value in (
                    translation.x,
                    translation.y,
                    translation.z,
                )):
                    self._log_error('TF returned a non-finite translation')
                    return None
                if not self._valid_quaternion(rotation):
                    self._log_error('TF returned an invalid quaternion')
                    return None
                if simple:
                    roll, pitch, yaw = self._euler_from_quaternion(rotation)
                    return [translation.x, translation.y, translation.z, roll, pitch, yaw]
                pose = PoseStamped()
                pose.header.frame_id = target_frame
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.pose.position.x = translation.x
                pose.pose.position.y = translation.y
                pose.pose.position.z = translation.z
                pose.pose.orientation = self._normalize_quaternion(rotation)
                return pose
            except TransformException as error:
                self.node.get_logger().debug(
                    f'TF lookup failed for {source_frame}: {error}'
                )
                if time.monotonic() >= deadline:
                    break
                if not self._spin_once(self._SPIN_INTERVAL):
                    break
        self._log_error(f'Could not resolve TF from {target_frame} to {source_frame}')
        return None

    def move_to_pose(
        self,
        pose: Pose | PoseStamped,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        tip_link: Optional[str] = None,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
    ) -> bool:
        """後方互換の pose 移動 API。"""
        return self._to_legacy_bool(self.move_to_pose_detailed(
            pose,
            planning_group=planning_group,
            wait=wait,
            execute=execute,
            tip_link=tip_link,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
            allow_position_only_fallback=True,
        ))

    def move_to_pose_detailed(
        self,
        pose: Pose | PoseStamped,
        *,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        tip_link: Optional[str] = None,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
        allow_position_only_fallback: bool = False,
    ) -> ArmControlResult:
        """構造化結果で目標 pose へ移動する。"""
        group = planning_group or self.planning_group
        link = tip_link or self.tip_link
        target, validation = self._as_valid_pose_stamped(pose)
        if validation is not None:
            return validation
        validation = self._validate_motion_arguments(
            group, planning_attempts, planning_time, wait, allow_position_only_fallback
        )
        if validation is not None:
            return validation
        ik = self._solve_ik_detailed(target, group, link)
        if ik.status == IkStatus.SUCCEEDED:
            result = self.joint_control_detailed(
                planning_group=group,
                wait=wait,
                execute=execute,
                planning_attempts=planning_attempts,
                planning_time=planning_time,
                **(ik.joints or {}),
            )
            if result.status not in {
                ArmControlStatus.MOVEIT_FAILED,
                ArmControlStatus.GOAL_REJECTED,
            }:
                return replace(result, used_ik=True)
        elif ik.status == IkStatus.NO_SOLUTION:
            pass
        elif ik.status == IkStatus.SERVICE_UNAVAILABLE:
            return self._result(ArmControlStatus.NOT_READY, ik.message)
        elif ik.status == IkStatus.TIMED_OUT:
            return self._result(ArmControlStatus.TIMED_OUT, ik.message)
        elif ik.status == IkStatus.STATE_UNAVAILABLE:
            return self._result(ArmControlStatus.STATE_UNAVAILABLE, ik.message)
        elif ik.status == IkStatus.ROS_SHUTDOWN:
            return self._result(ArmControlStatus.ROS_SHUTDOWN, ik.message)
        else:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, ik.message)

        full_goal = self._pose_goal(
            target, group, link, planning_attempts, planning_time, True
        )
        result = self._send_move_group_goal_detailed(
            full_goal, wait=wait, execute=execute
        )
        if result.succeeded or result.status == ArmControlStatus.SUBMITTED:
            return result
        if not allow_position_only_fallback:
            return result
        position_goal = self._pose_goal(
            target, group, link, planning_attempts, planning_time, False
        )
        result = self._send_move_group_goal_detailed(
            position_goal, wait=wait, execute=execute
        )
        if result.succeeded:
            return replace(
                result,
                status=ArmControlStatus.DEGRADED,
                message='Reached target position without target orientation',
                used_position_only_fallback=True,
            )
        return result

    def move_abs(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        reference_frame: Optional[str] = None,
        tip_link: Optional[str] = None,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
    ) -> bool:
        """後方互換の絶対座標移動 API。"""
        return self._to_legacy_bool(self.move_abs_detailed(
            x, y, z, roll, pitch, yaw,
            planning_group=planning_group,
            wait=wait,
            execute=execute,
            reference_frame=reference_frame,
            tip_link=tip_link,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
            allow_position_only_fallback=True,
        ))

    def move_abs_detailed(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        *,
        reference_frame: Optional[str] = None,
        **kwargs: Any,
    ) -> ArmControlResult:
        """構造化結果で絶対座標指定の pose 移動を行う。"""
        values = (x, y, z, roll, pitch, yaw)
        if not all(self._finite(value) for value in values):
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Pose is not finite')
        pose = PoseStamped()
        pose.header.frame_id = reference_frame or self.base_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation = self._quaternion_from_euler(roll, pitch, yaw)
        return self.move_to_pose_detailed(pose, **kwargs)

    def move_rel(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        reference_frame: Optional[str] = None,
        tip_link: Optional[str] = None,
        pose_timeout: float = 2.0,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
    ) -> bool:
        """後方互換の相対座標移動 API。"""
        return self._to_legacy_bool(self.move_rel_detailed(
            x, y, z, roll, pitch, yaw,
            planning_group=planning_group,
            wait=wait,
            execute=execute,
            reference_frame=reference_frame,
            tip_link=tip_link,
            pose_timeout=pose_timeout,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
            allow_position_only_fallback=True,
        ))

    def move_rel_detailed(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        *,
        reference_frame: Optional[str] = None,
        tip_link: Optional[str] = None,
        pose_timeout: float = 2.0,
        **kwargs: Any,
    ) -> ArmControlResult:
        """構造化結果で基準座標系上の相対 pose 移動を行う。"""
        values = (x, y, z, roll, pitch, yaw)
        if not all(self._finite(value) for value in values):
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Pose is not finite')
        current = self.get_current_pose(
            simple=False,
            reference_frame=reference_frame,
            tip_link=tip_link,
            timeout=pose_timeout,
        )
        if current is None:
            return self._result(ArmControlStatus.TF_UNAVAILABLE, 'Current pose unavailable')
        target = PoseStamped()
        target.header.frame_id = current.header.frame_id
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x = current.pose.position.x + float(x)
        target.pose.position.y = current.pose.position.y + float(y)
        target.pose.position.z = current.pose.position.z + float(z)
        delta = self._quaternion_from_euler(roll, pitch, yaw)
        target.pose.orientation = self._normalize_quaternion(
            self._multiply_quaternions(current.pose.orientation, delta)
        )
        if tip_link is not None:
            kwargs['tip_link'] = tip_link
        return self.move_to_pose_detailed(target, **kwargs)

    def joint_control(
        self,
        rel: bool = False,
        wait: bool = True,
        execute: bool = True,
        planning_group: Optional[str] = None,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
        **joint_values: float,
    ) -> bool:
        """後方互換の関節制御 API。"""
        return self._to_legacy_bool(self.joint_control_detailed(
            rel=rel,
            wait=wait,
            execute=execute,
            planning_group=planning_group,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
            **joint_values,
        ))

    def joint_control_detailed(
        self,
        *,
        rel: bool = False,
        wait: bool = True,
        execute: bool = True,
        planning_group: Optional[str] = None,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
        **joint_values: float,
    ) -> ArmControlResult:
        """構造化結果で関節角度を指定して移動する。"""
        group = planning_group or self.planning_group
        validation = self._validate_motion_arguments(group, planning_attempts, planning_time)
        if validation is not None:
            return validation
        if not joint_values:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'No joint target')
        joints = self._joints_for_group(group)
        if joints is None:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Unsupported group')
        unknown = sorted(set(joint_values).difference(joints))
        if unknown:
            return self._result(
                ArmControlStatus.INVALID_ARGUMENT,
                f'Unsupported joints for {group}: {", ".join(unknown)}',
            )
        try:
            targets = {name: float(value) for name, value in joint_values.items()}
        except (TypeError, ValueError):
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Invalid joint value')
        if not all(self._finite(value) for value in targets.values()):
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Joint value is not finite')
        needs_state = rel or set(targets) != set(joints)
        current: dict[str, float] = {}
        if needs_state:
            current, missing = self._fresh_joint_snapshot(joints, self.joint_state_max_age)
            if current is None:
                return self._result(
                    ArmControlStatus.STATE_UNAVAILABLE,
                    f'JointState is missing or stale: {", ".join(missing)}',
                )
        constraints = Constraints()
        for name in joints:
            target = targets.get(name, current.get(name))
            if rel and name in targets:
                target = current[name] + targets[name]
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target
            constraint.tolerance_above = self.joint_tolerance
            constraint.tolerance_below = self.joint_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = planning_attempts
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scale
        goal.request.max_acceleration_scaling_factor = self.acceleration_scale
        goal.request.goal_constraints.append(constraints)
        return self._send_move_group_goal_detailed(goal, wait=wait, execute=execute)

    def move_groupstate(
        self,
        group_state: str,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
    ) -> bool:
        """後方互換の SRDF group state 移動 API。"""
        return self._to_legacy_bool(self.move_groupstate_detailed(
            group_state,
            planning_group=planning_group,
            wait=wait,
            execute=execute,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
        ))

    def move_groupstate_detailed(
        self,
        group_state: str,
        *,
        planning_group: Optional[str] = None,
        wait: bool = True,
        execute: bool = True,
        planning_attempts: int = 10,
        planning_time: float = 5.0,
    ) -> ArmControlResult:
        """構造化結果で SRDF の group state へ移動する。"""
        group = planning_group or self.planning_group
        joints = self.get_group_state(group_state, group)
        if joints is None:
            return self._result(
                ArmControlStatus.INVALID_ARGUMENT,
                f"SRDF group_state '{group_state}' for group '{group}' was not found",
            )
        return self.joint_control_detailed(
            planning_group=group,
            wait=wait,
            execute=execute,
            planning_attempts=planning_attempts,
            planning_time=planning_time,
            **joints,
        )

    def gripper_control(
        self,
        position: float,
        wait: bool = True,
        execute: bool = True,
    ) -> bool:
        """後方互換のグリッパー制御 API。"""
        return self._to_legacy_bool(self.gripper_control_detailed(
            position, wait=wait, execute=execute
        ))

    def gripper_control_detailed(
        self,
        position: float,
        **kwargs: Any,
    ) -> ArmControlResult:
        """構造化結果でグリッパー角度を指定して移動する。"""
        return self.joint_control_detailed(
            planning_group=self.gripper_group,
            gripper=position,
            **kwargs,
        )

    def open_gripper(self, wait: bool = True, execute: bool = True) -> bool:
        """後方互換のグリッパー開放 API。"""
        return self._to_legacy_bool(
            self.open_gripper_detailed(wait=wait, execute=execute)
        )

    def open_gripper_detailed(self, **kwargs: Any) -> ArmControlResult:
        """構造化結果で SRDF の open 状態へ移動する。"""
        return self.move_groupstate_detailed(
            'open', planning_group=self.gripper_group, **kwargs
        )

    def close_gripper(self, wait: bool = True, execute: bool = True) -> bool:
        """後方互換のグリッパー閉鎖 API。"""
        return self._to_legacy_bool(
            self.close_gripper_detailed(wait=wait, execute=execute)
        )

    def close_gripper_detailed(self, **kwargs: Any) -> ArmControlResult:
        """構造化結果で SRDF の close 状態へ移動する。"""
        return self.move_groupstate_detailed(
            'close', planning_group=self.gripper_group, **kwargs
        )

    def list_group_states(
        self,
        planning_group: Optional[str] = None,
    ) -> tuple[str, ...]:
        """指定 group で利用可能な SRDF group state 名を返す。"""
        states = self._load_srdf_group_states()
        if states is None:
            return ()
        group = planning_group or self.planning_group
        return tuple(sorted(state for (name, state) in states if name == group))

    def get_group_state(
        self,
        group_state: str,
        planning_group: Optional[str] = None,
    ) -> Optional[dict[str, float]]:
        """SRDF group state の関節値コピーを返す。"""
        states = self._load_srdf_group_states()
        if states is None:
            return None
        group = planning_group or self.planning_group
        joints = states.get((group, group_state))
        return None if joints is None else joints.copy()

    def reload_group_states(self) -> ArmControlResult:
        """SRDF キャッシュを破棄して再読み込みする。"""
        self._srdf_group_states = None
        if self._load_srdf_group_states() is None:
            return self._result(ArmControlStatus.INTERNAL_ERROR, 'Failed to load SRDF')
        return self._result(ArmControlStatus.SUCCEEDED, 'SRDF group states reloaded')

    def get_supported_joints(
        self,
        planning_group: Optional[str] = None,
    ) -> tuple[str, ...]:
        """指定 group に属する関節名を返す。"""
        joints = self._joints_for_group(planning_group or self.planning_group)
        return () if joints is None else joints

    def _joint_state_callback(self, message: JointState) -> None:
        """JointState 値と受信時刻を関節ごとに保存する。"""
        if len(message.name) != len(message.position):
            self._log_warn('JointState name and position lengths do not match')
            return
        received_at = time.monotonic()
        with self._joint_state_lock:
            for name, position in zip(message.name, message.position):
                if name and self._finite(position):
                    self._joint_states[name] = float(position)
                    self._joint_state_updated_at[name] = received_at

    def _solve_ik_detailed(
        self,
        pose: PoseStamped,
        planning_group: str,
        tip_link: str,
    ) -> IkResult:
        """IK service の失敗理由を区別して関節解を取得する。"""
        if not self._context_ok():
            return IkResult(IkStatus.ROS_SHUTDOWN, message='ROS is shutting down')
        joints = self._joints_for_group(planning_group)
        if joints is None:
            return IkResult(IkStatus.INVALID_REQUEST, message='Unsupported group')
        current, missing = self._fresh_joint_snapshot(joints, self.joint_state_max_age)
        if current is None:
            return IkResult(
                IkStatus.STATE_UNAVAILABLE,
                message=f'JointState is missing or stale: {", ".join(missing)}',
            )
        if not self.is_ik_ready():
            return IkResult(IkStatus.SERVICE_UNAVAILABLE, message='IK service unavailable')
        request = GetPositionIK.Request()
        request.ik_request.group_name = planning_group
        request.ik_request.ik_link_name = tip_link
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout.sec = 1
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.joint_state.name = list(current)
        request.ik_request.robot_state.joint_state.position = list(current.values())
        try:
            future = self._ik_client.call_async(request)
        except (RCLError, InvalidHandle):
            return IkResult(IkStatus.ROS_SHUTDOWN, message='ROS is shutting down')
        wait_status = self._wait_for_future(future, self.goal_response_timeout)
        if wait_status == _FutureWaitStatus.TIMED_OUT:
            return IkResult(IkStatus.TIMED_OUT, message='IK service call timed out')
        if wait_status == _FutureWaitStatus.ROS_SHUTDOWN:
            return IkResult(IkStatus.ROS_SHUTDOWN, message='ROS is shutting down')
        if wait_status != _FutureWaitStatus.COMPLETED or future.result() is None:
            return IkResult(IkStatus.INVALID_RESPONSE, message='IK response unavailable')
        response = future.result()
        code = response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return IkResult(
                IkStatus.NO_SOLUTION,
                message=f'IK failed with error code: {code}',
                moveit_error_code=code,
            )
        solution = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        missing = [name for name in joints if name not in solution]
        if missing:
            return IkResult(
                IkStatus.INVALID_RESPONSE,
                message=f'IK solution is missing joints: {", ".join(missing)}',
            )
        return IkResult(
            IkStatus.SUCCEEDED,
            joints={name: solution[name] for name in joints},
        )

    def _send_move_group_goal_detailed(
        self,
        goal: MoveGroup.Goal,
        *,
        wait: bool,
        execute: bool,
        goal_response_timeout: Optional[float] = None,
        result_timeout: Optional[float] = None,
        cancel_timeout: Optional[float] = None,
        cancel_event: Optional[threading.Event] = None,
    ) -> ArmControlResult:
        """期限付きで MoveGroup goal を送信・監視する。"""
        try:
            response_timeout = self._resolve_timeout(
                'goal_response_timeout', goal_response_timeout,
                self.goal_response_timeout,
            )
            result_limit = self._resolve_timeout(
                'result_timeout', result_timeout, self.result_timeout
            )
            cancel_limit = self._resolve_timeout(
                'cancel_timeout', cancel_timeout, self.cancel_timeout
            )
        except ValueError as error:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, str(error))
        if not self._context_ok():
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        if not self.is_move_group_ready():
            return self._result(ArmControlStatus.NOT_READY, 'MoveGroup action unavailable')
        if self._cancel_requested.is_set() or (
            cancel_event is not None and cancel_event.is_set()
        ):
            return self._result(ArmControlStatus.CANCELLED, 'Goal cancellation requested')
        goal.planning_options.plan_only = not execute
        try:
            future = self._move_group_client.send_goal_async(goal)
        except (RCLError, InvalidHandle):
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        self._track_pending_goal(future)
        if not wait:
            future.add_done_callback(self._register_async_goal_handle)
            return self._result(
                ArmControlStatus.SUBMITTED,
                'MoveGroup goal submitted',
                plan_only=not execute,
            )
        try:
            status = self._wait_for_future(
                future, response_timeout, cancel_event=cancel_event
            )
            if status == _FutureWaitStatus.CANCELLED:
                future.add_done_callback(self._register_async_goal_handle)
                return self._result(ArmControlStatus.CANCELLED, 'Goal cancellation requested')
            if status == _FutureWaitStatus.TIMED_OUT:
                self.request_cancel()
                future.add_done_callback(self._register_async_goal_handle)
                return self._result(ArmControlStatus.TIMED_OUT, 'Goal response timed out')
            if status == _FutureWaitStatus.ROS_SHUTDOWN:
                return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
            handle = future.result()
            if handle is None or not handle.accepted:
                return self._result(ArmControlStatus.GOAL_REJECTED, 'MoveGroup goal rejected')
            self._track_active_goal(handle)
            result_future = handle.get_result_async()
            status = self._wait_for_future(
                result_future, result_limit, cancel_event=cancel_event, goal_handle=handle,
                cancel_timeout=cancel_limit,
            )
            if status == _FutureWaitStatus.CANCELLED:
                return self._result(ArmControlStatus.CANCELLED, 'MoveGroup goal cancelled')
            if status == _FutureWaitStatus.TIMED_OUT:
                self._cancel_goal_handle(handle, cancel_limit)
                return self._result(ArmControlStatus.TIMED_OUT, 'MoveGroup result timed out')
            if status == _FutureWaitStatus.ROS_SHUTDOWN:
                return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
            wrapped = result_future.result()
            return self._move_group_result(wrapped, plan_only=not execute)
        except KeyboardInterrupt:
            self.request_cancel()
            if self._context_ok() and 'handle' not in locals():
                future.add_done_callback(self._register_async_goal_handle)
            if self._context_ok():
                self._cancel_all_active_goals(cancel_limit)
            raise
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return self._result(ArmControlStatus.ROS_SHUTDOWN, 'ROS is shutting down')
        except Exception as error:
            self._log_error(f'MoveGroup action failed internally: {error}')
            return self._result(ArmControlStatus.INTERNAL_ERROR, str(error))
        finally:
            self._untrack_pending_goal(future)
            if 'handle' in locals():
                self._untrack_active_goal(handle)

    def _send_move_group_goal(
        self,
        goal: MoveGroup.Goal,
        wait: bool,
        execute: bool,
    ) -> bool:
        """旧内部 bool API を詳細実装へ転送する。"""
        return self._to_legacy_bool(
            self._send_move_group_goal_detailed(goal, wait=wait, execute=execute)
        )

    def _wait_for_future(
        self,
        future: Any,
        timeout: float,
        *,
        cancel_event: Optional[threading.Event] = None,
        goal_handle: Optional[Any] = None,
        cancel_timeout: Optional[float] = None,
        observe_cancel: bool = True,
    ) -> _FutureWaitStatus:
        """future を短い spin で待ち、timeout と cancel を監視する。"""
        deadline = time.monotonic() + timeout
        while self._context_ok() and not future.done():
            if observe_cancel and (self._cancel_requested.is_set() or (
                cancel_event is not None and cancel_event.is_set()
            )):
                if goal_handle is not None:
                    self._cancel_goal_handle(goal_handle, cancel_timeout or self.cancel_timeout)
                return _FutureWaitStatus.CANCELLED
            if time.monotonic() >= deadline:
                return _FutureWaitStatus.TIMED_OUT
            if not self._spin_once(
                min(self._SPIN_INTERVAL, max(0.0, deadline - time.monotonic()))
            ):
                return _FutureWaitStatus.ROS_SHUTDOWN
        if not self._context_ok():
            return _FutureWaitStatus.ROS_SHUTDOWN
        return _FutureWaitStatus.COMPLETED

    def _cancel_goal_handle(self, handle: Any, timeout: float) -> bool:
        """一つの受理済み goal に cancel request を送る。"""
        if not self._context_ok():
            return False
        try:
            future = handle.cancel_goal_async()
        except (RCLError, InvalidHandle):
            return False
        status = self._wait_for_future(future, timeout, observe_cancel=False)
        if status != _FutureWaitStatus.COMPLETED or future.result() is None:
            return False
        return bool(future.result().goals_canceling)

    def _cancel_all_active_goals(self, timeout: float) -> None:
        """追跡中の受理済み goal すべてへ cancel request を送る。"""
        with self._goal_lock:
            handles = list(self._active_goal_handles.values())
        for handle in handles:
            self._cancel_goal_handle(handle, timeout)

    def _register_async_goal_handle(self, future: Any) -> None:
        """非同期送信済み future を goal handle として登録する。"""
        self._untrack_pending_goal(future)
        if not self._context_ok():
            return
        try:
            handle = future.result()
        except Exception as error:
            self._log_error(f'Failed to receive MoveGroup goal handle: {error}')
            return
        if handle is None or not handle.accepted:
            self._log_error('MoveGroup goal rejected')
            return
        self._track_active_goal(handle)
        try:
            result_future = handle.get_result_async()
        except (RCLError, InvalidHandle):
            self._untrack_active_goal(handle)
            return
        result_future.add_done_callback(lambda _: self._untrack_active_goal(handle))
        if self._cancel_requested.is_set() and self._context_ok():
            try:
                handle.cancel_goal_async()
            except (RCLError, InvalidHandle):
                self._untrack_active_goal(handle)

    def _track_pending_goal(self, future: Any) -> None:
        """goal response future を追跡する。"""
        with self._goal_lock:
            self._pending_goal_futures.add(future)

    def _untrack_pending_goal(self, future: Any) -> None:
        """goal response future の追跡を解除する。"""
        with self._goal_lock:
            self._pending_goal_futures.discard(future)

    def _track_active_goal(self, handle: Any) -> None:
        """受理済み goal handle を追跡する。"""
        with self._goal_lock:
            self._active_goal_handles[self._goal_key(handle)] = handle

    def _untrack_active_goal(self, handle: Any) -> None:
        """受理済み goal handle の追跡を解除する。"""
        with self._goal_lock:
            self._active_goal_handles.pop(self._goal_key(handle), None)

    def _load_srdf_group_states(
        self,
    ) -> Optional[dict[tuple[str, str], dict[str, float]]]:
        """検証済み SRDF group state をキャッシュして読み込む。"""
        if self._srdf_group_states is not None:
            return self._srdf_group_states
        try:
            root = ET.parse(self._srdf_path).getroot()
        except (OSError, ET.ParseError) as error:
            self._log_error(f"Failed to load SRDF '{self._srdf_path}': {error}")
            return None
        states: dict[tuple[str, str], dict[str, float]] = {}
        for state in root.findall('group_state'):
            group = state.get('group')
            name = state.get('name')
            allowed = self._joints_for_group(group or '')
            if not group or not name or allowed is None or (group, name) in states:
                self._log_error('Invalid or duplicate SRDF group_state')
                return None
            joints: dict[str, float] = {}
            for joint in state.findall('joint'):
                joint_name = joint.get('name')
                value = joint.get('value')
                if joint_name in joints or joint_name not in allowed or value is None:
                    self._log_error(f'Invalid SRDF joint in {group}/{name}')
                    return None
                try:
                    numeric_value = float(value)
                except ValueError:
                    self._log_error(f'Invalid SRDF value in {group}/{name}')
                    return None
                if not self._finite(numeric_value):
                    self._log_error(f'Non-finite SRDF value in {group}/{name}')
                    return None
                joints[joint_name] = numeric_value
            if not joints:
                self._log_error(f'Empty SRDF group_state {group}/{name}')
                return None
            states[(group, name)] = joints
        self._srdf_group_states = states
        return states

    def _pose_goal(
        self,
        pose: PoseStamped,
        planning_group: str,
        tip_link: str,
        planning_attempts: int,
        planning_time: float,
        include_orientation: bool,
    ) -> MoveGroup.Goal:
        """pose constraint を含む MoveGroup goal を構築する。"""
        constraint = Constraints()
        position = PositionConstraint()
        position.header = pose.header
        position.link_name = tip_link
        position.constraint_region.primitive_poses.append(pose.pose)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0 * self.position_tolerance] * 3
        position.constraint_region.primitives.append(box)
        position.weight = 1.0
        constraint.position_constraints.append(position)
        if include_orientation:
            orientation = OrientationConstraint()
            orientation.header = pose.header
            orientation.link_name = tip_link
            orientation.orientation = pose.pose.orientation
            orientation.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation.absolute_z_axis_tolerance = self.orientation_tolerance
            orientation.weight = 1.0
            constraint.orientation_constraints.append(orientation)
        goal = MoveGroup.Goal()
        goal.request.group_name = planning_group
        goal.request.num_planning_attempts = planning_attempts
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scale
        goal.request.max_acceleration_scaling_factor = self.acceleration_scale
        goal.request.goal_constraints.append(constraint)
        return goal

    def _fresh_joint_snapshot(
        self,
        joints: tuple[str, ...],
        max_age: float,
    ) -> tuple[Optional[dict[str, float]], list[str]]:
        """指定関節の新しい状態コピー、または不足関節を返す。"""
        now = time.monotonic()
        with self._joint_state_lock:
            missing = [
                name for name in joints
                if name not in self._joint_states
                or name not in self._joint_state_updated_at
                or now - self._joint_state_updated_at[name] > max_age
            ]
            if missing:
                return None, missing
            return {name: self._joint_states[name] for name in joints}, []

    def _validate_motion_arguments(
        self,
        planning_group: str,
        planning_attempts: int,
        planning_time: float,
        wait: bool = True,
        allow_position_only_fallback: bool = False,
    ) -> Optional[ArmControlResult]:
        """共通の MoveIt planning 引数を検証する。"""
        if self._joints_for_group(planning_group) is None:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Unsupported group')
        if not isinstance(planning_attempts, int) or planning_attempts < 1:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Invalid planning_attempts')
        if not self._finite(planning_time) or planning_time <= 0.0:
            return self._result(ArmControlStatus.INVALID_ARGUMENT, 'Invalid planning_time')
        if not wait and allow_position_only_fallback:
            return self._result(
                ArmControlStatus.INVALID_ARGUMENT,
                'Position-only fallback requires wait=True',
            )
        return None

    def _as_valid_pose_stamped(
        self,
        pose: Pose | PoseStamped,
    ) -> tuple[Optional[PoseStamped], Optional[ArmControlResult]]:
        """Pose を検証済み・正規化済み PoseStamped へ変換する。"""
        source = pose.pose if isinstance(pose, PoseStamped) else pose
        frame = pose.header.frame_id if isinstance(pose, PoseStamped) else self.base_frame
        if not frame:
            return None, self._result(ArmControlStatus.INVALID_ARGUMENT, 'Empty frame')
        if not all(self._finite(value) for value in (
            source.position.x, source.position.y, source.position.z,
        )):
            return None, self._result(ArmControlStatus.INVALID_ARGUMENT, 'Pose is not finite')
        if not self._valid_quaternion(source.orientation):
            return None, self._result(ArmControlStatus.INVALID_ARGUMENT, 'Invalid quaternion')
        target = PoseStamped()
        target.header.frame_id = frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x = source.position.x
        target.pose.position.y = source.position.y
        target.pose.position.z = source.position.z
        target.pose.orientation = self._normalize_quaternion(source.orientation)
        return target, None

    def _move_group_result(self, wrapped: Any, *, plan_only: bool) -> ArmControlResult:
        """MoveGroup の action 結果を ArmControlResult へ変換する。"""
        if wrapped is None or wrapped.result is None:
            return self._result(ArmControlStatus.INTERNAL_ERROR, 'Missing MoveGroup result')
        action_status = getattr(wrapped, 'status', None)
        error_code = wrapped.result.error_code.val
        error_name = self._moveit_error_name(error_code)
        if action_status == GoalStatus.STATUS_CANCELED:
            return self._result(
                ArmControlStatus.CANCELLED, 'MoveGroup goal cancelled', error_code,
                error_name, action_status, plan_only,
            )
        if action_status not in (None, GoalStatus.STATUS_SUCCEEDED):
            return self._result(
                ArmControlStatus.MOVEIT_FAILED,
                f'MoveGroup action failed with status: {action_status}',
                error_code,
                error_name,
                action_status,
                plan_only,
            )
        if error_code == MoveItErrorCodes.SUCCESS:
            return self._result(
                ArmControlStatus.SUCCEEDED, 'MoveGroup succeeded', error_code,
                error_name, action_status, plan_only,
            )
        return self._result(
            ArmControlStatus.MOVEIT_FAILED, f'MoveGroup failed: {error_name}',
            error_code, error_name, action_status, plan_only,
        )

    def _result(
        self,
        status: ArmControlStatus,
        message: str,
        moveit_error_code: Optional[int] = None,
        moveit_error_name: Optional[str] = None,
        action_status: Optional[int] = None,
        plan_only: bool = False,
    ) -> ArmControlResult:
        """ArmControlResult を生成する。"""
        return ArmControlResult(
            status, message, moveit_error_code, moveit_error_name, action_status,
            plan_only,
        )

    def _joints_for_group(self, planning_group: str) -> Optional[tuple[str, ...]]:
        """SO-101 の planning group に属する関節列を返す。"""
        groups = {
            self.planning_group: self._ARM_JOINTS,
            self.gripper_group: self._GRIPPER_JOINTS,
            'so101': self._ARM_JOINTS + self._GRIPPER_JOINTS,
        }
        return groups.get(planning_group)

    def _default_srdf_path(self) -> Path:
        """既定の SO-101 SRDF パスを返す。"""
        try:
            share = get_package_share_directory('nakalab_so101_moveit')
        except PackageNotFoundError:
            return Path('nakalab_so101.srdf')
        return Path(share) / 'config' / 'nakalab_so101.srdf'

    def _log_error(self, message: str) -> None:
        """ノード logger に error を出力する。"""
        self.node.get_logger().error(message)

    def _log_warn(self, message: str) -> None:
        """ノード logger に warning を出力する。"""
        self.node.get_logger().warn(message)

    def _context_ok(self) -> bool:
        """ROS context が安全に利用できるかを返す。"""
        try:
            return bool(self.node.context.ok())
        except (AttributeError, ExternalShutdownException, RCLError, InvalidHandle):
            return False

    def _safe_action_ready(self, timeout: float) -> bool:
        """無効な context に触れず MoveGroup readiness を取得する。"""
        if not self._context_ok():
            return False
        try:
            return bool(self._move_group_client.wait_for_server(timeout_sec=timeout))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False

    def _safe_service_ready(self, timeout: float) -> bool:
        """無効な context に触れず IK readiness を取得する。"""
        if not self._context_ok():
            return False
        try:
            return bool(self._ik_client.wait_for_service(timeout_sec=timeout))
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False

    def _spin_once(self, timeout: float) -> bool:
        """context shutdown を通常の終了条件として扱って一度 spin する。"""
        if not self._context_ok():
            return False
        try:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, timeout))
            return self._context_ok()
        except KeyboardInterrupt:
            raise
        except (ExternalShutdownException, RCLError, InvalidHandle):
            return False

    @staticmethod
    def _to_legacy_bool(result: ArmControlResult) -> bool:
        """詳細結果を既存 API 互換の bool へ変換する。"""
        return result.status in {
            ArmControlStatus.SUCCEEDED,
            ArmControlStatus.DEGRADED,
            ArmControlStatus.SUBMITTED,
        }

    @staticmethod
    def _finite(value: Any) -> bool:
        """値が有限 float に変換可能かを返す。"""
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _validate_positive(name: str, value: float) -> float:
        """正の有限値を検証する。"""
        numeric = float(value)
        if not math.isfinite(numeric) or numeric <= 0.0:
            raise ValueError(f'{name} must be a positive finite value')
        return numeric

    @staticmethod
    def _validate_scale(name: str, value: float) -> float:
        """MoveIt scaling factor の範囲を検証する。"""
        scale = float(value)
        if not math.isfinite(scale) or not 0.0 < scale <= 1.0:
            raise ValueError(f'{name} must be greater than 0.0 and at most 1.0')
        return scale

    def _resolve_timeout(self, name: str, value: Optional[float], default: float) -> float:
        """省略可能な timeout を検証済み float へ変換する。"""
        return self._validate_positive(name, default if value is None else value)

    @staticmethod
    def _goal_key(handle: Any) -> Any:
        """goal handle の安定した辞書キーを返す。"""
        goal_id = getattr(handle, 'goal_id', None)
        uuid = getattr(goal_id, 'uuid', None)
        return bytes(uuid) if uuid is not None else id(handle)

    @staticmethod
    def _moveit_error_name(code: int) -> str:
        """MoveIt error code を定数名へ変換する。"""
        for name in dir(MoveItErrorCodes):
            if name.isupper() and getattr(MoveItErrorCodes, name) == code:
                return name
        return f'UNKNOWN_{code}'

    @staticmethod
    def _valid_quaternion(quaternion: Quaternion) -> bool:
        """Quaternion が有限でゼロ長でないかを返す。"""
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        return all(math.isfinite(value) for value in values) and sum(
            value * value for value in values
        ) > 1e-12

    @staticmethod
    def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
        """Quaternion を単位長へ正規化する。"""
        norm = math.sqrt(
            quaternion.x ** 2 + quaternion.y ** 2
            + quaternion.z ** 2 + quaternion.w ** 2
        )
        result = Quaternion()
        result.x = quaternion.x / norm
        result.y = quaternion.y / norm
        result.z = quaternion.z / norm
        result.w = quaternion.w / norm
        return result

    @staticmethod
    def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
        """roll、pitch、yaw から Quaternion を生成する。"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        result = Quaternion()
        result.w = cr * cp * cy + sr * sp * sy
        result.x = sr * cp * cy - cr * sp * sy
        result.y = cr * sp * cy + sr * cp * sy
        result.z = cr * cp * sy - sr * sp * cy
        return result

    @staticmethod
    def _euler_from_quaternion(quaternion: Quaternion) -> tuple[float, float, float]:
        """Quaternion から roll、pitch、yaw を生成する。"""
        sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
        cosr_cosp = 1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _multiply_quaternions(first: Quaternion, second: Quaternion) -> Quaternion:
        """二つの Quaternion を乗算する。"""
        result = Quaternion()
        result.w = (
            first.w * second.w - first.x * second.x
            - first.y * second.y - first.z * second.z
        )
        result.x = (
            first.w * second.x + first.x * second.w
            + first.y * second.z - first.z * second.y
        )
        result.y = (
            first.w * second.y - first.x * second.z
            + first.y * second.w + first.z * second.x
        )
        result.z = (
            first.w * second.z + first.x * second.y
            - first.y * second.x + first.z * second.w
        )
        return result
