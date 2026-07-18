import math
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Quaternion
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity


JOINTS = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'gripper',
]


@pytest.mark.launch_test
def generate_test_description():
    package_share = get_package_share_directory('nakalab_so101_moveit')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{package_share}/launch/moveit.launch.py'
            ),
            launch_arguments={
                'use_sim_hardware': 'true',
                'launch_rviz': 'false',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestMoveItSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('nakalab_so101_moveit_launch_test')
        cls.joint_state = None
        cls.node.create_subscription(JointState, '/joint_states', cls._joint_state_callback, 10)
        cls.list_controllers = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        cls.fk = cls.node.create_client(GetPositionFK, '/compute_fk')
        cls.ik = cls.node.create_client(GetPositionIK, '/compute_ik')
        cls.state_validity = cls.node.create_client(
            GetStateValidity, '/check_state_validity'
        )
        cls.move_group = ActionClient(cls.node, MoveGroup, '/move_action')
        cls.execute = ActionClient(cls.node, ExecuteTrajectory, '/execute_trajectory')
        cls.follower_controller = ActionClient(
            cls.node,
            FollowJointTrajectory,
            '/so101_follower_controller/follow_joint_trajectory',
        )
        cls.gripper_controller = ActionClient(
            cls.node,
            FollowJointTrajectory,
            '/so101_gripper_controller/follow_joint_trajectory',
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _joint_state_callback(cls, message):
        cls.joint_state = message

    def wait_for(self, predicate, description, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f'Timed out waiting for {description}')

    def call(self, client, request, description, timeout=15.0):
        self.wait_for(lambda: client.service_is_ready(), description, timeout)
        future = client.call_async(request)
        self.wait_for(lambda: future.done(), description, timeout)
        self.assertIsNotNone(future.result(), description)
        return future.result()

    def action(self, client, goal, description, timeout=30.0):
        self.wait_for(lambda: client.server_is_ready(), description, timeout)
        send_future = client.send_goal_async(goal)
        self.wait_for(lambda: send_future.done(), description, timeout)
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted, description)
        result_future = goal_handle.get_result_async()
        self.wait_for(lambda: result_future.done(), description, timeout)
        return result_future.result().result

    @staticmethod
    def robot_state(values):
        state = RobotState()
        state.joint_state.name = list(values)
        state.joint_state.position = [values[name] for name in values]
        return state

    def current_positions(self):
        self.assertIsNotNone(self.joint_state)
        return dict(zip(self.joint_state.name, self.joint_state.position))

    def wait_for_joint_target(self, target):
        self.wait_for(
            lambda: all(
                name in self.current_positions()
                and abs(self.current_positions()[name] - value) <= 0.01
                for name, value in target.items()
            ),
            f'joint target {target}',
        )

    def plan(self, group, target):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.pipeline_id = 'ompl'
        request.planner_id = 'RRTConnectkConfigDefault'
        request.num_planning_attempts = 1
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        constraints = Constraints()
        for name, value in target.items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = value
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        result = self.action(self.move_group, goal, f'plan {group}')
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)
        trajectory = result.planned_trajectory.joint_trajectory
        self.assertGreater(len(trajectory.joint_names), 0)
        self.assertGreater(len(trajectory.points), 1)
        previous_time = -1.0
        for point in trajectory.points:
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            self.assertGreater(point_time, previous_time)
            previous_time = point_time
        final = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        for name, value in target.items():
            self.assertAlmostEqual(final[name], value, delta=0.01)
        return result.planned_trajectory

    def execute_trajectory(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        result = self.action(self.execute, goal, 'execute trajectory')
        self.assertEqual(result.error_code.val, MoveItErrorCodes.SUCCESS)

    def test_moveit_simulation(self):
        for client, name in (
            (self.list_controllers, '/controller_manager/list_controllers'),
            (self.fk, '/compute_fk'),
            (self.ik, '/compute_ik'),
            (self.state_validity, '/check_state_validity'),
        ):
            self.wait_for(lambda client=client: client.service_is_ready(), name)
        for client, name in (
            (self.move_group, '/move_action'),
            (self.execute, '/execute_trajectory'),
            (self.follower_controller, '/so101_follower_controller/follow_joint_trajectory'),
            (self.gripper_controller, '/so101_gripper_controller/follow_joint_trajectory'),
        ):
            self.wait_for(lambda client=client: client.server_is_ready(), name)
        self.wait_for(
            lambda: self.joint_state is not None and set(JOINTS) <= set(self.joint_state.name),
            '/joint_states with all SO-101 joints',
        )

        controllers = self.call(
            self.list_controllers, ListControllers.Request(), 'controller list'
        )
        states = {controller.name: controller.state for controller in controllers.controller}
        self.assertEqual(states.get('joint_state_broadcaster'), 'active')
        self.assertEqual(states.get('so101_follower_controller'), 'active')
        self.assertEqual(states.get('so101_gripper_controller'), 'active')

        zero = dict.fromkeys(JOINTS, 0.0)
        validity_request = GetStateValidity.Request()
        validity_request.group_name = 'so101'
        validity_request.robot_state = self.robot_state(zero)
        validity = self.call(self.state_validity, validity_request, 'zero state validity')
        self.assertTrue(validity.valid, [(contact.contact_body_1, contact.contact_body_2)
                                         for contact in validity.contacts])

        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = 'world'
        fk_request.fk_link_names = ['gripper_frame_link']
        fk_request.robot_state = self.robot_state(zero)
        fk_response = self.call(self.fk, fk_request, 'zero state FK')
        self.assertEqual(fk_response.error_code.val, MoveItErrorCodes.SUCCESS)
        position = fk_response.pose_stamped[0].pose.position

        for orientation in (
            Quaternion(w=1.0),
            Quaternion(x=math.sqrt(0.5), w=math.sqrt(0.5)),
            Quaternion(z=1.0, w=0.0),
        ):
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'so101_follower'
            ik_request.ik_request.ik_link_name = 'gripper_frame_link'
            ik_request.ik_request.pose_stamped.header.frame_id = 'world'
            ik_request.ik_request.pose_stamped.pose.position = position
            ik_request.ik_request.pose_stamped.pose.orientation = orientation
            ik_request.ik_request.robot_state = self.robot_state(zero)
            ik_request.ik_request.timeout = Duration(sec=1)
            ik_response = self.call(self.ik, ik_request, 'position-only IK')
            self.assertEqual(ik_response.error_code.val, MoveItErrorCodes.SUCCESS)

            solution = dict(zip(
                ik_response.solution.joint_state.name,
                ik_response.solution.joint_state.position,
            ))
            solution.setdefault('gripper', 0.0)
            solution_validity = GetStateValidity.Request()
            solution_validity.group_name = 'so101'
            solution_validity.robot_state = self.robot_state(solution)
            valid = self.call(self.state_validity, solution_validity, 'IK solution validity')
            self.assertTrue(valid.valid)

            solution_fk = GetPositionFK.Request()
            solution_fk.header.frame_id = 'world'
            solution_fk.fk_link_names = ['gripper_frame_link']
            solution_fk.robot_state = self.robot_state(solution)
            fk = self.call(self.fk, solution_fk, 'IK solution FK')
            self.assertEqual(fk.error_code.val, MoveItErrorCodes.SUCCESS)
            reached = fk.pose_stamped[0].pose.position
            error = math.sqrt(
                (reached.x - position.x) ** 2
                + (reached.y - position.y) ** 2
                + (reached.z - position.z) ** 2
            )
            self.assertLessEqual(error, 0.005)

        arm_target = {
            'shoulder_pan': 0.15,
            'shoulder_lift': -0.10,
            'elbow_flex': 0.10,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
        }
        self.execute_trajectory(self.plan('so101_follower', arm_target))
        self.wait_for_joint_target(arm_target)

        gripper_target = {'gripper': 0.10}
        self.execute_trajectory(self.plan('so101_gripper', gripper_target))
        self.wait_for_joint_target(gripper_target)

        self.execute_trajectory(self.plan('so101', zero))
        self.wait_for_joint_target(zero)
