#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from nakalab_so101_api import ArmControl


def main():
    rclpy.init()
    node = Node('demo_get_states')
    try:
        arm = ArmControl(node=node)
        arm.move_groupstate('zero')
        joint_poses = arm.get_current_joints_pose()
        endeffector_pose = arm.get_current_pose()
        for key, value in joint_poses.items():
            print(key, ':', value)
        print('End effector pose :', endeffector_pose)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
