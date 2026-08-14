#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from nakalab_so101_api import ArmControl


def main():
    rclpy.init()
    node = Node('demo_move_joints')
    try:
        arm = ArmControl(node=node)
        arm.move_groupstate('zero')
        arm.joint_control(wrist_roll=1.57)
        arm.joint_control(wrist_flex=1.57)
        arm.joint_control(elbow_flex=-1.57)
        arm.joint_control(shoulder_lift=-1.57)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
