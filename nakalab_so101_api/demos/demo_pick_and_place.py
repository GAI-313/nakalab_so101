#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from nakalab_so101_api import ArmControl


def main():
    rclpy.init()
    node = Node('demo_pick_and_place')
    try:
        arm = ArmControl(node=node)
        arm.move_groupstate('home')
        arm.open_gripper()
        arm.move_abs(x=0.3, z=0.05)
        arm.move_abs(x=0.3, z=0.0)
        arm.close_gripper()
        arm.move_groupstate('home')
        arm.move_abs(x=0.1, y=0.2, z=0.05)
        arm.open_gripper()
        arm.move_rel(z=0.05)
        arm.close_gripper()
        arm.move_groupstate('home')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
