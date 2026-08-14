#!/usr/bin/env python3

from rclpy.node import Node
import rclpy

from nakalab_so101_api import ArmControl


def main():
    rclpy.init()
    node = Node('demo_groupstate')
    try:
        arm = ArmControl(node=node)
        arm.move_groupstate('zero')
        arm.move_groupstate('home')
        arm.move_groupstate('open', 'so101_gripper')
        arm.move_groupstate('close', 'so101_gripper')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
