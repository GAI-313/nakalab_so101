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
        arm.move_abs(x=0.1, z=0.05)
        arm.move_abs(x=0.1, y=0.2, z=0.05)
        arm.move_abs(x=0.1, y=-0.2, z=0.05)
        arm.move_abs(x=0.2, z=0.3)
        arm.move_abs(x=0.2, y=0.2, z=0.2)
        arm.move_abs(x=0.2, y=-0.2, z=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
