"""Safe import and constructor smoke check for ArmControl."""

import rclpy

from .nakalab_so101_api import ArmControl


def main() -> None:
    """Construct ArmControl without sending a goal or accessing hardware."""
    rclpy.init()
    node = rclpy.create_node('nakalab_so101_api_smoke')
    try:
        ArmControl(node, wait_time=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
