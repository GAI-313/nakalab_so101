#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool
import math
from nakalab_so101_py.feetech_bus import FeetechBus


class LeaderArmDriverNode(Node):
    def __init__(self):
        super().__init__('leader_arm_driver_node')
        self.declare_parameter('device', '')
        self.declare_parameter('enable_teleop', False)
        device_port = self.get_parameter('device').value
        self.teleop_enabled = self.get_parameter('enable_teleop').value

        if not device_port:
            self.get_logger().error("Parameter 'device' must be set!")
            return

        # Assumption based on lerobot implementation naming:
        self.motor_ids = {
            'shoulder_pan': 1,
            'shoulder_lift': 2,
            'elbow_flex': 3,
            'wrist_flex': 4,
            'wrist_roll': 5,
            'gripper': 6
        }
        self.id_to_name = {v: k for k, v in self.motor_ids.items()}

        self.bus = FeetechBus(port=device_port)
        self.bus.connect()
        # Ensure torque is disabled for leader (teleop mode)
        self.bus.enable_torque(list(self.motor_ids.values()), False)

        self.joint_state_pub = self.create_publisher(JointState, 'leader/joint_states', 10)

        self.teleop_service = self.create_service(SetBool, 'teleop', self.teleop_service_callback)

        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz read
        self.get_logger().info(f"Leader Arm Driver started on {device_port}")


    def teleop_service_callback(self, request, response):
        self.teleop_enabled = request.data
        response.success = True
        response.message = f"Teleop enabled: {self.teleop_enabled}"
        self.get_logger().info(response.message)
        return response

    def pos_to_rad(self, pos: int) -> float:
        return (pos - 2048.0) * (2.0 * math.pi) / 4096.0

    def timer_callback(self):
        if not self.teleop_enabled or not self.bus.is_connected():
            return

        positions = self.bus.sync_read_present_positions(list(self.motor_ids.values()))
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []

        for mid, pos in positions.items():
            if mid in self.id_to_name:
                msg.name.append(self.id_to_name[mid])
                msg.position.append(self.pos_to_rad(pos))

        self.joint_state_pub.publish(msg)


def leader(args=None):
    rclpy.init(args=args)
    node = LeaderArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'bus') and node.bus.is_connected():
            # Disable torque as extra precaution
            node.bus.enable_torque(list(node.motor_ids.values()), False)
            node.bus.disconnect()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
