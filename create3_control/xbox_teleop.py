import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import DockServo as Dock
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection

import time
import random

class XboxTeleopDockNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_dock_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)

        self.dock_client = ActionClient(self, Dock, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')

        self.dock_client.wait_for_server()
        self.undock_client.wait_for_server()

        self.last_bumper_time = 0.0
        self.bumper_cooldown = 3.0  # seconds

        self.last_button_time = {
            'dock': 0.0,
            'undock': 0.0
        }
        self.button_cooldown = 0.5  # seconds

        self.get_logger().info("Ready: RB=enable, stick=drive, LT=reverse, A=dock, B=undock")

    def joy_callback(self, msg: Joy):
        now = time.time()
        twist = Twist()

        if msg.buttons[5]:  # RB
            reverse_mode = msg.axes[2] < 0.5
            linear_input = msg.axes[1]
            angular_input = msg.axes[0]

            twist.linear.x = -0.3 if reverse_mode else linear_input * 0.3
            twist.angular.z = angular_input * 1.5

            self.cmd_vel_pub.publish(twist)

        if msg.buttons[0] and now - self.last_button_time['dock'] > self.button_cooldown:
            self.get_logger().info("Dock button pressed")
            self.last_button_time['dock'] = now
            self.send_dock_goal()

        if msg.buttons[1] and now - self.last_button_time['undock'] > self.button_cooldown:
            self.get_logger().info("Undock button pressed")
            self.last_button_time['undock'] = now
            self.send_undock_goal()

    def hazard_callback(self, msg: HazardDetectionVector):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_bumper_time < self.bumper_cooldown:
            return

        for hazard in msg.detections:
            self.get_logger().info(f"Hazard detected: {hazard.type}")
            if hazard.type == HazardDetection.BUMP:
                self.get_logger().warn("Bumper triggered! Executing avoidance maneuver.")
                self.last_bumper_time = now

                # Step 1: Stop
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)

                # Step 2: Back up for 0.8s
                reverse_twist = Twist()
                reverse_twist.linear.x = -0.15
                end_time = (self.get_clock().now().nanoseconds / 1e9) + 0.8
                while self.get_clock().now().nanoseconds / 1e9 < end_time:
                    self.cmd_vel_pub.publish(reverse_twist)
                    time.sleep(0.1)

                # Step 3: Rotate randomly left or right
                turn_twist = Twist()
                turn_twist.angular.z = random.choice([-1.3, 1.3])  # Left or right
                end_time = (self.get_clock().now().nanoseconds / 1e9) + 0.6
                while self.get_clock().now().nanoseconds / 1e9 < end_time:
                    self.cmd_vel_pub.publish(turn_twist)
                    time.sleep(0.1)

                # Step 4: Stop again
                self.cmd_vel_pub.publish(Twist())
                break
            elif hazard.type == HazardDetection.OBJECT_PROXIMITY:
                self.get_logger().warn("Object detected nearby! Stopping robot to avoid collision.")
                self.last_bumper_time = now
                self.cmd_vel_pub.publish(Twist())  # Stop or modify behavior here
                break
    def send_dock_goal(self):
        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_dock_response)

    def handle_dock_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Dock goal was rejected.")
            return

        self.get_logger().info("Docking started...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_dock_result)

    def handle_dock_result(self, future):
        result = future.result().result
        self.get_logger().info("Docking finished.")

    def send_undock_goal(self):
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_undock_response)

    def handle_undock_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Undock goal was rejected.")
            return

        self.get_logger().info("Undocking started...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_undock_result)

    def handle_undock_result(self, future):
        result = future.result().result
        self.get_logger().info("Undocking finished.")


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopDockNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()