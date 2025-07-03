import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from irobot_create_msgs.action import DockServo, Undock
from builtin_interfaces.msg import Duration
import threading
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState

class DockingClient(Node):
    def __init__(self):
        super().__init__('docking_client')
        self.dock_client = ActionClient(self, DockServo, '/dock')
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.executor_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.executor_thread.start()
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile_sensor_data
        )
        self.battery = 0
    def dock_robot(self):
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Dock server not available")
            return

        self.get_logger().info("Sending Dock action...")
        goal_msg = DockServo.Goal()
        future = self.dock_client.send_goal_async(goal_msg)

    def undock_robot(self):
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Undock server not available")
            return

        self.get_logger().info("Sending Undock action...")
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)

    def battery_callback(self, msg):
        self.battery = msg.percentage * 100 if msg.percentage is not None else None

    def get_battery(self):
        return self.battery if self.battery is not None else "Battery data not available"

rclpy.init()
dock_node = DockingClient()

def dock_robot():
    dock_node.dock_robot()
    return "Docking initiated"

def undock_robot():
    dock_node.undock_robot()
    return "Undocking initiated"

def get_battery():
    return dock_node.get_battery()