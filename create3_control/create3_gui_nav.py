import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
import tkinter as tk
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from irobot_create_msgs.action import Undock, DriveDistance, RotateAngle
from irobot_create_msgs.action import DockServo as Dock
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from rclpy.action import ActionClient
import queue
from tkinter import messagebox
from PIL import Image, ImageTk
import cv2

# Constant of distances
DISTANCE_A = 2.07  # Distance to point A from Base
DISTANCE_B = 1.35 # Distance to point B from A
DISTANCE_c = 0.61 # Distance to point c from B
DISTANCE_C = 1.52 # Distance to point C from c
DISTANCE_a1 = 1.22 #Distance to point a1 from A
DISTANCE_a2 = 0.93 #Distance to point a2 from a1
DISTANCE_a3 = 1.0 #Distance to point a3 from a2
DISTANCE_a4 = 1.0 #Distance to point a4 from a3
DISTANCE_b1 = 1.22 
DISTANCE_b2 = 0.91
DISTANCE_b3 = 0.91
DISTANCE_b4 = 0.91 #Distance to point b4 from b3
DISTANCE_c1 = 0.57 #Distance to point c1 from c
DISTANCE_c2 = 0.96 #Distance to point c2 from c1
DISTANCE_c3 = 0.90 #Distance to point c3 from c2
DISTANCE_c4 = 0.90 #Distance to point c4 from c3
DISTANCE_c5 = 0.90 #Distance to point c5 from c4
dictionary_distances = {
    'A': DISTANCE_A,
    'B': DISTANCE_B,
    'c': DISTANCE_c,
    'C': DISTANCE_C,
}
dictionary_distances_backwards = {
    'c': DISTANCE_C,
    'B': DISTANCE_c,
    'A': DISTANCE_B,
    'Base': DISTANCE_A,    
}
distances_inside_backwards = {
    'A': {
        'A': DISTANCE_a1,
        'a1': DISTANCE_a2,
        'a2': DISTANCE_a3,
        'a3': DISTANCE_a4,
    },
    'B': {
        'B': DISTANCE_b1,
        'b1': DISTANCE_b2,
        'b2': DISTANCE_b3,
        'b3': DISTANCE_b4,
    },
    'C': {
        'C': DISTANCE_c1,
        'c1': DISTANCE_c2,
        'c2': DISTANCE_c3,
        'c3': DISTANCE_c4,
        'c4': DISTANCE_c5,
    }
}
distances_inside = {
    'A': {
        'A': DISTANCE_a1,
        'a1': DISTANCE_a1,
        'a2': DISTANCE_a2,
        'a3': DISTANCE_a3,
        'a4': DISTANCE_a4,
    },
    'B': {
        'B': DISTANCE_b1,
        'b1': DISTANCE_b1,
        'b2': DISTANCE_b2,
        'b3': DISTANCE_b3,
        'b4': DISTANCE_b4,
    },
    'C': {
        'C': DISTANCE_c1,
        'c1': DISTANCE_c1,
        'c2': DISTANCE_c2,
        'c3': DISTANCE_c3,
        'c4': DISTANCE_c4,
        'c5': DISTANCE_c5,
    }
}
order_inside = {
    'A': ['a1', 'a2', 'a3', 'a4'],
    'B': ['b1', 'b2', 'b3', 'b4'],
    'C': ['c1', 'c2', 'c3', 'c4', 'c5'],
}
OPTIONS = [
    'Base',
    'a1',
    'a2',
    'a3',
    'a4',
    'b1',
    'b2',
    'b3',
    'b4',
    'c1',
    'c2',
    'c3',
    'c4',
    'c5',
    'A',
    'B',
    'C',
]
ORDER = [
    'Base',
    'A',
    'B',
    'c',
    'C',
]
# order for inside points
dev = True  # Set to True for development mode, False for production
class Create3ManualGUI(Node):
    def __init__(self):
        super().__init__('create3_manual_gui')
        self.current_location = 'Docked'  # Start at Base
        self.path = []
        self.rotation = 0
        self.gui_queue = queue.Queue()
        self.action_timeout = 30.0  # Timeout for actions (seconds)
        self.next_angle = 0
        self.is_backward = False 
        self.is_rotated = False
        self.main_point = ''
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(HazardDetectionVector, '/hazard_detection', self.hazard_callback, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.undock_client = ActionClient(self, Undock, '/undock')
        self.dock_client = ActionClient(self, Dock, '/dock')
        self.drive_client = ActionClient(self, DriveDistance, '/drive_distance')
        self.rotate_angle = ActionClient(self, RotateAngle, '/rotate_angle')

        self.last_bumper_time = 0.0
        self.bumper_cooldown = 3.0  # seconds

        # GUI setup
        self.window = tk.Tk()
        self.window.title("Create 3 Manual Controller")
        self.battery_label = tk.Label(self.window, text="Battery: --%")
        self.battery_label.pack(pady=5)
        self.button_frame = tk.Frame(self.window)
        self.button_frame.pack()
        
        self.dockButton = tk.Button(self.button_frame, text="Dock", bg="green", command=self.dock)
        self.dockButton.pack(pady=5)
        if not dev:
            self.dockButton['state'] = 'disabled'  # Disable dock button initially
        
        self.undockButton = tk.Button(self.button_frame, text="Undock", bg="orange", command=self.undock)
        self.undockButton.pack(pady=5)

        self.value_inside = tk.StringVar(self.window)
        self.value_inside.set("Base")
        question_label = tk.Label(self.window, text="A dónde quieres ir?")
        question_label.pack(pady=5)
        question_menu = tk.OptionMenu(self.window, self.value_inside, *OPTIONS)
        question_menu.pack(pady=5)
        
        # Add buttons for navigation
        # disable if current_location is docked
        self.b1 = tk.Button(self.button_frame, text="Ir", bg="lightblue", command=self.go_to_point)
        self.b2 = tk.Button(self.button_frame, text="Set location", bg="lightblue", command=self.set_current_location)
        self.b3 = tk.Button(self.button_frame, text="Stop", bg="red", command=self.stop_robot)
        self.b4 = tk.Button(self.button_frame, text="Rotate", bg="lightblue", command=self.rotate_robot)
        self.b1.pack(pady=5)
        if (dev):
            self.b2.pack(pady=5)
            self.b4.pack(pady=5)

        if not dev:
            self.b1['state'] = 'disabled'
        # Add a label and number input for distance
        self.distance_label = tk.Label(self.window, text="Distance (m):")
        self.distance_label.pack(pady=5)
        self.distance_entry = tk.Entry(self.window)
        self.distance_entry.pack(pady=5)
        tk.Button(self.button_frame, text="Drive Distance", bg="lightgreen", command=self.drive_distance).pack(pady=5)
        # Add 

        self.location_label = tk.Label(self.window, text=f"Sitio actual: {self.current_location}")
        self.location_label.pack(pady=5)

        imageCv2 = cv2.imread('/home/juanjg/Pictures/mapa.jpeg')
        if imageCv2 is None:
            self.get_logger().error("Error loading image")
            return
        else:
            self.get_logger().info("Image loaded successfully")
        # Put points on the image
        imageCv2 = cv2.resize(imageCv2, (400, 500), interpolation=cv2.INTER_AREA)
        imageCv2 = cv2.circle(imageCv2, (244, 70), 5, (255, 255, 0), -1)  # Point Docked
        imageCv2 = cv2.circle(imageCv2, (244, 147), 5, (255, 0, 0), -1)  # Point Base
        if dev:
            imageCv2 = cv2.circle(imageCv2, (244, 270), 5, (0, 0, 255), -1)  # Point A
            imageCv2 = cv2.circle(imageCv2, (244, 332), 5, (0, 255, 0), -1)  # Point B
            imageCv2 = cv2.circle(imageCv2, (280, 405), 5, (255, 0, 0), -1)  # Point C
        imageCv2 = cv2.circle(imageCv2, (200, 270), 5, (255, 0, 255), -1)  # Point a1
        imageCv2 = cv2.circle(imageCv2, (170, 270), 5, (0, 255, 255), -1)  # Point a2
        imageCv2 = cv2.circle(imageCv2, (140, 270), 5, (128, 0, 128), -1)  # Point a3
        imageCv2 = cv2.circle(imageCv2, (110, 400), 5, (128, 128, 0), -1)  # Point a4

        imageCv2 = cv2.circle(imageCv2, (244, 332), 5, (0, 128, 128), -1)  # Point B
        imageCv2 = cv2.circle(imageCv2, (205, 332), 5, (128, 128, 128), -1)  # Point b1
        imageCv2 = cv2.circle(imageCv2, (175, 332), 5, (128, 128, 128), -1)  # Point b2
        imageCv2 = cv2.circle(imageCv2, (145, 332), 5, (255, 128, 0), -1)  # Point b3
        imageCv2 = cv2.circle(imageCv2, (99, 332), 5, (0, 255, 128), -1)  # Point b4

        imageCv2 = cv2.circle(imageCv2, (264, 405), 5, (128, 0, 255), -1)  # Point c1
        imageCv2 = cv2.circle(imageCv2, (233, 405), 5, (255, 0, 128), -1)  # Point c2
        imageCv2 = cv2.circle(imageCv2, (204, 405), 5, (0, 128, 255), -1)  # Point c3
        imageCv2 = cv2.circle(imageCv2, (175, 405), 5, (128, 255, 0), -1)  # Point c4
        imageCv2 = cv2.circle(imageCv2, (146, 405), 5, (255, 128, 128), -1)  # Point c5
        #imageCv2 = cv2.resize(imageCv2, (500, 800), interpolation=cv2.INTER_AREA)
        #imageCv2 = cv2.rotate(imageCv2, -90)  # Rotate the image to match the layout
        # Show text on the image
        cv2.putText(imageCv2, 'Docked', (220, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(imageCv2, 'Base', (220, 147), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        if dev:
            cv2.putText(imageCv2, 'A', (220, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(imageCv2, 'B', (220, 332), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(imageCv2, 'C', (260, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv2.putText(imageCv2, 'a1', (180, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        cv2.putText(imageCv2, 'a2', (150, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(imageCv2, 'a3', (120, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 128), 1)
        cv2.putText(imageCv2, 'a4', (90, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 0), 1)
        cv2.putText(imageCv2, 'b1', (180, 332), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 128), 1)
        cv2.putText(imageCv2, 'b2', (150, 332), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        cv2.putText(imageCv2, 'b3', (120, 332), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
        cv2.putText(imageCv2, 'b4', (70, 332), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 1)
        cv2.putText(imageCv2, 'c1', (240, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 0, 255), 1)
        cv2.putText(imageCv2, 'c2', (210, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 128), 1)
        cv2.putText(imageCv2, 'c3', (180, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 1)
        cv2.putText(imageCv2, 'c4', (150, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 255, 0), 1)
        cv2.putText(imageCv2, 'c5', (120, 405), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 128), 1)

        imageCv2 = cv2.cvtColor(imageCv2, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(imageCv2)
        imgTk = ImageTk.PhotoImage(image=im)
        self.image_label = tk.Label(self.window, image=imgTk)
        self.image_label.image = imgTk  # Keep a reference to avoid garbage collection
        self.image_label.pack()

        # Queue for thread-safe GUI updates
        self.gui_queue = queue.Queue()
        self.window.after(100, self.process_gui_queue)

    def battery_callback(self, msg):
        percent = int(msg.percentage * 100)
        self.gui_queue.put(lambda: self.battery_label.config(text=f"Battery: {percent}%"))

    def process_gui_queue(self):
        try:
            while True:
                callback = self.gui_queue.get_nowait()
                callback()
        except queue.Empty:
            pass
        self.window.after(100, self.process_gui_queue)

    def dock(self):
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Dock server not available")
            return

        self.get_logger().info("Sending Dock action...")
        goal_msg = Dock.Goal()
        future = self.dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.dock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Dock goal rejected 😞")
                return
            self.get_logger().info("Dock goal accepted 🎯")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.dock_result_callback)
        except Exception as e:
            self.get_logger().error(f"Dock goal response error: {e}")

    def dock_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Dock result received: {result}")
            self.current_location = 'Docked'
            # deactivate the dock button and activate the undock button
            if not dev:
                self.gui_queue.put(lambda: self.dockButton.config(state='disabled'))
                self.gui_queue.put(lambda: self.undockButton.config(state='normal'))
                self.gui_queue.put(lambda: self.b1.config(state='normal'))
                self.gui_queue.put(lambda: self.location_label.config(text=f"Sitio actual: {self.current_location}"))
        except Exception as e:
            self.get_logger().error(f"Dock result error: {e}")

    def undock(self):
        if not self.undock_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Undock server not available")
            return

        self.get_logger().info("Sending Undock action...")
        goal_msg = Undock.Goal()
        future = self.undock_client.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Undock goal rejected ❌")
                return
            self.get_logger().info("Undock goal accepted 🎯")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.undock_result_callback)
        except Exception as e:
            self.get_logger().error(f"Undock goal response error: {e}")

    def undock_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Undock result received: {result}")
            if not self.drive_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("DriveDistance server not available")
                return
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 1.6  # meters
            drive_goal.max_translation_speed = 0.2  # m/s
            drive_future = self.drive_client.send_goal_async(drive_goal)
            drive_future.add_done_callback(self.drive_goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Undock result error: {e}")

    def drive_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("DriveDistance goal rejected ❌")
                return
            self.get_logger().info("DriveDistance goal accepted 🚗")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.drive_result_callback)
        except Exception as e:
            self.get_logger().error(f"Drive goal response error: {e}")

    def drive_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"DriveDistance result received: {result}")
            self.current_location = 'Base'
            self.gui_queue.put(lambda: self.location_label.config(text=f"Sitio actual: {self.current_location}"))
            # deactivate the undock button and activate the dock button
            if not dev:
                self.gui_queue.put(lambda: self.undockButton.config(state='disabled'))
                self.gui_queue.put(lambda: self.dockButton.config(state='normal'))
                self.gui_queue.put(lambda: self.b1.config(state='normal'))

        except Exception as e:
            self.get_logger().error(f"Drive result error: {e}")

    def drive_distance(self):
        try:
            distance = float(self.distance_entry.get())
        except ValueError:
            self.get_logger().error("Invalid distance input")
            return
        if not self.drive_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("DriveDistance server not available")
            return

        self.get_logger().info(f"Driving {distance:.2f} m")

        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.25
        future = self.drive_client.send_goal_async(goal)
        future.add_done_callback(self.drive_to_point_response_callback)

    def drive_to_point_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("DriveDistance goal rejected")
                return

            self.get_logger().info("DriveDistance goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.drive_to_point_result_callback)

        except Exception as e:
            self.get_logger().error(f"Drive goal error: {e}")

    def drive_to_point_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Arrived at point: {result}")
        except Exception as e:
            self.get_logger().error(f"Drive result error: {e}")

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def run(self):
        def ros_spin():
            rclpy.spin(self)

        threading.Thread(target=ros_spin, daemon=True).start()
        self.window.mainloop()

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()
    
    def rotate_robot(self, angle=1.57):
        if not self.rotate_angle.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("RotateAngle server not available")
            return

        self.get_logger().info(f"Rotating robot {angle} radians")
        goal = RotateAngle.Goal()
        goal.angle = angle
        future = self.rotate_angle.send_goal_async(goal)
        future.add_done_callback(self.rotate_callback_custom)

    def rotate_callback_custom(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected")
                return

            self.get_logger().info("Rotate angle goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.rotate_result_callback_custom)
        except Exception as e:
            self.get_logger().error(f"Drive goal error: {e}")
    
    def rotate_result_callback_custom(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Rotated {result}")
        except Exception as e:
            self.get_logger().error(f"Drive result error: {e}")

    def send_drive_goal(self, distance, target_label):
        if not self.drive_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("DriveDistance server not available")
            return
        #show angle and 
        print(self.rotation)
        # if next point is c, rotate 90 degrees the robot
        # positive is clockwise
        if self.rotation == 180:
            self.is_rotated = True
        if self.is_backward and not self.is_rotated:
            self.get_logger().info("Rotating 180 degrees clockwise")
            goal = RotateAngle.Goal()
            goal.angle = -3.14
            self.next_angle = 180
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        # entrando al pasillo de A
        elif target_label == 'a1' and self.rotation != 90 and self.current_location == 'A':
            self.get_logger().info("Rotating from A")
            goal = RotateAngle.Goal()
            goal.angle = -1.57
            self.next_angle = 90
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        # Entrando al pasillo de B
        elif target_label == 'b1' and self.rotation != 90 and self.current_location == 'B':
            self.get_logger().info('Rotating from B')
            goal = RotateAngle.Goal()
            goal.angle = -1.57
            self.next_angle = 90
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        # Entrando al pasillo de C
        elif target_label == 'c1' and self.rotation != 90 and not self.is_backward and self.current_location == 'C':
            self.get_logger().info("Rotating from C")
            goal = RotateAngle.Goal()
            goal.angle = -1.57
            self.next_angle = 90
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)

        # Rotación en B para ir a c
        elif target_label == 'c' and self.rotation != -90 and not self.is_backward:
            self.get_logger().info("Rotating from B")
            goal = RotateAngle.Goal()
            goal.angle = 1.57
            self.next_angle = -90
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        elif target_label == 'c' and self.is_backward and self.current_location == 'C' and self.rotation != 0:
            self.get_logger().info("Rotating from C")
            goal = RotateAngle.Goal()
            goal.angle = 1.57
            self.next_angle = 0
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        # Rotación en c para ir a C
        elif target_label == 'C' and self.rotation == -90 and not self.is_backward:
            self.get_logger().info("Rotating from c")
            goal = RotateAngle.Goal()
            goal.angle = -1.57
            self.next_angle = 0
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        
        elif target_label == 'B' and self.is_backward and self.rotation != -90 and self.current_location == 'c':
            self.get_logger().info("Rotating from c")
            goal = RotateAngle.Goal()
            goal.angle = 1.57
            self.next_angle = -90
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        elif target_label == 'A' and self.is_backward and self.rotation == -90 and self.current_location == 'B':
            self.get_logger().info("Rotating from B")
            goal = RotateAngle.Goal()
            goal.angle = -1.57
            self.next_angle = 0
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        elif target_label == 'A' and self.is_backward and self.rotation == 180 and self.current_location == 'B':
            self.get_logger().info('Rotating from B')
            goal = RotateAngle.Goal()
            goal.angle = 1.57
            self.next_angle = 0
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        elif target_label == 'Base' and self.is_backward and self.rotation != 0 and self.current_location == 'A':
            self.get_logger().info("Rotating from A")
            goal = RotateAngle.Goal()
            goal.angle = 1.57
            self.next_angle = 0
            future = self.rotate_angle.send_goal_async(goal)
            future.add_done_callback(self.rotate_callback)
        else: 
            self.get_logger().info(f"Driving {distance:.2f} m to {target_label}")
            goal = DriveDistance.Goal()
            goal.distance = distance
            goal.max_translation_speed = 0.25
            future = self.drive_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.drive_to_point_response_callback(f, target_label))
    
    def drive_to_point_response_callback(self, future, target_label):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("DriveDistance goal rejected")
                return

            self.get_logger().info("DriveDistance goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda f: self.drive_to_point_result_callback(f, target_label))

        except Exception as e:
            self.get_logger().error(f"Drive goal error: {e}")
    
    def drive_to_point_result_callback(self, future, target_label):
        try:
            result = future.result().result
            self.get_logger().info(f"Arrived at {target_label}")
            self.current_location = target_label
            self.gui_queue.put(lambda: self.location_label.config(text=f"Current: {target_label}"))
            # check the path, remove the first element and send the next one
            if self.path:
                # remove the first element from the path
                self.path.pop(0)
                if self.path.__len__() > 0:
                    if target_label in dictionary_distances or target_label in dictionary_distances_backwards:
                        distance = dictionary_distances.get(self.path[0], 0) if not self.is_backward else dictionary_distances_backwards.get(self.path[0], 0)
                    else:
                        point = 'A'
                        if self.path[0] in order_inside['B'] or self.path[0] == 'B':
                            point = 'B'
                        elif self.path[0] in order_inside['C'] or self.path[0] == 'C':
                            point = 'C'
                        distances = distances_inside[point] if not self.is_backward else distances_inside_backwards[point]
                        distance = distances.get(self.path[0], 0)
                    self.send_drive_goal(distance, self.path[0])
                elif self.is_rotated:
                    #rotate
                    if self.current_location == 'A' or self.current_location == 'B' or self.current_location == 'C':
                        self.rotate_robot(1.57)
                    else:
                        self.rotate_robot(3.14)
                    self.is_rotated = False

        except Exception as e:
            self.get_logger().error(f"Drive result error: {e}")

    def rotate_callback(self, future):
        print('rotate callback')
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected")
                return

            self.get_logger().info("Rotate angle goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.rotate_result_callback)
        except Exception as e:
            self.get_logger().error(f"Drive goal error: {e}")
    
    def rotate_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Rotated")
            self.rotation = self.next_angle
            next_point = self.path[0]
            distance = dictionary_distances.get(self.path[0], 0) if not self.is_backward else dictionary_distances_backwards.get(self.path[0], 0)
            if ((next_point in dictionary_distances) and
                (self.current_location != 'a1' and self.current_location != 'b1' and self.current_location != 'c1')):
                distance = dictionary_distances.get(self.path[0], 0) if not self.is_backward else dictionary_distances_backwards.get(self.path[0], 0)
            elif next_point == 'Base':
                distance = DISTANCE_A
            else:
                point = 'A'
                if next_point in order_inside['B'] or next_point == 'B':
                    point = 'B'
                elif next_point in order_inside['C'] or next_point == 'C':
                    point = 'C'
                distances = distances_inside[point] if not self.is_backward else distances_inside_backwards[point]
                distance = distances.get(next_point, 0)
            self.send_drive_goal(distance, self.path[0])
        except Exception as e:
            self.get_logger().error(f"Drive result error: {e}")


    def go_to_point(self):
        value_inside = self.value_inside.get()
        if value_inside == self.current_location:
            # popup
            self.get_logger().info(f"Already at {value_inside}")
            self.gui_queue.put(lambda: messagebox.showinfo("Info", f"Already at {value_inside}"))
            return
        if value_inside == 'Docked':
            self.get_logger().info("Docked, cannot go to another point")
            self.gui_queue.put(lambda: messagebox.showerror("Error", "No se puede ir a Docked"))
            return
        points = self.calculate_path(value_inside)
    
    def calculate_path(self, target):
        if target not in dictionary_distances and target not in dictionary_distances_backwards and target not in order_inside['A'] and target not in order_inside['B'] and target not in order_inside['C']:
            self.get_logger().error(f"Invalid target point: {target}")
            return
    
        self.path = []
        current = self.current_location 
        if (current in order_inside['A'] or current in order_inside['B'] or current in order_inside['C']):
            if not ((current in order_inside['A'] and target in order_inside['A']) or
                (current in order_inside['B'] and target in order_inside['B']) or
                (current in order_inside['C'] and target in order_inside['C'])):
                self.calculate_path_to_ABC(target)
        
        starting_point = self.current_location
        auxTarget = target
        if not self.main_point:
            self.calculate_main_point(target)
        if not (self.current_location in ORDER):
            starting_point = self.main_point
        if target not in ORDER:
            if target in order_inside['A']:
                auxTarget = 'A'
            elif target in order_inside['B']:
                auxTarget = 'B'
            elif target in order_inside['C']:
                auxTarget = 'C'

        idxOfStartingPoint = ORDER.index(starting_point)
        idxOfTarget = ORDER.index(auxTarget)
        if idxOfTarget > idxOfStartingPoint:
            # Moving forward in the order, without the starting point
            self.path += ORDER[idxOfStartingPoint + 1:idxOfTarget + 1]
            self.is_backward = False
        # inverse path
        elif idxOfTarget < idxOfStartingPoint:
            # Moving backward in the order
            self.path += ORDER[idxOfTarget:idxOfStartingPoint][::-1]
            self.is_backward = True
            self.is_rotated = False
        # if the target is inside a point, add the inside points
        for key in ['A','B', 'C']:
            group = order_inside[key]
            if target in group:
                self.get_logger().info(f"Target {target} is inside {key}")
                idxOfTarget = group.index(target)
                idxOfStartingPoint = 0
                if self.current_location in group:
                    idxOfStartingPoint = group.index(self.current_location) + 1
                else:
                    idxOfStartingPoint = 0
                inside_points = group[idxOfStartingPoint:idxOfTarget + 1]
                self.path += inside_points
        self.get_logger().info(f"Calculated path: {self.path}")
        distance = 0
        next_point = self.path[0]
        if ((next_point in dictionary_distances) and
            (self.current_location != 'a1' and self.current_location != 'b1' and self.current_location != 'c1')):
            print('Main branch')
            distance = dictionary_distances.get(self.path[0], 0) if not self.is_backward else dictionary_distances_backwards.get(self.path[0], 0)
        else:
            point = 'A'
            if next_point in order_inside['B']:
                point = 'B'
            elif next_point in order_inside['C']:
                point = 'C'
            distances = distances_inside[point] if not self.is_backward else distances_inside_backwards[point]
            print('Inside branch')
            print(f"Distances: {distances}")
            distance = distances.get(next_point, 0)
        self.send_drive_goal(distance, self.path[0])

    def calculate_path_to_ABC(self, target):
        self.main_point = None
        location = self.current_location
        if location in dictionary_distances:
            self.get_logger().info(f"Current {location} is a main point")
            return
        elif location in order_inside['A']:
            self.get_logger().info(f"Current {location} is inside A")
            self.main_point = 'A'
        elif location in order_inside['B']:
            self.get_logger().info(f"Current {location} is inside B")
            self.main_point = 'B'
        elif location in order_inside['C']:
            self.get_logger().info(f"Current {location} is inside C")
            self.main_point = 'C'
        else:
            self.get_logger().error(f"Invalid location point: {location}")

        # Calculate the path to the main point
        # if starting point is c4 path = [c3, c2, c1, C]
        if self.main_point is not None:
            print(f"Main point is {self.main_point}")
            idxOfStartingPoint = 0
            try:
                idxOfStartingPoint = order_inside[self.main_point].index(self.current_location)
            except:
                return
            # take from 0 to startingPoint and reverse
            self.path = order_inside[self.main_point][:idxOfStartingPoint][::-1]
            self.is_backward = True
            self.is_rotated = False
            self.path.append(self.main_point)

    def set_current_location(self):
        value_inside = self.value_inside.get()
        self.current_location = value_inside
        self.gui_queue.put(lambda: self.location_label.config(text=f"Sitio actual: {self.current_location}"))

    def calculate_main_point(self, target):
        if target in dictionary_distances or target in dictionary_distances_backwards:
            self.main_point = target
            return
        if target in order_inside['A']:
            self.main_point = 'A'
        elif target in order_inside['B']:
            self.main_point = 'B'
        elif target in order_inside['C']:
            self.main_point = 'C'
        else:
            self.get_logger().error(f"Invalid target point: {target}")
    
    def hazard_callback(self, msg: HazardDetectionVector):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_bumper_time < self.bumper_cooldown:
            return

        for hazard in msg.detections:
            if hazard.type == HazardDetection.BUMP:
                self.get_logger().warn("Bumper triggered! Executing avoidance maneuver.")
                self.last_bumper_time = now

                # Step 1: Stop
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                # stop path and everything
                self.path = []
                self.is_rotated = False
                #self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    try:
        gui = Create3ManualGUI()
        gui.run()
    except KeyboardInterrupt:
        gui.get_logger().info("Shutting down Create 3 Manual GUI")
        pass
    except Exception as e:
        gui.get_logger().error(f"An error occurred: {e}")
        messagebox.showerror("Error", f"An error occurred: {e}")
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()