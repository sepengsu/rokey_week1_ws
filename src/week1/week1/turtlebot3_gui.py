import sys
import threading
import tkinter as tk
from tkinter import Text

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action.client import GoalStatus

class NavigationClient(Node):
    def __init__(self):
        super().__init__('node')
        # setting
        self.len = 3
        self.init_pose = [-2.0, -0.5, 0.0, 1.0]  # pose: x, y orient: z, w
        
        # init
        self.goal_poses = [[0, 0] for i in range(self.len)]
        self.setting_poses = [False for i in range(self.len)]
        
        # GUI
        self.app = tk.Tk()
        self.app.title("Navigation Client")
        self.create_gui()
        
        self.publisher = self.create_publisher(String, 'example_topic', 10)
        
        # create Topic Subscriber
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            "clicked_point",
            self.clicked_point_callback,
            10
        )
        
        # create Service Client
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')
        
        # create Action Client
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # Init function
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        self.set_initial_pose(*self.init_pose)
    
    def create_gui(self):
        self.get_buttons = []
        self.labels = []
        self.go_buttons = []
        
        for i in range(self.len):
            get_button = tk.Button(self.app, text=f"get{i}", command=lambda i=i: self.get_button_clicked(i))
            get_button.grid(row=i, column=0, padx=10, pady=5)
            self.get_buttons.append(get_button)
            
            label = tk.Label(self.app, text="x= 0.0, y= 0.0")
            label.grid(row=i, column=1, padx=10, pady=5)
            self.labels.append(label)
            
            go_button = tk.Button(self.app, text=f"go{i}", command=lambda i=i: self.go_button_clicked(i))
            go_button.grid(row=i, column=2, padx=10, pady=5)
            self.go_buttons.append(go_button)
        
        self.text_browser = Text(self.app, height=10, width=40)
        self.text_browser.grid(row=self.len, column=0, columnspan=3, padx=10, pady=10)
    
    def get_button_clicked(self, i):
        self.setting_poses[i] = True
    
    def go_button_clicked(self, i):
        self.navigate_to_pose_send_goal(i)
    
    # Topic subscriber GET POSE
    def clicked_point_callback(self, msg):
        for i in range(self.len):
            if self.setting_poses[i]:
                x = round(float(msg.point.x), 1)
                y = round(float(msg.point.y), 1)
                self.goal_poses[i][0] = x
                self.goal_poses[i][1] = y
                
                self.labels[i].config(text=f"x= {x:.1f}, y= {y:.1f}")
                message = f"[GET] table_{i} is x= {x:.1f}, y= {y:.1f}"
                self.text_browser.insert(tk.END, message + "\n")
                
                self.setting_poses[i] = False
    
    # Service client SET INIT POSE ESTIMATE
    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        
        future = self.set_initial_pose_service_client.call_async(req)
        
        if future.result() is not None:
            message = "[INFO] Initial pose set successfully"
        else:
            message = "[WARN] Failed to set initial pose"
        
        self.text_browser.insert(tk.END, message + "\n")
    
    # Action client NAVIGATE
    def navigate_to_pose_send_goal(self, i):
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                self.text_browser.insert(tk.END, message + "\n")
                return False
            wait_count += 1
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_poses[i][0]
        goal_msg.pose.pose.position.y = self.goal_poses[i][1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        
        return True

    def navigate_to_pose_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = "[WARN] Action goal rejected."
            self.text_browser.insert(tk.END, message + "\n")
            return

        message = "[INFO] Action goal accepted."
        self.text_browser.insert(tk.END, message + "\n")
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)

    def navigate_to_pose_action_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback

    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            message = "[INFO] Action succeeded!"
        else:
            message = f"[WARN] Action failed with status: {action_status}"
        self.text_browser.insert(tk.END, message + "\n")
    
    def run(self):
        self.ros_thread = threading.Thread(target=self.run_ros, daemon=True)
        self.ros_thread.start()
        
        self.app.mainloop()
    
    def run_ros(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = NavigationClient()
    node.run()
    
    try:
        rclpy.shutdown()
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()