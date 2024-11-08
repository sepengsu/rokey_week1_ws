import sys
import threading
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action.client import GoalStatus
class NavigationClient(Node):
    def __init__(self):
        super().__init__('node')
        # Set the number of goal locations and initialize coordinates
        self.len = 9
        self.init_pose = [0.0, 0.0, 0.0, 1.0]  # Initial pose: x, y, orient:z, w
        self.goal_poses = [
            #1테이블      #2번테이블     #3번테이블        #4번테이블       #5번테이블
            [0.8, 0.6], [0.7, -0.3], [0.9, -0.7], [0.068, 1.4], [0.0417, -0.41], 
            [-0.02, -1.47], [-1.23, 1.396], [-1.5, -0.088], [-1.16, -1.42]
             #6번테이블         #7번테이블         #8번테이블        #9번테이블
        ]  # Replace with desired x, y coordinates for each goal position
        
        # Initialize the GUI
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.create_gui()
        self.publisher = self.create_publisher(String, 'example_topic', 10)
        # Create Service Client for setting initial pose
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')
        
        # Create Action Client for navigating to a pose
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # Wait for the initial pose service to be available
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        
        # Set the initial pose
        self.set_initial_pose(*self.init_pose)
        self.starting_position = self.init_pose[:2]  # Only storing x and y coordinates
    def create_gui(self):
        # Set up the window layout
        self.window.setWindowTitle("Navigation Client")
        self.window.resize(600, 700)
        central_widget = QWidget(self.window)
        self.window.setCentralWidget(central_widget)
        # Set layout for central widget
        main_layout = QVBoxLayout(central_widget)
        
        # Create a grid layout for checkboxes and labels
        grid_layout = QGridLayout()
        grid_layout.setContentsMargins(10, 10, 10, 10)  # Set margins for grid
        grid_layout.setSpacing(10)  # Set spacing between elements
        main_layout.addLayout(grid_layout)
        self.checkboxes = []
        self.labels = []
        for i in range(self.len):
            checkbox = QCheckBox(f"Table {i+1}")  # Change from "Go to {i+1}" to "Table {i+1}"
            label = QLabel(f"x={self.goal_poses[i][0]}, y={self.goal_poses[i][1]}")  # Adjust coordinate display
            # Styling for checkboxes and labels
            checkbox.setStyleSheet("QCheckBox { font-size: 14px; color: #1E90FF; padding: 5px; }")
            label.setStyleSheet("QLabel { font-size: 14px; color: #32CD32; padding: 5px; }")
            self.checkboxes.append(checkbox)
            self.labels.append(label)
            row = i // 3  # Create a 3-column grid
            col = i % 3
            grid_layout.addWidget(checkbox, row, col * 2)  # Column for checkbox
            grid_layout.addWidget(label, row, col * 2 + 1)  # Column for label
        # Start button with color styling
        self.startButton = QPushButton("Start Navigation")
        self.startButton.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.startButton.clicked.connect(self.start_navigation)
        main_layout.addWidget(self.startButton)
        # Text Browser for status messages with a background color
        self.textBrowser = QTextBrowser()
        self.textBrowser.setStyleSheet("""
            QTextBrowser {
                background-color: #f4f4f4;
                color: #333333;
                font-size: 12px;
                border: 1px solid #ccc;
                border-radius: 5px;
                padding: 10px;
            }
        """)
        main_layout.addWidget(self.textBrowser)
    def start_navigation(self):
        # Get selected goal positions
        self.selected_goals = [i for i in range(self.len) if self.checkboxes[i].isChecked()]
        
        if not self.selected_goals:
            self.textBrowser.append("[INFO] No tables selected.")
            return
        self.current_goal_index = 0
        self.navigate_to_selected_goal()
    def navigate_to_selected_goal(self):
        if self.current_goal_index < len(self.selected_goals):
            goal_index = self.selected_goals[self.current_goal_index]
            self.navigate_to_pose_send_goal(goal_index)
        else:
            # After all goals, return to start
            self.return_to_start()
    def navigate_to_pose_send_goal(self, goal_index):
        # Set up navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.goal_poses[goal_index][0]
        goal_msg.pose.pose.position.y = self.goal_poses[goal_index][1]
        goal_msg.pose.pose.orientation.w = 1.0
        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback
        )
        self.send_goal_future.add_done_callback(self.on_goal_reached)
    def on_goal_reached(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.textBrowser.append("[WARN] Action goal rejected.")
            return
        self.textBrowser.append("[INFO] Action goal accepted.")
        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.on_goal_finished)
    def on_goal_finished(self, future):
        action_status = future.result().status
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.textBrowser.append("[INFO] Reached goal.")
            self.current_goal_index += 1
            self.navigate_to_selected_goal()
        else:
            self.textBrowser.append(f"[WARN] Navigation failed with status: {action_status}")
            self.return_to_start()
    def return_to_start(self):
        self.textBrowser.append("[INFO] Returning to start...")
        return_goal_msg = NavigateToPose.Goal()
        return_goal_msg.pose.header.frame_id = "map"
        return_goal_msg.pose.pose.position.x = self.starting_position[0]
        return_goal_msg.pose.pose.position.y = self.starting_position[1]
        return_goal_msg.pose.pose.orientation.w = 1.0
        self.navigate_to_pose_action_client.send_goal_async(
            return_goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback
        )
    def navigate_to_pose_action_feedback(self, feedback_msg):
        pass  # Placeholder for feedback handling
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
        self.textBrowser.setText(message)
        return future.result()
    def run(self):
        self.ros_thread = threading.Thread(target=self.run_ros, daemon=True)
        self.ros_thread.start()
        
        self.window.show()
        sys.exit(self.app.exec_())
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