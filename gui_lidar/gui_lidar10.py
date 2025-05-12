#!/usr/bin/env python
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
import threading
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MapDisplay(QMainWindow):
    def __init__(self):
        super().__init__()
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        self.robot_position = (0.0, 0.0)
        self.goal_position = None
        self.f_points = []
        self.b_points = []
        self.is_moving = False
        self.drag_start = None
        self.path_points = []
        self.nav_goal_mode = False
        self.data_matrix = []  # Ma trận 2 chiều để lưu dữ liệu theo thời gian

        self.initUI()
        self.initROS()

    def initUI(self):
        self.setWindowTitle("Map Display")
        self.setGeometry(100, 100, 800, 800)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout(self.central_widget)

        self.fig = Figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        self.layout.addWidget(self.canvas)

        self.control_layout = QHBoxLayout()
        self.layout.addLayout(self.control_layout)

        self.entry_x = QLineEdit()
        self.entry_y = QLineEdit()
        self.entry_z = QLineEdit()
        self.entry_w = QLineEdit()

        self.control_layout.addWidget(QLabel("X:"))
        self.control_layout.addWidget(self.entry_x)
        self.control_layout.addWidget(QLabel("Y:"))
        self.control_layout.addWidget(self.entry_y)
        self.control_layout.addWidget(QLabel("Z:"))
        self.control_layout.addWidget(self.entry_z)
        self.control_layout.addWidget(QLabel("W:"))
        self.control_layout.addWidget(self.entry_w)

        self.send_button = QPushButton("Send Goal")
        self.send_button.clicked.connect(self.send_goal_thread)
        self.control_layout.addWidget(self.send_button)

        self.nav_goal_button = QPushButton("2D Nav Goal")
        self.nav_goal_button.clicked.connect(self.toggle_nav_goal_mode)
        self.control_layout.addWidget(self.nav_goal_button)

        self.position_label = QLabel("Robot Position: (0.00, 0.00)")
        self.layout.addWidget(self.position_label)

        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)
        self.timer.start(100)  # Cập nhật vị trí mỗi 100ms

    def initROS(self):
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)
        self.tf_listener = tf.TransformListener()

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        self.nav_goal_button.setStyleSheet("background-color: green" if self.nav_goal_mode else "background-color: lightgray")

    # def on_click(self, event):
    #     if event.inaxes == self.ax and self.nav_goal_mode:
    #         self.drag_start = (event.xdata, event.ydata)

    # def on_drag(self, event):
    #     if self.drag_start is not None and event.inaxes == self.ax:
    #         x_start, y_start = self.drag_start
    #         x_end, y_end = event.xdata, event.ydata

    #         self.entry_x.setText(str(x_start))
    #         self.entry_y.setText(str(y_start))

    #         self.ax.clear()
    #         self.draw_map_and_lidar()

    #         if x_end is not None and y_end is not None:
    #             self.ax.annotate('', xy=(x_end, y_end), xytext=(x_start, y_start),
    #                             arrowprops=dict(arrowstyle='->', color='red', linewidth=2))

    #         self.canvas.draw()

    # def on_release(self, event):
    #     if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
    #         x_end, y_end = event.xdata, event.ydata
    #         z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

    #         self.entry_z.setText(str(z))
    #         self.entry_w.setText(str(w))
    #         self.send_goal_thread()
    #     self.drag_start = None

    def on_click(self, event):
        if event.inaxes == self.ax and self.nav_goal_mode:
            self.drag_start = (event.xdata, event.ydata)
            # Tạo một annotation (mũi tên) ban đầu
            self.arrow = self.ax.annotate('', xy=self.drag_start, xytext=self.drag_start,
                                        arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
            self.canvas.draw()

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.ax:
            x_end, y_end = event.xdata, event.ydata
            if x_end is not None and y_end is not None:
                # Cập nhật vị trí của mũi tên
                self.arrow.xy = (x_end, y_end)
                self.canvas.draw()

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata
            z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

            self.entry_x.setText(str(self.drag_start[0]))
            self.entry_y.setText(str(self.drag_start[1]))
            self.entry_z.setText(str(z))
            self.entry_w.setText(str(w))

            # Gửi mục tiêu và xóa mũi tên
            self.send_goal_thread()
            self.arrow.remove()
            self.drag_start = None
            self.canvas.draw()

    def calculate_orientation(self, start, end):
        orientation = np.arctan2(end[1] - start[1], end[0] - start[0])
        z = np.sin(orientation / 2)
        w = np.cos(orientation / 2)
        return z, w

    def send_goal_thread(self):
        threading.Thread(target=self.send_goal).start()

    def send_goal(self):
        try:
            x = float(self.entry_x.text())
            y = float(self.entry_y.text())
            z = float(self.entry_z.text())
            w = float(self.entry_w.text())
            self.movebase_client(x, y, z, w)
        except ValueError:
            rospy.logerr("Invalid input for goal position.")

    def movebase_client(self, x, y, z, w):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w

        self.is_moving = True
        client.send_goal(goal)
        self.nav_goal_button.setStyleSheet("background-color: lightgray")

        client.wait_for_result()
        self.is_moving = False
        self.nav_goal_mode = False
        self.goal_position = None

    def rviz_goal_callback(self, goal_msg):
        x = goal_msg.pose.position.x
        y = goal_msg.pose.position.y
        z = goal_msg.pose.orientation.z
        w = goal_msg.pose.orientation.w
        self.goal_position = (x, y)

        self.entry_x.setText(str(x))
        self.entry_y.setText(str(y))
        self.entry_z.setText(str(z))
        self.entry_w.setText(str(w))

        self.movebase_client(x, y, z, w)

    def map_callback(self, data):
        self.map_data = data
        self.resolution = data.info.resolution
        self.origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.draw_map_and_lidar()

    def f_scan_callback(self, msg):
        self.f_points = self.process_lidar_data(msg, False)

    def b_scan_callback(self, msg):
        self.b_points = self.process_lidar_data(msg, True)

    def process_lidar_data(self, msg, is_back):
        points = []
        frame_id = '/front_laser_link' if not is_back else '/back_laser_link'
        for i in range(len(msg.ranges)):
            r = msg.ranges[i]
            angle = i * msg.angle_increment + msg.angle_min
            if r < msg.range_max and r > msg.range_min:
                x = r * np.cos(angle)
                y = r * np.sin(angle)

                point_lidar = PointStamped()
                point_lidar.header.frame_id = frame_id
                point_lidar.header.stamp = rospy.Time(0)
                point_lidar.point.x = x
                point_lidar.point.y = y

                try:
                    point_map = self.tf_listener.transformPoint("map", point_lidar)
                    points.append((point_map.point.x, point_map.point.y))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

        return points

    def path_callback(self, path_msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]

    def draw_map_and_lidar(self):
        self.ax.clear()
        if self.map_data is not None:
            map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            map_array = np.where(map_array == -1, np.nan, map_array)

            self.ax.imshow(map_array, cmap='gray', origin='lower', extent=(
                self.origin[0], self.origin[0] + self.map_data.info.width * self.resolution,
                self.origin[1], self.origin[1] + self.map_data.info.height * self.resolution
            ))

        if self.goal_position:
            self.ax.plot(self.goal_position[0], self.goal_position[1], "go")

        if self.robot_position:
            x, y = self.robot_position

            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                x, y = trans[0], trans[1]
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                rect_length = 0.6
                rect_width = 0.4
                corners = [
                    (x + rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
                     y + rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw)),
                    (x + rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
                     y + rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
                    (x - rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
                     y - rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
                    (x - rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
                     y - rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw))
                ]
                robot_shape = plt.Polygon(corners, closed=True, color='white', alpha=0.4)
                self.ax.add_patch(robot_shape)
                tri_side = 0.3
                triangle_points = [
                    (x + tri_side * np.cos(yaw), y + tri_side * np.sin(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) + (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                     y - tri_side / 2 * np.sin(yaw) - (tri_side * np.sqrt(3) / 2) * np.cos(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) - (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                     y - tri_side / 2 * np.sin(yaw) + (tri_side * np.sqrt(3) / 2) * np.cos(yaw))
                ]
                direction_triangle = plt.Polygon(triangle_points, closed=True, color='blue', alpha=0.7)
                self.ax.add_patch(direction_triangle)

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        if self.f_points:
            x_vals, y_vals = zip(*self.f_points)
            self.ax.plot(x_vals, y_vals, "r.")

        if self.b_points:
            x_vals, y_vals = zip(*self.b_points)
            self.ax.plot(x_vals, y_vals, "r.")

        if self.path_points:
            path_x, path_y = zip(*self.path_points)
            self.ax.plot(path_x, path_y, 'g--', label='Path')

        self.canvas.draw()

    def update_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_position = (trans[0], trans[1])
            self.position_label.setText(f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
            self.draw_map_and_lidar()

            # Lưu dữ liệu vào ma trận 2 chiều
            self.data_matrix.append([trans[0], trans[1], rospy.get_time()])
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass

if __name__ == "__main__":
    rospy.init_node("map_display")
    app = QApplication(sys.argv)
    window = MapDisplay()
    window.show()
    sys.exit(app.exec_())