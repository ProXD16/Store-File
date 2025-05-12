import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Twist
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
        self.robot_orientation = 0.0
        self.goal_position = None
        self.f_points = []
        self.b_points = []
        self.is_moving = False
        self.drag_start = None
        self.path_points = []
        self.nav_goal_mode = False
        self.data_matrix = []
        self.arrow = None
        self.background = None  # Lưu nền để sử dụng trong blitting
        self.robot_patch = None  # Đối tượng vẽ robot
        self.goal_point = None  # Đối tượng vẽ mục tiêu
        self.lidar_points = None  # Đối tượng vẽ dữ liệu Lidar
        self.path_line = None  # Đối tượng vẽ đường đi

        # Biến lưu trữ dữ liệu cũ
        self.last_f_points = []
        self.last_b_points = []
        self.last_robot_position = (0.0, 0.0)
        self.last_path_points = []

        self.initUI()
        self.initROS()
        self.last_robot_update = rospy.Time.now()
        self.linear_speed = 0.2 # Default linear speed
        self.angular_speed = 0.4 # Default angular speed

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

        self.control_layout.addWidget(QLabel("X:"))
        self.control_layout.addWidget(self.entry_x)
        self.control_layout.addWidget(QLabel("Y:"))
        self.control_layout.addWidget(self.entry_y)
        self.move_button = QPushButton("Move")
        self.move_button.clicked.connect(self.move_to_goal_thread)
        self.control_layout.addWidget(self.move_button)

        # Add stop button
        self.stop_button = QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_movement)
        self.control_layout.addWidget(self.stop_button)

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
        self.timer.start(200)  # Cập nhật vị trí mỗi 200ms

    def initROS(self):
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)
        self.tf_listener = tf.TransformListener()
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        self.nav_goal_button.setStyleSheet("background-color: green" if self.nav_goal_mode else "background-color: lightgray")

    def on_click(self, event):
        if event.inaxes == self.ax and self.nav_goal_mode:
            self.drag_start = (event.xdata, event.ydata)
            # Tạo một annotation (mũi tên) ban đầu
            self.arrow = self.ax.annotate('', xy=self.drag_start, xytext=self.drag_start,
                                          arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
            self.canvas.draw()
            if hasattr(self, 'ax') and hasattr(self, 'canvas') and self.ax.bbox:
                 self.background = self.canvas.copy_from_bbox(self.ax.bbox)

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.ax:
            x_end, y_end = event.xdata, event.ydata
            if x_end is not None and y_end is not None:
                # Khôi phục nền và cập nhật mũi tên
                if self.background:
                    self.canvas.restore_region(self.background)
                self.arrow.xy = (x_end, y_end)
                self.ax.draw_artist(self.arrow)
                if self.background:
                    self.canvas.blit(self.ax.bbox)

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata

            self.entry_x.setText(str(self.drag_start[0])) #Save to lineEdit for direct control
            self.entry_y.setText(str(self.drag_start[1]))

            self.arrow.remove()
            self.drag_start = None
            self.canvas.draw()
    def move_to_goal_thread(self):
        """Move to goal on another thread"""
        threading.Thread(target=self.move_to_goal).start()

    def move_to_goal(self):
        """Move to the specified goal using velocity commands."""
        try:
            target_x = float(self.entry_x.text())
            target_y = float(self.entry_y.text())
        except ValueError:
            rospy.logerr("Invalid input for goal position.")
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"TF error: {e}")
            return

        angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)

        # Rotate to face the point
        while abs(angle_error) > 0.05 and not self.stop_moving:
            angular_vel = self.angular_speed if angle_error > 0 else -self.angular_speed
            self.send_velocity(0.0, angular_vel)
            rospy.sleep(0.02)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x, robot_y = trans[0], trans[1]
                robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF error: {e}")
                return
            angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)
        self.send_velocity(0.0, 0.0)

        # Move forward to the point
        distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
        while distance > self.distance_tolerance and not self.stop_moving:
            linear_vel = self.linear_speed
            self.send_velocity(linear_vel, 0.0)
            rospy.sleep(0.02)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x = trans[0], trans[1]
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF error: {e}")
                return
            distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
        self.send_velocity(0.0, 0.0)
        rospy.loginfo("Arrived at goal.")

    def stop_movement(self):
        """Stops the robot's movement."""
        self.stop_moving = True
        self.send_velocity(0.0, 0.0)

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
            self.goal_position = (x, y)
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

    def map_callback(self, data):
        self.map_data = data
        self.resolution = data.info.resolution
        self.origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.draw_map_and_lidar()

    def f_scan_callback(self, msg):
        self.f_points = self.process_lidar_data(msg, False)
        timestamp = rospy.Time.now().to_sec()
        self.data_matrix.append([timestamp, "f_scan", self.f_points, None, None, None])
        self.update_lidar_points()

    def b_scan_callback(self, msg):
        self.b_points = self.process_lidar_data(msg, True)
        timestamp = rospy.Time.now().to_sec()
        self.data_matrix.append([timestamp, "b_scan", None, self.b_points, None, None,])
        self.update_lidar_points()

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
        timestamp = rospy.Time.now().to_sec()
        self.data_matrix.append([timestamp, "path", None, None, self.path_points, None])
        self.update_path()

    def draw_map_and_lidar(self):
        if self.conf is None:
            return

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
                robot_shape = Polygon(corners, closed=True, color='white', alpha=0.4)
                self.ax.add_patch(robot_shape)
                tri_side = 0.3
                triangle_points = [
                    (x + tri_side * np.cos(yaw), y + tri_side * np.sin(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) + (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                    y - tri_side / 2 * np.sin(yaw) - (tri_side * np.sqrt(3) / 2) * np.cos(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) - (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                    y - tri_side / 2 * np.sin(yaw) + (tri_side * np.sqrt(3) / 2) * np.cos(yaw))
                ]
                direction_triangle = Polygon(triangle_points, closed=True, color='blue', alpha=0.7)
                self.ax.add_patch(direction_triangle)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        if self.f_points:
            x_vals, y_vals = zip(*self.f_points)
            self.ax.plot(x_vals, y_vals, "r.")

        if self.b_points:
            x_vals, y_vals = zip(*self.b_points)
            self.ax.plot(x_vals, y_vals, "b.")

        if self.path_points:
            path_x, path_y = zip(*self.path_points)
            self.ax.plot(path_x, path_y, 'g--', label='Path')
        if hasattr(self, 'ax') and hasattr(self, 'canvas') and self.ax.bbox:
            self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.canvas.draw()
    def update_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_position = (trans[0], trans[1])
            self.robot_orientation = tf.transformations.euler_from_quaternion(rot)[2] # Get yaw
            self.position_label.setText(f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
            self.update_robot_patch() #Re-render robot pose in GUI
            # Check that the time passed for last message is greater that the time passed by loop
            timestamp = rospy.Time.now().to_sec()
            self.data_matrix.append([timestamp, "robot_pose", None, None, None, self.robot_position])
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass

    def update_robot_patch(self):
        if not hasattr(self, 'ax') or not hasattr(self, 'canvas'):
            return
        if self.robot_patch:
            self.robot_patch.remove()
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
        if self.background:
            if hasattr(self, 'background') and self.background:
                 self.canvas.restore_region(self.background)
            self.ax.draw_artist(self.robot_patch)
            self.ax.draw_artist(direction_triangle)
            if hasattr(self, 'background') and self.background:
                 self.canvas.blit(self.ax.bbox)

    def update_lidar_points(self):
         if not hasattr(self, 'ax') or not hasattr(self, 'canvas'):
            return
         all_points = self.f_points + self.b_points
         if all_points:
            x_vals, y_vals = zip(*all_points)
            if self.lidar_points is None:
                 self.lidar_points = self.ax.scatter(x_vals, y_vals, color='red', s=1)
            else:
                 self.lidar_points.set_offsets(np.c_[x_vals, y_vals])
            if self.background:
               if hasattr(self, 'background') and self.background:
                   self.canvas.restore_region(self.background)
               self.ax.draw_artist(self.lidar_points)
               if hasattr(self, 'background') and self.background:
                    self.canvas.blit(self.ax.bbox)
         elif self.lidar_points is not None:
            self.lidar_points.set_offsets(np.array([]))

    def update_path(self):
        if not hasattr(self, 'ax') or not hasattr(self, 'canvas'):
            return
        if self.path_line:
            self.path_line.remove()
        if self.path_points:
            path_x, path_y = zip(*self.path_points)
            self.path_line, = self.ax.plot(path_x, path_y, 'g--', linewidth=2)
            if self.background:
               if hasattr(self, 'background') and self.background:
                    self.canvas.restore_region(self.background)
               self.ax.draw_artist(self.path_line)
               if hasattr(self, 'background') and self.background:
                   self.canvas.blit(self.ax.bbox)
    def calculate_angle_to_line(self, target_x, target_y, robot_x, robot_y, robot_yaw):
        """Tính góc giữa hướng của robot và hướng đến điểm đích."""
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        angle_diff = angle_to_target - robot_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        return angle_diff

    def send_velocity(self, linear_vel, angular_vel):
        """Gửi vận tốc cho robot."""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_publisher.publish(twist)
    
    def move_to_goal(self):
        try:
            target_x = float(self.entry_x.text())
            target_y = float(self.entry_y.text())
        except ValueError:
            rospy.logerr("Invalid input for goal position.")
            return

        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"TF error: {e}")
            return

        angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)

        # Rotate to face the point
        while abs(angle_error) > 0.05 and not self.stop_moving:
            angular_vel = self.angular_speed if angle_error > 0 else -self.angular_speed
            self.send_velocity(0.0, angular_vel)
            rospy.sleep(0.02)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x, robot_y = trans[0], trans[1]
                robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF error: {e}")
                return
            angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)
        self.send_velocity(0.0, 0.0)

        # Move forward to the point
        distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
        while distance > self.distance_tolerance and not self.stop_moving:
            linear_vel = self.linear_speed
            self.send_velocity(linear_vel, 0.0)
            rospy.sleep(0.02)
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x = trans[0], trans[1]
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF error: {e}")
                return
            distance = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
        self.send_velocity(0.0, 0.0)
        rospy.loginfo("Arrived at goal.")
    def move_along_path_thread(self):
         """Hàm chạy move_along_path trong một thread riêng biệt."""
         threading.Thread(target=self.move_to_goal).start()

    def stop_movement(self):
        """Stops the robot's movement."""
        self.stop_moving = True
        self.send_velocity(0.0, 0.0) # Stop the robot
        rospy.loginfo("Movement stopped.")

if __name__ == "__main__":
    rospy.init_node("map_display")
    app = QApplication(sys.argv)
    window = MapDisplay()
    window.show()
    sys.exit(app.exec_())
