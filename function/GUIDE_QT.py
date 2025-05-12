import sys
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import tf
import threading
import requests
import json
import subprocess
import os
import signal
from matplotlib.widgets import Slider
from matplotlib.backend_bases import MouseButton
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import os
import signal

global ip
ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

class MapDisplayWindow(QtWidgets.QMainWindow):
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
        self.is_on = True
        self.zoom_level = 1.0
        self.pan_start = None
        self.nav_goal_mode = False
        self.zoom_slider = None

        self.setup_ui()
        self.create_connections()
        self.init_ros_subscriptions()
        self.update_position_continuously()
        self.pin()

    def setup_ui(self):
        self.setWindowTitle("MiR Map Viewer")

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QGridLayout(central_widget)

        # On/Off Button
        self.on_button = QPushButton("OFF")
        self.on_button.setStyleSheet("background-color: red; color: white;")
        main_layout.addWidget(self.on_button, 0, 0, 1, 1)

        # IP Entry
        ip_label = QLabel("Address IP:")
        main_layout.addWidget(ip_label, 1, 0, 1, 1)
        self.entry_ip = QLineEdit("192.168.0.172")
        main_layout.addWidget(self.entry_ip, 2, 0, 1, 1)
        
        # X,Y,Z,W Entry
        x_label = QLabel("X:")
        main_layout.addWidget(x_label, 0, 1)
        self.entry_x = QLineEdit()
        main_layout.addWidget(self.entry_x, 0, 2)

        y_label = QLabel("Y:")
        main_layout.addWidget(y_label, 1, 1)
        self.entry_y = QLineEdit()
        main_layout.addWidget(self.entry_y, 1, 2)

        z_label = QLabel("Z:")
        main_layout.addWidget(z_label, 2, 1)
        self.entry_z = QLineEdit()
        main_layout.addWidget(self.entry_z, 2, 2)

        w_label = QLabel("W:")
        main_layout.addWidget(w_label, 3, 1)
        self.entry_w = QLineEdit()
        main_layout.addWidget(self.entry_w, 3, 2)

        # Send Goal button
        send_button = QPushButton("Send Goal")
        main_layout.addWidget(send_button, 4, 0, 1, 2)
        self.send_button = send_button

         # 2D Nav Goal Button
        self.nav_goal_button = QPushButton("2D Nav Goal")
        main_layout.addWidget(self.nav_goal_button, 4, 2, 1, 1)

        # Matplotlib Canvas Setup
        self.fig = Figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, 5, 0, 1, 3)

        # Robot position label
        self.position_label = QLabel("Robot Position: (0.00, 0.00)")
        main_layout.addWidget(self.position_label, 6, 0, 1, 3)

        # Battery Percentage Display
        self.battery_percentage_value = "N/A"
        self.battery_percentage_display = QLabel("N/A")
        self.battery_percentage_display.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.battery_percentage_display.setLineWidth(2)
        self.battery_percentage_display.setAlignment(QtCore.Qt.AlignCenter)
        main_layout.addWidget(self.battery_percentage_display, 7, 0, 1, 3)

        # Add slider for zooming
        self.zoom_slider_ax = self.fig.add_axes([0.15, 0.01, 0.65, 0.03])
        self.zoom_slider = Slider(self.zoom_slider_ax, 'Zoom', 0.1, 5.0, valinit=1.0)
        self.zoom_slider.on_changed(self.update_zoom)

    def create_connections(self):
        self.on_button.clicked.connect(self.stop_mir_launch)
        self.send_button.clicked.connect(self.send_goal_thread)
        self.nav_goal_button.clicked.connect(self.toggle_nav_goal_mode)
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect('scroll_event', self.zoom_event)


    def init_ros_subscriptions(self):
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)
        self.tf_listener = tf.TransformListener()

    def stop_mir_launch(self):
        global ip
        ip = self.entry_ip.text()
        if not self.is_on:
            QtWidgets.QMessageBox.information(self, "Info", "MiR driver chưa được khởi chạy!")
            return

        try:
            ros_processes = subprocess.check_output(["pgrep", "-f", "ros"]).decode("utf-8").strip().split("\n")
            shell_processes = subprocess.check_output(["pgrep", "-f", "bash"]).decode("utf-8").strip().split("\n")
            all_processes = set(ros_processes + shell_processes)
            for pid in all_processes:
                try:
                    os.kill(int(pid), signal.SIGINT)
                except ProcessLookupError:
                    continue
            self.is_on = False
            QtWidgets.QMessageBox.information(self, "Info", "Đã ngắt kết nối, đóng các terminal và tắt giao diện!")

        except subprocess.CalledProcessError:
            QtWidgets.QMessageBox.information(self, "Info", "Không tìm thấy tiến trình terminal nào liên quan!")
        except Exception as e:
           QtWidgets.QMessageBox.critical(self, "Error", f"Lỗi khi đóng ứng dụng và các terminal:\n{e}")

        if QtWidgets.QMessageBox.question(self, "Confirm Exit", "Bạn có chắc muốn dừng chương trình không?", QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No) == QtWidgets.QMessageBox.Yes:
            rospy.signal_shutdown("User requested shutdown.")
            self.close()
            os._exit(0)

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        button_color = "green" if self.nav_goal_mode else "lightgray"
        self.nav_goal_button.setStyleSheet(f"background-color: {button_color};")

    def on_click(self, event):
        if event.button == MouseButton.LEFT:
            if event.inaxes == self.ax and self.nav_goal_mode:
                self.drag_start = (event.xdata, event.ydata)
        elif event.button == MouseButton.RIGHT and event.inaxes == self.ax:
            self.pan_start = (event.xdata, event.ydata)

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.ax:
            x_start, y_start = self.drag_start
            x_end, y_end = event.xdata, event.ydata

            self.entry_x.setText(str(x_start))
            self.entry_y.setText(str(y_start))

            self.ax.clear()
            self.draw_map_and_lidar()
            
            if (x_end is not None and y_end is not None):
                self.ax.annotate('', xy=(x_end, y_end), xytext=(x_start, y_start),
                                arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
            self.canvas.draw()

        elif self.pan_start is not None and event.inaxes == self.ax:
            x_start, y_start = self.pan_start
            x_end, y_end = event.xdata, event.ydata
            dx = x_end - x_start
            dy = y_end - y_start

            self.pan_start = (x_end, y_end)

            xmin, xmax = self.ax.get_xlim()
            ymin, ymax = self.ax.get_ylim()

            new_xmin = xmin - dx
            new_xmax = xmax - dx
            new_ymin = ymin - dy
            new_ymax = ymax - dy

            self.ax.set_xlim(new_xmin, new_xmax)
            self.ax.set_ylim(new_ymin, new_ymax)
            self.canvas.draw()

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata
            z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

            self.entry_z.setText(str(z))
            self.entry_w.setText(str(w))
            self.send_goal_thread()
        self.drag_start = None
        self.pan_start = None
    
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
        self.nav_goal_button.setStyleSheet("background-color: lightgray;")

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
        self.draw_map_and_lidar()

    def pin(self):
        try:
            b = requests.get(host + '/status', headers=headers)

            if b.status_code == 200:
                a = json.loads(b.content)
                p = a['battery_percentage']
                self.battery_percentage_display.setStyleSheet("background-color: lightgray;")

                if p <= 15:
                   self.battery_percentage_display.setStyleSheet("color: red;")
                else:
                    self.battery_percentage_display.setStyleSheet("color: blue;")
                
                value = round(p, 2)
                c = f'{value}%'
                self.battery_percentage_display.setText(c)
            else:
                print("Failed to retrieve battery status.")
        except Exception as e:
            print(f"Error occurred: {e}")

        QtCore.QTimer.singleShot(5000, self.pin)

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

        self.update_axes_limits()
        self.canvas.draw()
    
    def update_axes_limits(self):
        if self.map_data:
            map_width = self.map_data.info.width * self.resolution
            map_height = self.map_data.info.height * self.resolution
            
            center_x = self.origin[0] + map_width / 2
            center_y = self.origin[1] + map_height / 2

            width = map_width / self.zoom_level
            height = map_height / self.zoom_level

            xmin = center_x - width / 2
            xmax = center_x + width / 2
            ymin = center_y - height / 2
            ymax = center_y + height / 2

            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)
    
    def update_zoom(self, val):
         self.zoom_level = val
         self.draw_map_and_lidar()
    
    def zoom_event(self,event):
        if event.inaxes == self.ax:
          
          zoom_factor = 1.1 if event.button == 'up' else 1/1.1
          self.zoom_slider.set_val(self.zoom_slider.val * zoom_factor)
    
    def update_position_continuously(self):
        threading.Thread(target=self.update_position, daemon=True).start()
    
    def update_position(self):
         while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.robot_position = (trans[0], trans[1])
                self.position_label.setText(f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
                self.draw_map_and_lidar()
                rospy.sleep(0.1)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue
    

class FirstDisplayWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.is_on = False
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        self.robot_position = (0.0, 0.0)
        self.goal_position = None

        self.setWindowTitle("MiR Launcher")

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QGridLayout(central_widget)

        # ON/OFF Button
        self.on_button = QPushButton("ON")
        self.on_button.setStyleSheet("background-color: green; color: white;")
        main_layout.addWidget(self.on_button, 0, 0, 1, 1)

        # IP Address Label & Entry
        ip_label = QLabel("Address IP:")
        main_layout.addWidget(ip_label, 1, 0, 1, 1)
        self.entry_ip = QLineEdit("192.168.0.172")
        main_layout.addWidget(self.entry_ip, 2, 0, 1, 1)

        # X, Y, Z, W Entry
        x_label = QLabel("X:")
        main_layout.addWidget(x_label, 0, 1)
        self.entry_x = QLineEdit()
        main_layout.addWidget(self.entry_x, 0, 2)

        y_label = QLabel("Y:")
        main_layout.addWidget(y_label, 1, 1)
        self.entry_y = QLineEdit()
        main_layout.addWidget(self.entry_y, 1, 2)

        z_label = QLabel("Z:")
        main_layout.addWidget(z_label, 2, 1)
        self.entry_z = QLineEdit()
        main_layout.addWidget(self.entry_z, 2, 2)

        w_label = QLabel("W:")
        main_layout.addWidget(w_label, 3, 1)
        self.entry_w = QLineEdit()
        main_layout.addWidget(self.entry_w, 3, 2)

        # Send Goal Button
        send_button = QPushButton("Send Goal")
        main_layout.addWidget(send_button, 4, 0, 1, 2)

        # 2D Nav Goal Button
        self.nav_goal_button = QPushButton("2D Nav Goal")
        main_layout.addWidget(self.nav_goal_button, 4, 2, 1, 1)

         # Robot position label
        self.position_label = QLabel("Robot Position: (0.00, 0.00)")
        main_layout.addWidget(self.position_label, 6, 0, 1, 3)
        
        # Matplotlib Canvas
        self.fig = Figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas, 5, 0, 1, 3)
        
        self.on_button.clicked.connect(self.run_mir_launch)

    def run_mir_launch(self):
        global ip
        ip = self.entry_ip.text()

        if self.is_on:
            QtWidgets.QMessageBox.information(self, "Info", "MiR driver đã chạy trước đó!")
            return

        try:
            subprocess.Popen(['roslaunch', 'mir_driver', 'mir.launch', f'mir_hostname:={ip}'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.Popen(['rosrun', 'tf', 'tf_monitor'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.is_on = True
            QtWidgets.QMessageBox.information(self, "Info", "MiR driver và tf_monitor đã được khởi chạy thành công!")
            self.switch_to_next()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to start MiR driver:\n{e}")

    def switch_to_next(self):
         QtCore.QTimer.singleShot(5000, self.perform_switch)

    def perform_switch(self):
        rospy.init_node('robot_gui', anonymous=True)
        self.hide()
        self.map_window = MapDisplayWindow()
        self.map_window.show()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    first_window = FirstDisplayWindow()
    first_window.show()
    sys.exit(app.exec_())