#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tf
import threading
import math
from sensor_msgs.msg import LaserScan
from matplotlib.patches import Polygon
from teleop_control import TeleopControl
import matplotlib.patches as patches

class MapDisplay:
    def __init__(self, master):
        self.master = master
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        self.robot_position = (0.0, 0.0)
        self.robot_orientation = 0.0
        self.goal_position = None
        self.is_moving = False
        self.drag_start = None
        self.path_points = []
        self.lines = []
        self.selected_line = None
        self.drawing_line = False
        self.temp_line_start = None
        self.temp_line_end = None
        self.drawing_circle = False
        self.circles = []
        self.drawing_circle = False
        self.temp_circle_start = None
        self.temp_circle_end = None
        self.stop_moving = False
        self.distance_tolerance = 0.1
        self.segment_length = 0.2
        self.f_points = []
        self.b_points = []
        self.lidar_points = []
        self.data_matrix = []
        self.lidar_points_plot = None

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)
        self.tf_listener = tf.TransformListener()
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.radius_circle = 1
        self.angle_circle = 0
        self.arc_patch = None
        self.clockwise_var = tk.BooleanVar(value=True)
        self.counterclockwise_var = tk.BooleanVar(value=False)

        self.fig = Figure(figsize=(6, 6))
        self.conf = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)

        self.create_notebook()
        self.create_widgets()
        self.init_blit()
        self.update_position_continuously()
        self.teleop_control = TeleopControl(self.main_tab, self.cmd_vel_publisher, self.linear_speed, self.angular_speed)

    def init_blit(self):
        if not hasattr(self, 'conf'):
            return
        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.conf.bbox)

    def create_notebook(self):
        self.notebook = ttk.Notebook(self.master)
        self.notebook.grid(row=0, column=0, columnspan=2, sticky="nsew") 
        self.main_tab = ttk.Frame(self.notebook)
        self.drawing_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.main_tab, text="Điều khiển")
        self.notebook.add(self.drawing_tab, text="Vẽ")
        self.canvas.get_tk_widget().grid(row=1, column=0, columnspan=2, sticky="nsew")

    def create_widgets(self):
        goal_frame = ttk.Frame(self.main_tab)
        goal_frame.grid(row=0, column=0, columnspan=2, pady=5)

        tk.Label(goal_frame, text="X:").grid(row=0, column=0)
        self.entry_x = tk.Entry(goal_frame)
        self.entry_x.grid(row=0, column=1)

        tk.Label(goal_frame, text="Y:").grid(row=0, column=2)
        self.entry_y = tk.Entry(goal_frame)
        self.entry_y.grid(row=0, column=3)

        tk.Label(goal_frame, text="Z:").grid(row=1, column=0)
        self.entry_z = tk.Entry(goal_frame)
        self.entry_z.grid(row=1, column=1)

        tk.Label(goal_frame, text="W:").grid(row=1, column=2)
        self.entry_w = tk.Entry(goal_frame)
        self.entry_w.grid(row=1, column=3)

        send_button = tk.Button(self.main_tab, text="Send Goal", command=self.send_goal_thread)
        send_button.grid(row=2, column=0, pady=5)

        self.nav_goal_button = tk.Button(self.main_tab, text="2D Nav Goal", command=self.toggle_nav_goal_mode)
        self.nav_goal_button.grid(row=2, column=1)

        speed_frame = ttk.Frame(self.main_tab)
        speed_frame.grid(row=3, column=0, columnspan=2, pady=5)

        tk.Label(speed_frame, text="Linear Speed:").grid(row=0, column=0)
        self.entry_linear_speed = tk.Entry(speed_frame)
        self.entry_linear_speed.insert(0, str(self.linear_speed))
        self.entry_linear_speed.grid(row=0, column=1)
        self.entry_linear_speed.bind("<FocusOut>", lambda event: self.update_teleop_speed())

        tk.Label(speed_frame, text="Angular Speed:").grid(row=1, column=0)
        self.entry_angular_speed = tk.Entry(speed_frame)
        self.entry_angular_speed.insert(0, str(self.angular_speed))
        self.entry_angular_speed.grid(row=1, column=1)
        self.entry_angular_speed.bind("<FocusOut>",  lambda event: self.update_teleop_speed())

        move_button = tk.Button(self.main_tab, text="Move along line", command=self.move_along_path_thread)
        move_button.grid(row=4, column=0, columnspan=2, pady=5)

        move_circle = tk.Button(self.main_tab, text="Move along arc", command=self.move_along_arc_thread)
        move_circle.grid(row=4, column=1, columnspan=2, pady=5)

        stop_button = tk.Button(self.main_tab, text="Stop Emergency", command=self.stop_movement)
        stop_button.grid(row=5, column=0, columnspan=2, pady=5)

        self.position_label = tk.Label(self.main_tab, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=6, column=0, columnspan=2)

        line_frame = ttk.Frame(self.drawing_tab)
        line_frame.grid(row=0, column=0, columnspan=2, pady=5)

        self.draw_line_button = tk.Button(line_frame, text="Draw Line", command=self.toggle_draw_line_mode)
        self.draw_line_button.grid(row=0, column=0)

        self.delete_line_button = tk.Button(line_frame, text="Delete Line", command=self.delete_line)
        self.delete_line_button.grid(row=0, column=1)

        self.draw_circle_button = tk.Button(line_frame, text="Draw Arc", command=self.toggle_draw_circle_mode)
        self.draw_circle_button.grid(row=1, column=0)

        tk.Label(line_frame, text="Radius:").grid(row=1, column=1)
        self.entry_radius_circle= tk.Entry(line_frame)
        self.entry_radius_circle.insert(1, str(self.radius_circle))
        self.entry_radius_circle.grid(row=1, column=2)

        tk.Label(line_frame, text="Mode Angle:").grid(row=2, column=1)
        self.entry_angle_circle= tk.Entry(line_frame)
        self.entry_angle_circle.insert(1, str(self.angle_circle))
        self.entry_angle_circle.grid(row=2, column=2)

        self.clockwise_var = tk.BooleanVar(value=True)
        self.counterclockwise_var = tk.BooleanVar(value=False)

        self.clockwise_check = tk.Checkbutton(line_frame, text="Clockwise", variable=self.clockwise_var, command=self.toggle_direction_clockwise)
        self.clockwise_check.grid(row=3, column=0)

        self.counterclockwise_check = tk.Checkbutton(line_frame, text="Counterclockwise", variable=self.counterclockwise_var, command=self.toggle_direction_counterclockwise)
        self.counterclockwise_check.grid(row=3, column=1)

        self.nav_goal_mode = False
        self.draw_line_mode = False
        self.draw_circle_mode = False

    def toggle_direction_clockwise(self):
        if self.clockwise_var.get():
            self.counterclockwise_var.set(False)
        else:
            self.clockwise_var.set(False)

    def toggle_direction_counterclockwise(self):
        if self.counterclockwise_var.get():
            self.clockwise_var.set(False)
        else:
            self.counterclockwise_var.set(False)

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        button_color = "green" if self.nav_goal_mode else "lightgray"
        self.nav_goal_button.config(bg=button_color)
        self.draw_line_mode = False
        self.draw_line_button.config(bg='lightgray')

    def toggle_draw_line_mode(self):
        self.draw_line_mode = not self.draw_line_mode
        button_color = "green" if self.draw_line_mode else "lightgray"
        self.draw_line_button.config(bg=button_color)
        self.nav_goal_mode = False
        self.nav_goal_button.config(bg="lightgray")

    def toggle_draw_circle_mode(self):
        self.draw_circle_mode = not self.draw_circle_mode
        print(f"Draw circle mode: {self.draw_circle_mode}")  
        button_color = "green" if self.draw_circle_mode else "lightgray"
        self.draw_circle_button.config(bg=button_color)
        self.nav_goal_mode = False
        self.draw_line_mode = False

        self.nav_goal_button.config(bg="lightgray")
        self.draw_line_button.config(bg="lightgray")

    def on_click(self, event):
        if event.inaxes == self.conf:
            if self.nav_goal_mode:
                self.drag_start = (event.xdata, event.ydata)
            elif self.draw_line_mode:
                if not self.drawing_line:
                    self.drawing_line = True
                    if self.lines:
                        self.temp_line_start = self.lines[-1][1]
                        self.temp_line_end = self.lines[-1][1]
                    else:
                        try:
                            (trans, _) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                            self.temp_line_start = (trans[0], trans[1])
                            self.temp_line_end = (trans[0], trans[1])
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            self.drawing_line = False
                            rospy.logerr("Could not get robot position for line start.")
                            return
                else:
                    self.temp_line_end = (event.xdata, event.ydata)
            elif self.draw_circle_mode:
                try:
                    (trans, _) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    x, y = trans[0], trans[1]
                    end_x = event.xdata
                    end_y = event.ydata
                    print("Toa do di chuyen tren cung tron: " + str(event.xdata) + " " + str(event.ydata))
                    radius = float(self.entry_radius_circle.get()) 
                    self.circles = []
                    self.circles.append(((x, y),(end_x, end_y) ,radius))
                    self.display_map()
                    self.conf.patches.clear()
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                  rospy.logerr(f"TF error: {e}")
            self.canvas.draw()

    def on_drag(self, event):
        if event.inaxes == self.conf:
            if self.nav_goal_mode and self.drag_start:
                self.entry_x.delete(0, tk.END)
                self.entry_x.insert(0, str(self.drag_start[0]))
                self.entry_y.delete(0, tk.END)
                self.entry_y.insert(0, str(self.drag_start[1]))

                self.conf.clear()
                self.display_map()

                if event.xdata is not None and event.ydata is not None:
                    self.conf.annotate('', xy=(event.xdata, event.ydata), xytext=self.drag_start,
                                       arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
                self.canvas.draw()
            elif self.draw_line_mode and self.drawing_line:
                self.temp_line_end = (event.xdata, event.ydata)
                print("Toa do dich: " + str(event.xdata) + " " + str(event.ydata))  
                self.conf.clear()
                self.display_map()
                self.canvas.draw()

    def on_release(self, event):
        if event.inaxes == self.conf:
            if self.nav_goal_mode and self.drag_start:
                x_end, y_end = event.xdata, event.ydata
                z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))
                self.entry_z.delete(0, tk.END)
                self.entry_z.insert(0, str(z))
                self.entry_w.delete(0, tk.END)
                self.entry_w.insert(0, str(w))
                self.send_goal_thread()
                self.drag_start = None
            elif self.draw_line_mode and self.drawing_line:
                self.drawing_line = False
                if self.temp_line_start and self.temp_line_end:
                    self.lines.append((self.temp_line_start, self.temp_line_end))
                    self.temp_line_start = None
                    self.temp_line_end = None
                self.display_map()
                self.canvas.draw()

    def on_pick(self, event):
        if self.lines:
            artist = event.artist
            for i, (start, end) in enumerate(self.lines):
                x_values = [start[0], end[0]]
                y_values = [start[1], end[1]]
                line = self.conf.plot(x_values, y_values, 'r-', picker=True, label=f"Line {i}")
                self.conf.plot(x_values, y_values, 'ro', picker=True, markersize=2)
                if artist == line:
                    self.selected_line = i
                    break

    def set_radius(self, text):
        try:
            self.radius_circle = float(text)
        except ValueError:
            rospy.logwarn("Invalid input for radius. Please enter a number.")
            self.radius = 0
    
    def f_scan_callback(self, msg):
        self.f_points = self.process_lidar_data(msg, False)
        timestamp = rospy.Time.now().to_sec()
        self.data_matrix.append([timestamp, "f_scan", self.f_points, None, None, None])
        self.update_lidar_points()

    def b_scan_callback(self, msg):
        self.b_points = self.process_lidar_data(msg, True)
        timestamp = rospy.Time.now().to_sec()
        self.data_matrix.append([timestamp, "b_scan", None, self.b_points, None, None])
        self.update_lidar_points()

    def process_lidar_data(self, msg, is_back):
        points = []
        frame_id = '/front_laser_link' if not is_back else '/back_laser_link'
        point_lidar = PointStamped()
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
                point_lidar.point.z = 0

            try:
                point_map = self.tf_listener.transformPoint("map", point_lidar)
                points.append((point_map.point.x, point_map.point.y))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        return points

    def update_lidar_points(self):
        if self.conf is None:
            return
        try:
            if not self.f_points and not self.b_points:
                self.lidar_points = []
            else:
                self.lidar_points = self.f_points + self.b_points

            if not self.lidar_points:
                if self.lidar_points_plot is not None:
                    self.lidar_points_plot.set_data([], [])
                    self.canvas.restore_region(self.background)
                    self.conf.draw_artist(self.lidar_points_plot)
                    self.canvas.blit(self.conf.bbox)
                return

            x_vals, y_vals = zip(*self.lidar_points)
            if self.lidar_points_plot is None:
                self.lidar_points_plot, = self.conf.plot(x_vals, y_vals, 'r.', markersize=2)  
                self.canvas.draw()
            else:
                self.lidar_points_plot.set_data(x_vals, y_vals)
                self.canvas.restore_region(self.background)
                self.conf.draw_artist(self.lidar_points_plot)
                self.canvas.blit(self.conf.bbox)
        except Exception as e:
            rospy.logwarn(f"Lidar exception: {e}")

    def calculate_orientation(self, start, end):
        orientation = np.arctan2(end[1] - start[1], end[0] - start[0])
        z = np.sin(orientation / 2)
        w = np.cos(orientation / 2)
        return z, w

    def send_goal_thread(self):
        threading.Thread(target=self.send_goal).start()

    def send_goal(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            w = float(self.entry_w.get())
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
        self.nav_goal_button.config(bg="lightgray")

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

        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, str(x))
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, str(y))
        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, str(z))
        self.entry_w.delete(0, tk.END)
        self.entry_w.insert(0, str(w))

    def map_callback(self, data):
        self.map_data = data
        self.resolution = data.info.resolution
        self.origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.display_map()

    def path_callback(self, path_msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self.display_map()

    def display_map(self):
        if self.map_data is not None:
            data_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            data_array = np.where(data_array == -1, np.nan, data_array)

            self.conf.clear()
            self.conf.imshow(data_array, cmap='gray', origin='lower', extent=[
                self.origin[0], self.origin[0] + self.map_data.info.width * self.resolution,
                self.origin[1], self.origin[1] + self.map_data.info.height * self.resolution
            ])
            self.conf.set_title('Map')
            self.draw_robot()
            if self.goal_position:
                self.conf.plot(self.goal_position[0], self.goal_position[1], 'go', label='Goal Position')
            if self.path_points:
                path_x, path_y = zip(*self.path_points)
                self.conf.plot(path_x, path_y, 'b--', label='Path')
            for i, (start, end) in enumerate(self.lines):
                self.path_points = []
                x_values = [start[0], end[0]]
                y_values = [start[1], end[1]]
                line = self.conf.plot(x_values, y_values, 'r-', picker=True, label=f"Line {i}")
                self.conf.plot(x_values, y_values, 'ro', picker=True, markersize=2)

            if self.drawing_line and self.temp_line_start and self.temp_line_end:
                x_values = [self.temp_line_start[0], self.temp_line_end[0]]
                y_values = [self.temp_line_start[1], self.temp_line_end[1]]
                self.conf.plot(x_values, y_values, 'r--')

            if self.f_points:
                x_vals, y_vals = zip(*self.f_points)
                self.conf.plot(x_vals, y_vals, "r.")

            if self.b_points:
                x_vals, y_vals = zip(*self.b_points)
                self.conf.plot(x_vals, y_vals, "b.")

            for circle_start, circle_end, circle_radius in self.circles: 
                mode = float(self.entry_angle_circle.get())
                x1, y1 = circle_start
                x2, y2 = circle_end
                radius = circle_radius 
                mid_x = (x1 + x2)/2
                mid_y = (y1 + y2)/2
                d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2) 
                h = math.sqrt(radius**2 - (d/2)**2) 
                pt_angle = math.atan2((y2-y1),(x2-x1)) 
                center_x = mid_x - h * math.sin(pt_angle) 
                center_y = mid_y + h * math.cos(pt_angle) 
                startAngle = math.atan2(y1- center_y, x1 - center_x) *180 / math.pi
                endAngle   = math.atan2(y2- center_y, x2 - center_x) *180 / math.pi 
                if not mode:
                    if self.counterclockwise_var.get():
                        arc = patches.Arc((center_x, center_y), width= radius*2, height = radius*2 , angle = 0, theta1 = startAngle, theta2 = endAngle, color = 'green')
                        self.conf.add_patch(arc)
                    elif self.clockwise_var.get():
                        center_x = 2 * mid_x - center_x
                        center_y = 2 * mid_y - center_y
                        startAngle = math.atan2(y1- center_y, x1 - center_x) *180 / math.pi
                        endAngle   = math.atan2(y2- center_y, x2 - center_x) *180 / math.pi 
                        arc = patches.Arc((center_x, center_y), width= radius*2, height = radius*2 , angle = 0, theta1 = endAngle, theta2 = startAngle+math.pi, color = 'green')
                        self.conf.add_patch(arc)
                else:
                    if self.clockwise_var.get():
                        arc = patches.Arc((center_x, center_y), width= radius*2, height = radius*2 , angle = 0, theta1 = endAngle, theta2 = startAngle+math.pi, color = 'green')
                        self.conf.add_patch(arc)
                    elif self.counterclockwise_var.get():
                        center_x = 2 * mid_x - center_x
                        center_y = 2 * mid_y - center_y
                        startAngle = math.atan2(y1- center_y, x1 - center_x) *180 / math.pi
                        endAngle   = math.atan2(y2- center_y, x2 - center_x) *180 / math.pi 
                        arc = patches.Arc((center_x, center_y), width= radius*2, height = radius*2 , angle = 0, theta1 = startAngle, theta2 = endAngle, color = 'green')
                        self.conf.add_patch(arc)

        self.canvas.draw()

    def draw_robot(self):
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
                self.conf.add_patch(robot_shape)
                tri_side = 0.3
                triangle_points = [
                    (x + tri_side * np.cos(yaw), y + tri_side * np.sin(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) + (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                     y - tri_side / 2 * np.sin(yaw) - (tri_side * np.sqrt(3) / 2) * np.cos(yaw)),
                    (x - tri_side / 2 * np.cos(yaw) - (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                     y - tri_side / 2 * np.sin(yaw) + (tri_side * np.sqrt(3) / 2) * np.cos(yaw))
                ]
                direction_triangle = Polygon(triangle_points, closed=True, color='blue', alpha=0.7)
                self.conf.add_patch(direction_triangle)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def delete_line(self):
        if self.selected_line is not None:
            del self.lines[self.selected_line]
            self.selected_line = None
            self.display_map()
            self.canvas.draw()

    def on_pick(self, event):
        if self.lines:
            artist = event.artist
            for i, (start, end) in enumerate(self.lines):
                x_values = [start[0], end[0]]
                y_values = [start[1], end[1]]
                line = self.conf.plot(x_values, y_values, 'r-', picker=True, label=f"Line {i}")
                self.conf.plot(x_values, y_values, 'ro', picker=True, markersize=2)
                if artist == line:
                    self.selected_line = i
                    break

    def calculate_angle_to_line(self, target_x, target_y, robot_x, robot_y, robot_yaw):
        angle_to_target = math.atan2(target_y - robot_y, target_x - robot_x)
        angle_diff = angle_to_target - robot_yaw
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        return angle_diff

    def send_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_publisher.publish(twist)

    # def move_along_path(self):
    #     try:
    #         self.linear_speed = float(self.entry_linear_speed.get())
    #         self.angular_speed = float(self.entry_angular_speed.get())
    #     except ValueError:
    #         rospy.logerr("Invalid input for speed.")
    #         return

    #     while self.lines and not self.stop_moving:
    #         line_start, line_end = self.lines[0]
    #         target_x, target_y = line_end[0], line_end[1]

    #         try:
    #             (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    #             robot_x, robot_y = trans[0], trans[1]
    #             robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
    #             angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)
    #             while abs(angle_error) >= (0.017 / 2) and not self.stop_moving:
    #                 angular_vel = self.angular_speed if angle_error > 0 else -self.angular_speed
    #                 self.send_velocity(0, angular_vel)
    #                 rospy.sleep(0.01)
    #                 try:
    #                     (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    #                     robot_x, robot_y = trans[0], trans[1]
    #                     robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
    #                 except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #                     rospy.logerr(f"TF error: {e}")
    #                     self.send_velocity(0, 0)
    #                     return
    #                 angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)

    #             self.send_velocity(0, 0)
    #             rospy.sleep(0.1)
    #             distance_error = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
    #             while distance_error > 0.1 and not self.stop_moving:
    #                 linear_vel = self.linear_speed
    #                 self.send_velocity(linear_vel, 0)
    #                 rospy.sleep(0.02)
    #                 angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)
    #                 if abs(angle_error) > (0.017 / 10):
    #                     angular_vel = self.angular_speed if angle_error > 0 else -self.angular_speed
    #                     self.send_velocity(0, angular_vel)
    #                     rospy.sleep(0.02)
    #                 try:
    #                     (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    #                     robot_x, robot_y = trans[0], trans[1]
    #                     robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
    #                 except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #                     rospy.logerr(f"TF error: {e}")
    #                     self.send_velocity(0, 0)
    #                     return
    #                 distance_error = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
    #                 angle_error = self.calculate_angle_to_line(target_x, target_y, robot_x, robot_y, robot_yaw)
    #             self.send_velocity(0, 0)
    #             del self.lines[0]
    #             self.master.after(0, self.display_map)
    #         except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #             rospy.logerr(f"Error during movement: {e}")
    #             self.send_velocity(0, 0)
    #             return
    #     rospy.loginfo("Finished moving along all paths.")
    #     self.send_velocity(0, 0)

    def move_along_path(self):
        try:
            self.linear_speed = float(self.entry_linear_speed.get())
            self.angular_speed = float(self.entry_angular_speed.get())
        except ValueError:
            rospy.logerr("Invalid input for speed.")
            return

        while self.lines and not self.stop_moving:
            line_start, line_end = self.lines[0]
            target_x, target_y = line_end[0], line_end[1]

            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                robot_x, robot_y = trans[0], trans[1]
                robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
                desired_angle = math.atan2(target_y - robot_y, target_x - robot_x)
                angle_error = desired_angle - robot_yaw
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
                while abs(angle_error) > math.radians(0.001) and not self.stop_moving: #0.001
                    angular_vel = (abs(angle_error) / (math.pi/2)) * self.angular_speed if angle_error > 0 else -(abs(angle_error) / (math.pi/2)) * self.angular_speed
                    self.send_velocity(0, angular_vel)
                    rospy.sleep(0.01)
                    try:
                        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                        robot_x, robot_y = trans[0], trans[1]
                        robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
                    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logerr(f"TF error: {e}")
                        self.send_velocity(0, 0)
                        return
                    desired_angle = math.atan2(target_y - robot_y, target_x - robot_x)
                    angle_error = desired_angle - robot_yaw
                    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi 
                self.send_velocity(0, 0)
                rospy.sleep(0.1)
                distance_error = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
                while distance_error > 0.1 and not self.stop_moving:
                    linear_vel = self.linear_speed
                    self.send_velocity(linear_vel, 0) 
                    rospy.sleep(0.02)
                    try:
                        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                        robot_x, robot_y = trans[0], trans[1]
                    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logerr(f"TF error: {e}")
                        self.send_velocity(0, 0)
                        return
                    distance_error = math.sqrt((target_x - robot_x) ** 2 + (target_y - robot_y) ** 2)
                self.send_velocity(0, 0)
                del self.lines[0]
                self.master.after(0, self.display_map)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"Error during movement: {e}")
                self.send_velocity(0, 0)
                return

            rospy.loginfo("Finished moving along all paths.")
            self.send_velocity(0, 0)

    def move_along_path_thread(self):
        threading.Thread(target=self.move_along_path).start()

    def move_along_arc(self):
        try:
            if not self.circles:
                rospy.logwarn("No arc to follow.")
                return
            circle_start, circle_end, radius = self.circles[0]
            x1, y1 = circle_start
            x2, y2 = circle_end
            R = radius

            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2
            d = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            h = math.sqrt(R ** 2 - (d / 2) ** 2)
            pt_angle = math.atan2((y2 - y1), (x2 - x1))
            center_x = mid_x - h * math.sin(pt_angle)
            center_y = mid_y + h * math.cos(pt_angle)

            if float(self.entry_angle_circle.get()):
                if self.clockwise_var.get():
                    center_x = mid_x - h * math.sin(pt_angle)
                    center_y = mid_y + h * math.cos(pt_angle)
                elif self.counterclockwise_var.get():
                    center_x = 2 * mid_x - center_x
                    center_y = 2 * mid_y - center_y
            else:
                if self.counterclockwise_var.get():
                    center_x = mid_x - h * math.sin(pt_angle)
                    center_y = mid_y + h * math.cos(pt_angle)
                elif self.clockwise_var.get():
                    center_x = 2 * mid_x - center_x
                    center_y = 2 * mid_y - center_y

            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
            angle_to_center = math.atan2(center_y - robot_y, center_x - robot_x)
            tangent_angle = angle_to_center + math.pi / 2 
            if self.clockwise_var.get():
                angle_error = (tangent_angle - robot_yaw + math.pi) % (2 * math.pi) - math.pi
            elif self.counterclockwise_var.get():
                angle_error = (tangent_angle - robot_yaw) % (2 * math.pi) - math.pi

            while abs(angle_error) >= (0.01) and not self.stop_moving:
                angular_vel = (abs(angle_error) / (math.pi/2)) * self.angular_speed if angle_error > 0 else -(abs(angle_error) / (math.pi/2)) * self.angular_speed
                self.send_velocity(0, angular_vel)
                rospy.sleep(0.01)
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    robot_x, robot_y = trans[0], trans[1]
                    robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF error: {e}")
                    self.send_velocity(0, 0)
                    return
                angle_to_center = math.atan2(center_y - robot_y, center_x - robot_x)
                tangent_angle = angle_to_center + math.pi / 2
                if self.clockwise_var.get():
                    angle_error = (tangent_angle - robot_yaw + math.pi) % (2 * math.pi) - math.pi
                elif self.counterclockwise_var.get():
                    angle_error = (tangent_angle - robot_yaw) % (2 * math.pi) - math.pi
            self.send_velocity(0, 0)
            rospy.sleep(0.1)
            while not self.stop_moving:
                linear_vel = float(self.entry_linear_speed.get())
                angular_vel = linear_vel / R
                if float(self.entry_angle_circle.get()):
                    if self.clockwise_var.get():
                        self.send_velocity(linear_vel, -angular_vel)
                    elif self.counterclockwise_var.get():
                        self.send_velocity(linear_vel, angular_vel)
                else:
                    if self.clockwise_var.get():
                        self.send_velocity(linear_vel, -angular_vel)
                    elif self.counterclockwise_var.get():
                        self.send_velocity(linear_vel, angular_vel)
                rospy.sleep(0.02)
                distance_to_end = math.sqrt((x2 - robot_x) ** 2 + (y2 - robot_y) ** 2)
                if distance_to_end < self.distance_tolerance:
                    break
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    robot_x, robot_y = trans[0], trans[1]
                    robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
                except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr(f"TF error: {e}")
                    self.send_velocity(0, 0)
                    return
            self.send_velocity(0, 0)
            rospy.loginfo("Finished moving along the arc.")
        except Exception as e:
            rospy.logerr(f"Error in move_along_arc: {e}")
            self.send_velocity(0, 0)

    def move_along_arc_thread(self):
        threading.Thread(target=self.move_along_arc).start()   

    def stop_movement(self):
        self.stop_moving = True
        self.send_velocity(0, 0)
        rospy.loginfo("Movement stopped.")
        self.teleop_control.stop()

    def update_robot_position_label(self, x, y):
        self.position_label.config(text=f"Robot Position: ({x:.2f}, {y:.2f})")

    def update_position_continuously(self):
        threading.Thread(target=self.update_position).start()

    def update_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_position = (trans[0], trans[1])
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            self.robot_orientation = yaw
            self.position_label.config(text=f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
            self.display_map()
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            pass
        finally:
            self.master.after(50, self.update_position)     
            
    def update_lidar_points(self):
        if self.conf is None:
            return
        try:
            if not self.f_points and not self.b_points:
                self.lidar_points = []
            else:
                self.lidar_points = self.f_points + self.b_points

            if not self.lidar_points:
                if self.lidar_points_plot is not None:
                    self.lidar_points_plot.set_data([], []) 
                    self.canvas.restore_region(self.background)
                    self.conf.draw_artist(self.lidar_points_plot)
                    self.canvas.blit(self.conf.bbox)
                return  
            
            x_vals, y_vals = zip(*self.lidar_points)
            if self.lidar_points_plot is None:
                self.lidar_points_plot, = self.conf.plot(x_vals, y_vals, 'r.', markersize=2)  # Store plot object
                self.canvas.draw()  
            else:
                self.lidar_points_plot.set_data(x_vals, y_vals)
                self.canvas.restore_region(self.background)
                self.conf.draw_artist(self.lidar_points_plot)
                self.canvas.blit(self.conf.bbox)
        except Exception as e:
            rospy.logwarn(f"Lidar exception: {e}")

    def update_lidar_points_continuously(self):
        rospy.Timer(rospy.Duration(0.05), self.update_lidar_points)
        threading.Thread(target=self.update_lidar_points).start()

    def update_teleop_speed(self):
        try:
            self.linear_speed = float(self.entry_linear_speed.get())
            self.angular_speed = float(self.entry_angular_speed.get())
            self.teleop_control.linear_speed = self.linear_speed
            self.teleop_control.angular_speed = self.angular_speed
        except ValueError:
            rospy.logerr("Invalid input for speed.")


if __name__ == "__main__":
    rospy.init_node('map_display_gui', anonymous=True)
    root = tk.Tk()
    root.title("Map Display")

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)

    app = MapDisplay(root)
    app.canvas.mpl_connect('button_press_event', app.on_click)
    app.canvas.mpl_connect('motion_notify_event', app.on_drag)
    app.canvas.mpl_connect('button_release_event', app.on_release)
    app.canvas.mpl_connect('pick_event', app.on_pick)

    root.mainloop()