#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tf
import threading
from matplotlib.patches import Polygon, Arc
import matplotlib.pyplot as plt
import math
import time
from matplotlib.widgets import TextBox
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
        self.circles = []
        self.selected_line = None
        self.drawing_line = False
        self.temp_line_start = None
        self.temp_line_end = None
        self.drawing_circle = False
        self.temp_circle_start = None
        self.temp_circle_end = None
        self.stop_moving = False
        self.distance_tolerance = 0.1
        self.segment_length = 0.2
        self.radius_circle = 1.0
        self.arc_patch = None 

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)

        self.tf_listener = tf.TransformListener()
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.fig = Figure(figsize=(6, 6))
        self.conf = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)

        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect('pick_event', self.on_pick)

        self.create_widgets()
        self.update_position_continuously()

    def create_widgets(self):
        tk.Label(self.master, text="X:").grid(row=0, column=0)
        self.entry_x = tk.Entry(self.master)
        self.entry_x.grid(row=0, column=1)

        tk.Label(self.master, text="Y:").grid(row=1, column=0)
        self.entry_y = tk.Entry(self.master)
        self.entry_y.grid(row=1, column=1)

        tk.Label(self.master, text="Z:").grid(row=2, column=0)
        self.entry_z = tk.Entry(self.master)
        self.entry_z.grid(row=2, column=1)

        tk.Label(self.master, text="W:").grid(row=3, column=0)
        self.entry_w = tk.Entry(self.master)
        self.entry_w.grid(row=3, column=1)

        send_button = tk.Button(self.master, text="Send Goal", command=self.send_goal_thread)
        send_button.grid(row=4, column=0, pady=10)

        self.nav_goal_button = tk.Button(self.master, text="2D Nav Goal", command=self.toggle_nav_goal_mode)
        self.nav_goal_button.grid(row=4, column=1)

        self.draw_line_button = tk.Button(self.master, text="Draw Line", command=self.toggle_draw_line_mode)
        self.draw_line_button.grid(row=7, column=0)

        self.delete_line_button = tk.Button(self.master, text="Delete Line", command=self.delete_line)
        self.delete_line_button.grid(row=7, column=2)

        tk.Label(self.master, text="Linear Speed:").grid(row=8, column=0)
        self.entry_linear_speed = tk.Entry(self.master)
        self.entry_linear_speed.insert(0, str(self.linear_speed))
        self.entry_linear_speed.grid(row=8, column=1)

        tk.Label(self.master, text="Angular Speed:").grid(row=9, column=0)
        self.entry_angular_speed = tk.Entry(self.master)
        self.entry_angular_speed.insert(0, str(self.angular_speed))
        self.entry_angular_speed.grid(row=9, column=1)

        move_button = tk.Button(self.master, text="Run", command=self.move_along_path_thread)
        move_button.grid(row=11, column=0, columnspan=2, pady=10)

        tk.Label(self.master, text="Radius Circle:").grid(row=10, column=0)
        self.entry_radius_circle = tk.Entry(self.master)
        self.entry_radius_circle.insert(0, str(self.radius_circle))
        self.entry_radius_circle.grid(row=10, column=1)

        self.draw_circle_button = tk.Button(self.master, text="Draw Circle", command=self.toggle_draw_circle_mode)
        self.draw_circle_button.grid(row=7, column=1, pady=10)

        stop_button = tk.Button(self.master, text="Stop", command=self.stop_movement)
        stop_button.grid(row=12, column=0, columnspan=2, pady=10)

        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=6, column=0, columnspan=2)

        self.nav_goal_mode = False
        self.draw_line_mode = False
        self.draw_circle_mode = False

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        button_color = "green" if self.nav_goal_mode else "lightgray"
        self.nav_goal_button.config(bg=button_color)
        self.draw_line_mode = False
        self.draw_circle_mode = False

    def toggle_draw_line_mode(self):
        self.draw_line_mode = not self.draw_line_mode
        button_color = "green" if self.draw_line_mode else "lightgray"
        self.draw_line_button.config(bg=button_color)
        self.nav_goal_mode = False
        self.draw_circle_mode = False

    def toggle_draw_circle_mode(self):
        self.draw_circle_mode = not self.draw_circle_mode
        print(f"Draw circle mode: {self.drawing_circle}")  
        button_color = "green" if self.draw_circle_mode else "lightgray"
        self.draw_circle_mode = True
        self.draw_circle_button.config(bg=button_color)
        self.nav_goal_mode = False 
        self.draw_line_mode = False

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
                    (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    x, y = trans[0], trans[1]
                    end_x = event.xdata
                    end_y = event.ydata
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
                x_values = [start[0], end[0]]
                y_values = [start[1], end[1]]
                line = self.conf.plot(x_values, y_values, 'r-', picker=True, label=f"Line {i}")
                self.conf.plot(x_values, y_values, 'ro', picker=True, markersize=2)

            if self.drawing_line and self.temp_line_start and self.temp_line_end:
                x_values = [self.temp_line_start[0], self.temp_line_end[0]]
                y_values = [self.temp_line_start[1], self.temp_line_end[1]]
                self.conf.plot(x_values, y_values, 'r--')
                
            for circle_start, circle_end, circle_radius in self.circles: 
                x1, y1 = circle_start
                x2, y2 = circle_end
                radius = circle_radius 
                mid_x = (x1 + x2)/2
                mid_y = (y1+y2)/2
                d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2) 
                h = math.sqrt(radius**2 - (d/2)**2) 
                pt_angle = math.atan2((y2 - y1),(x2-x1)) 
                center_x = mid_x - h * math.sin(pt_angle) 
                center_y = mid_y + h * math.cos(pt_angle) 
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
                while abs(angle_error) > math.radians(0.01) and not self.stop_moving:
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

    def stop_movement(self):
        self.stop_moving = True
        self.send_velocity(0, 0)
        rospy.loginfo("Movement stopped.")

    def update_robot_position_label(self, x, y):
        self.position_label.config(text=f"Robot Position: ({x:.2f}, {y:.2f})")

    def update_position_continuously(self):
        threading.Thread(target=self.update_position, daemon=True).start()

    def update_position(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.robot_position = (trans[0], trans[1])
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                self.robot_orientation = yaw
                self.position_label.config(text=f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
                self.display_map()
                rospy.sleep(0.05)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue
        self.is_moving = False 

if __name__ == "__main__":
    rospy.init_node('map_display_gui', anonymous=True)
    root = tk.Tk()
    root.title("Map Display")
    app = MapDisplay(root)
    app.canvas.mpl_connect('pick_event', app.on_pick)
    root.mainloop()