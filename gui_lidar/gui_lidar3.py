#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tf
import threading

class MapDisplay:
    def __init__(self, master):
        self.master = master
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        self.robot_position = (0.0, 0.0)
        self.goal_position = None
        self.f_points = []
        self.b_points = []
        
        # Initialize the tf listener
        self.listener = tf.TransformListener()

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)

        self.fig = Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)

        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)

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
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        self.nav_goal_button = tk.Button(self.master, text="2D Nav Goal", command=self.toggle_nav_goal_mode)
        self.nav_goal_button.grid(row=4, column=1)

        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=6, column=0, columnspan=2)

        self.nav_goal_mode = False 

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        self.nav_goal_button.config(bg="green" if self.nav_goal_mode else "lightgray")

    def on_click(self, event):
        if event.inaxes == self.ax and self.nav_goal_mode:
            self.drag_start = (event.xdata, event.ydata)

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.ax:
            x_start, y_start = self.drag_start
            self.entry_x.delete(0, tk.END)
            self.entry_x.insert(0, str(x_start))
            self.entry_y.delete(0, tk.END)
            self.entry_y.insert(0, str(y_start))

            self.draw_map_and_lidar()
            self.ax.annotate('', xy=(event.xdata, event.ydata), xytext=(x_start, y_start),
                             arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
            self.canvas.draw()

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata
            z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

            self.entry_z.delete(0, tk.END)
            self.entry_z.insert(0, str(z))
            self.entry_w.delete(0, tk.END)
            self.send_goal_thread()
        self.drag_start = None

    def calculate_orientation(self, start, end):
        orientation = np.arctan2(end[1] - start[1], end[0] - start[0])
        return np.sin(orientation / 2), np.cos(orientation / 2)

    def send_goal_thread(self):
        threading.Thread(target=self.send_goal).start()

    def send_goal(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            w = float(self.entry_w.get())
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

        client.send_goal(goal)
        self.nav_goal_button.config(bg="lightgray")
        client.wait_for_result()

    def rviz_goal_callback(self, goal_msg):
        self.goal_position = (goal_msg.pose.position.x, goal_msg.pose.position.y)
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, str(self.goal_position[0]))
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, str(self.goal_position[1]))
        self.movebase_client(goal_msg.pose.position.x, goal_msg.pose.position.y,
                             goal_msg.pose.orientation.z, goal_msg.pose.orientation.w)

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
                    point_map = self.listener.transformPoint("map", point_lidar)
                    points.append((point_map.point.x, point_map.point.y))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

        return points

    def path_callback(self, path_msg):
        self.draw_map_and_lidar()

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
            self.ax.plot(self.robot_position[0], self.robot_position[1], "ro")

        if self.f_points:
            x_vals, y_vals = zip(*self.f_points)
            self.ax.plot(x_vals, y_vals, "b.")

        if self.b_points:
            x_vals, y_vals = zip(*self.b_points)
            self.ax.plot(x_vals, y_vals, "g.")

        self.canvas.draw()

    def update_position_continuously(self):
        try:
            self.listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, _) = self.listener.lookupTransform("map", "base_link", rospy.Time(0))
            self.robot_position = (trans[0], trans[1])
            self.position_label.config(text="Robot Position: ({:.2f}, {:.2f})".format(*self.robot_position))
            self.draw_map_and_lidar()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.master.after(100, self.update_position_continuously)

if __name__ == "__main__":
    rospy.init_node("map_display")
    root = tk.Tk()
    app = MapDisplay(root)
    root.mainloop()
