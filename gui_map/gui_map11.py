#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
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
        self.is_moving = False
        self.drag_start = None
        self.path_points = []

        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)

        self.tf_listener = tf.TransformListener()

        self.fig = Figure(figsize=(5, 5))
        self.conf = self.fig.add_subplot(111)
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
        button_color = "green" if self.nav_goal_mode else "lightgray"
        self.nav_goal_button.config(bg=button_color)

    def on_click(self, event):
        if event.inaxes == self.conf and self.nav_goal_mode:
            self.drag_start = (event.xdata, event.ydata)

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.conf:
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

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.conf and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata
            z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

            self.entry_z.delete(0, tk.END)
            self.entry_z.insert(0, str(z))
            self.entry_w.delete(0, tk.END)
            self.entry_w.insert(0, str(w))
            self.send_goal_thread()
        self.drag_start = None

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

        self.movebase_client(x, y, z, w)

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
            self.conf.plot(self.robot_position[0], self.robot_position[1], 'ro', label='Robot Position')
            if self.goal_position:
                self.conf.plot(self.goal_position[0], self.goal_position[1], 'go', label='Goal Position')

            if self.path_points:
                path_x, path_y = zip(*self.path_points)
                self.conf.plot(path_x, path_y, 'b--', label='Path')

            self.canvas.draw()

    def update_position_continuously(self):
        threading.Thread(target=self.update_position, daemon=True).start()

    def update_position(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.robot_position = (trans[0], trans[1])
                self.position_label.config(text=f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
                self.display_map()
                rospy.sleep(0.1)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue

if __name__ == "__main__":
    rospy.init_node('map_display_gui', anonymous=True)
    root = tk.Tk()
    root.title("Map Display")
    app = MapDisplay(root)
    root.mainloop()
