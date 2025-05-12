#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String  # Đảm bảo bạn import đúng kiểu dữ liệu của status message
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np

class MapDisplay:
    def __init__(self, master):
        self.master = master
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        
        # Khai báo subscriber cho bản đồ và trạng thái
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.status_subscriber = rospy.Subscriber("/mir_status_msg", String, self.status_callback)

        # Tạo không gian hiển thị bản đồ
        self.fig = Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)

        self.create_widgets()

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

        send_button = tk.Button(self.master, text="Send Goal", command=self.send_goal)
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        # Thêm label để hiển thị trạng thái robot
        self.status_label = tk.Label(self.master, text="Robot Status: Unknown")
        self.status_label.grid(row=6, column=0, columnspan=2)

    def send_goal(self):
        x = float(self.entry_x.get())
        y = float(self.entry_y.get())
        z = float(self.entry_z.get())
        w = float(self.entry_w.get())
        self.movebase_client(x, y, z, w)

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
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()

    def map_callback(self, data):
        self.map_data = data
        self.resolution = data.info.resolution
        self.origin = (data.info.origin.position.x, data.info.origin.position.y)
        self.display_map()

    def display_map(self):
        if self.map_data is not None:
            # Chuyển đổi dữ liệu bản đồ thành định dạng hình ảnh
            data_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
            data_array = np.where(data_array == -1, np.nan, data_array)  # Đặt -1 thành NaN để không hiển thị
            self.ax.clear()
            self.ax.imshow(data_array, cmap='gray', origin='lower', extent=[
                self.origin[0], self.origin[0] + self.map_data.info.width * self.resolution,
                self.origin[1], self.origin[1] + self.map_data.info.height * self.resolution
            ])
            self.ax.set_title('Map')
            self.canvas.draw()

    def status_callback(self, msg):
        # Cập nhật trạng thái robot vào label
        self.status_label.config(text="Robot Status: " + msg.data)

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    root = tk.Tk()
    app = MapDisplay(root)
    root.mainloop()
