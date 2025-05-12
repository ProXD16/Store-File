#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
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
        self.current_goal_position = None  # Biến để lưu tọa độ hiện tại khi kéo
        self.dragging = False  # Biến để kiểm tra xem có đang kéo hay không

        # Khai báo subscriber cho bản đồ
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Tạo TF listener để lấy vị trí của robot
        self.tf_listener = tf.TransformListener()

        # Tạo không gian hiển thị bản đồ
        self.fig = Figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)

        self.create_widgets()
        self.update_position_continuously()  # Bắt đầu cập nhật vị trí robot

        # Thiết lập sự kiện cho nhấp chuột và kéo
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)

    def create_widgets(self):
        tk.Label(self.master, text="X:").grid(row=0, column=0)
        self.entry_x = tk.Entry(self.master)
        self.entry_x.grid(row=0, column=1)

        tk.Label(self.master, text="Y:").grid(row=1, column=0)
        self.entry_y = tk.Entry(self.master)
        self.entry_y.grid(row=1, column=1)

        tk.Label(self.master, text="Z:").grid(row=2, column=0)
        self.entry_z = tk.Entry(self.master)
        self.entry_z.insert(0, "0.0")  # Giá trị mặc định cho Z
        self.entry_z.grid(row=2, column=1)

        tk.Label(self.master, text="W:").grid(row=3, column=0)
        self.entry_w = tk.Entry(self.master)
        self.entry_w.insert(0, "1.0")  # Giá trị mặc định cho W
        self.entry_w.grid(row=3, column=1)

        send_button = tk.Button(self.master, text="Send Goal", command=self.send_goal_thread)
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=6, column=0, columnspan=2)

    def send_goal_thread(self):
        # Khởi động một luồng mới để gửi mục tiêu
        threading.Thread(target=self.send_goal).start()

    def calculate_goal_position(self):
        if self.current_goal_position is not None:
            robot_x, robot_y = self.robot_position
            goal_x, goal_y = self.current_goal_position

            # Tính toán khoảng cách và góc từ vị trí robot đến mục tiêu
            dx = goal_x - robot_x
            dy = goal_y - robot_y
            distance = np.sqrt(dx**2 + dy**2)

            # Tính toán góc
            theta = np.arctan2(dy, dx)  # Góc theo radian
            z = np.sin(theta / 2)
            w = np.cos(theta / 2)

            return goal_x, goal_y, z, w  # Trả về tọa độ và hướng
        return None

    def send_goal(self):
        goal_position = self.calculate_goal_position()
        if goal_position:
            x, y, z, w = goal_position
            self.movebase_client(x, y, z, w)

    def on_click(self, event):
        # Kiểm tra xem chuột có nhấp vào bản đồ không
        if event.inaxes is not None:
            self.dragging = True
            # Lưu vị trí hiện tại của mục tiêu
            self.current_goal_position = (event.xdata, event.ydata)
            self.update_goal_entry()  # Cập nhật giá trị tọa độ vào ô nhập

    def on_release(self, event):
        # Khi nhả chuột, ngừng kéo
        if self.dragging:
            self.dragging = False
            self.current_goal_position = None  # Đặt lại vị trí mục tiêu

    def on_drag(self, event):
        # Cập nhật vị trí khi kéo chuột
        if self.dragging and event.inaxes is not None:
            self.current_goal_position = (event.xdata, event.ydata)
            self.update_goal_entry()  # Cập nhật giá trị tọa độ vào ô nhập

    def update_goal_entry(self):
        if self.current_goal_position is not None:
            x, y = self.current_goal_position
            self.entry_x.delete(0, tk.END)
            self.entry_x.insert(0, f"{x:.2f}")
            self.entry_y.delete(0, tk.END)
            self.entry_y.insert(0, f"{y:.2f}")

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
        client.wait_for_result()

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
            self.ax.plot(self.robot_position[0], self.robot_position[1], 'ro')  # Hiển thị vị trí robot

            # Hiển thị điểm đích nếu có
            if self.current_goal_position is not None:
                goal_x, goal_y = self.current_goal_position
                self.ax.plot(goal_x, goal_y, 'bo', label='Goal')  # Vẽ điểm đích bằng màu xanh
                self.ax.legend()  # Hiển thị chú thích

            self.canvas.draw()

    def update_position_continuously(self):
        self.update_position()  # Cập nhật vị trí robot
        self.master.after(1000, self.update_position_continuously)  # Gọi lại sau 1 giây

    def update_position(self):
        try:
            # Lấy vị trí của robot từ tf
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            self.robot_position = trans
            self.position_label.config(text="Robot Position: ({:.2f}, {:.2f})".format(trans[0], trans[1]))
            self.display_map()  # Cập nhật bản đồ để hiển thị vị trí robot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get robot position")

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    root = tk.Tk()
    app = MapDisplay(root)

    # Chạy rospy.spin() trong luồng riêng
    def run_ros():
        rospy.spin()

    ros_thread = threading.Thread(target=run_ros)
    ros_thread.start()

    root.mainloop()
