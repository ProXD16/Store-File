import rospy
import numpy as np
import tkinter as tk
from tkinter import Canvas
from nav_msgs.msg import OccupancyGrid
import threading
import subprocess

# Biến toàn cục để theo dõi trạng thái kết nối ROS
connected = False
map_data = None
robot_position = (0, 0)
goal_position = None
path_points = []

# Hàm thực thi ROS launch
def start_roslaunch():
    global connected
    if not connected:
        try:
            rospy.loginfo("Starting roslaunch...")
            subprocess.Popen(["roslaunch", "mir_driver", "mir.launch", "mir_hostname:=192.168.0.172"])
            rospy.loginfo("ROS launch command executed.")
            connected = True
        except Exception as e:
            rospy.logerr(f"Error starting roslaunch: {e}")

# Hàm callback nhận dữ liệu bản đồ
def map_callback(msg):
    global map_data
    rospy.loginfo("Received map data")
    map_data = msg
    display_map()  # Vẽ bản đồ ngay khi nhận được dữ liệu

# Hàm đăng ký subscriber và bắt đầu nhận dữ liệu từ ROS
def subscribe_to_map():
    rospy.Subscriber("/map", OccupancyGrid, map_callback)  # Subscribe vào topic /map

# Hàm vẽ bản đồ lên canvas
def display_map():
    global map_data, robot_position, goal_position, path_points
    if map_data is not None:
        try:
            # Chuyển đổi dữ liệu map thành mảng numpy
            data_array = np.array(map_data.data).reshape((map_data.info.height, map_data.info.width))
            data_array = np.where(data_array == -1, np.nan, data_array)  # Thay -1 bằng NaN (unknown)

            # Lấy kích thước bản đồ và thiết lập độ phân giải
            width = map_data.info.width
            height = map_data.info.height
            resolution = map_data.info.resolution
            origin_x = map_data.info.origin.position.x
            origin_y = map_data.info.origin.position.y

            # Xóa canvas và vẽ lại bản đồ
            canvas.delete("all")

            # Hiển thị bản đồ
            for y in range(height):
                for x in range(width):
                    color = "black" if data_array[y, x] > 0 else "white"
                    canvas.create_rectangle(
                        x * resolution + origin_x,
                        y * resolution + origin_y,
                        (x + 1) * resolution + origin_x,
                        (y + 1) * resolution + origin_y,
                        fill=color, outline=color
                    )

            # Vẽ vị trí của robot
            canvas.create_oval(robot_position[0] - 5, robot_position[1] - 5,
                               robot_position[0] + 5, robot_position[1] + 5,
                               fill="red", outline="red")

            # Vẽ vị trí goal nếu có
            if goal_position:
                canvas.create_oval(goal_position[0] - 5, goal_position[1] - 5,
                                   goal_position[0] + 5, goal_position[1] + 5,
                                   fill="green", outline="green")

            # Vẽ đường đi của robot nếu có
            if path_points:
                for i in range(len(path_points) - 1):
                    canvas.create_line(path_points[i][0], path_points[i][1],
                                        path_points[i + 1][0], path_points[i + 1][1],
                                        fill="blue", dash=(4, 2))
        except Exception as e:
            rospy.logerr(f"Error processing map data: {e}")

# Hàm để cập nhật vị trí robot
def update_robot_position():
    global robot_position
    robot_position = (100, 100)  # Ví dụ: Cập nhật vị trí robot
    display_map()

# Hàm để cập nhật vị trí goal
def update_goal_position():
    global goal_position
    goal_position = (200, 200)  # Ví dụ: Cập nhật vị trí goal
    display_map()

# Hàm chính để chạy giao diện Tkinter
def main():
    global canvas
    root = tk.Tk()
    root.title("Map Display")
    root.geometry("800x800")

    # Tạo frame chứa canvas và các nút
    frame = tk.Frame(root)
    frame.pack(fill=tk.BOTH, expand=True)

    # Tạo canvas để vẽ bản đồ
    canvas = Canvas(frame, width=600, height=600)
    canvas.pack(side=tk.LEFT, padx=10, pady=10)

    # Tạo panel chứa các nút
    control_panel = tk.Frame(frame)
    control_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)

    # Nút bấm Connect để bắt đầu ROS launch
    connect_button = tk.Button(control_panel, text="Connect", command=start_roslaunch)
    connect_button.pack(pady=10)

    # Nút bấm để cập nhật vị trí robot và goal
    robot_button = tk.Button(control_panel, text="Update Robot Position", command=update_robot_position)
    robot_button.pack(pady=10)

    goal_button = tk.Button(control_panel, text="Update Goal Position", command=update_goal_position)
    goal_button.pack(pady=10)

    # Khởi tạo ROS và bắt đầu nhận dữ liệu từ topic map
    rospy.init_node('map_display_gui', anonymous=True)  # Khởi tạo ROS node ngay từ đầu
    subscribe_to_map()  # Đăng ký subscriber vào topic /map

    # Chạy giao diện Tkinter
    root.mainloop()

if __name__ == "__main__":
    main()
