import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tkinter as tk
from queue import Queue
import threading
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import tf

# To lưu dữ liệu bản đồ và điểm Lidar
map_queue = Queue()
height = 0
width = 0
resolution = 0.05
listener = None 
lidar_points = []  # Điểm Lidar

# Hàm callback để nhận dữ liệu bản đồ
def map_callback(map_data):
    global height, width, resolution
    resolution = map_data.info.resolution
    width = map_data.info.width
    height = map_data.info.height
    data = map_data.data
    map_array = np.array(data).reshape((height, width))
    map_queue.put(map_array)

# Hàm callback để nhận dữ liệu Lidar
def lidar_callback(msg):
    global lidar_points
    lidar_points = process_lidar_data(msg)  # Chuyển đổi và lấy tọa độ Lidar
    draw_map()  # Vẽ lại bản đồ và các điểm Lidar

# Hàm xử lý dữ liệu Lidar và chuyển đổi sang frame /map
def process_lidar_data(msg):
    points = []
    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min  
        if r < msg.range_max and r > msg.range_min:
            x = r * np.cos(angle)
            y = r * np.sin(angle)

            # Tạo PointStamped cho điểm Lidar
            point_lidar = PointStamped()
            point_lidar.header.frame_id = msg.header.frame_id  # frame của Lidar
            point_lidar.header.stamp = rospy.Time(0)
            point_lidar.point.x = x
            point_lidar.point.y = y

            try:
                # Chuyển đổi điểm sang frame "map"
                point_map = listener.transformPoint("map", point_lidar)
                points.append((point_map.point.x, point_map.point.y))  # Lưu tọa độ đã chuyển đổi
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 

    return points

# Hàm vẽ bản đồ và điểm Lidar
def draw_map():
    global height, width
    if not map_queue.empty():
        map_array = map_queue.get()
        canvas.delete("all")  
        
        # Vẽ bản đồ
        for y in range(height):
            for x in range(width):
                value = map_array[y][x]
                if value == 0:
                    color = 'white'  # Không có chướng ngại vật
                elif value == 100:
                    color = 'black'  # Có chướng ngại vật
                else:
                    color = 'gray'  # Không chắc chắn

                # Điều chỉnh tọa độ cho Tkinter
                canvas.create_rectangle(
                    x, height - y - 1, x + 1, height - y, fill=color, outline=color
                )

    # Vẽ các điểm Lidar đã chuyển đổi sang frame /map
    for x, y in lidar_points:
        # Tính tọa độ pixel từ tọa độ /map với độ phân giải
        canvas_x = (x / resolution) + (canvas.winfo_width() / 2)
        canvas_y = -(y / resolution) + (canvas.winfo_height() / 2)
        if 0 <= canvas_x < canvas.winfo_width() and 0 <= canvas_y < canvas.winfo_height():
            # Vẽ điểm Lidar với kích thước 4 pixel
            canvas.create_oval(canvas_x - 2, canvas_y - 2, canvas_x + 2, canvas_y + 2, fill='green')

# Khởi tạo GUI Tkinter
root = tk.Tk()
root.title("Map Display with Lidar")
canvas = tk.Canvas(root, width=800, height=600)
canvas.pack()

# Khởi tạo node ROS
rospy.init_node('map_listener', anonymous=True)

# Đăng ký callback cho topic /map và cảm biến Lidar
rospy.Subscriber('/map', OccupancyGrid, map_callback)
rospy.Subscriber('/lidar_scan', LaserScan, lidar_callback)  # Đăng ký callback cho Lidar

# Vòng lặp chính
def main_loop():
    global listener
    listener = tf.TransformListener() 
    while not rospy.is_shutdown():
        root.update()  

# Chạy vòng lặp chính trong một luồng riêng
threading.Thread(target=main_loop, daemon=True).start()

# Bắt đầu vẽ bản đồ
draw_map()

# Giữ cho GUI chạy
root.mainloop()
