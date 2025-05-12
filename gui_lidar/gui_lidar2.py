import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
import numpy as np
import tkinter as tk
import tf

# Biến toàn cục
robot_x = 0
robot_y = 0
robot_theta = 0  # Góc yaw của robot
f_points = []
b_points = []
map_data = None
map_width = 0
map_height = 0
resolution = 0.05  # Độ phân giải của bản đồ (mỗi pixel tương ứng với 5cm)

# Kích thước vị trí cảm biến Lidar (điều chỉnh theo thực tế)
dx_f, dy_f = 0.445, 0  # Vị trí cảm biến trước
dx_b, dy_b = -0.445, 0.29  # Vị trí cảm biến sau

# Khởi tạo transform listener
listener = None

def pose_callback(msg):
    global robot_x, robot_y, robot_theta
    # Lấy tọa độ x, y từ Pose
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    # Lấy góc yaw từ quaternion
    orientation = msg.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    robot_theta = euler[2]  # Yaw (góc quay quanh trục z)

def f_scan_callback(msg):
    global f_points
    f_points = process_lidar_data(msg, False)

def b_scan_callback(msg):
    global b_points
    b_points = process_lidar_data(msg, True)

def process_lidar_data(msg, is_back):
    points = []
    msg.header.frame_id = 'front_laser_link' if not is_back else 'back_laser_link'
    for i in range(len(msg.ranges)):
        r = msg.ranges[i]
        angle = i * msg.angle_increment + msg.angle_min  # Tính toán góc tương ứng cho mỗi điểm
        if r < msg.range_max and r > msg.range_min:
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            
            # Tạo PointStamped để chuyển đổi
            point_lidar = PointStamped()
            point_lidar.header.frame_id = msg.header.frame_id
            point_lidar.header.stamp = rospy.Time(0)
            point_lidar.point.x = x
            point_lidar.point.y = y

            try:
                # Chuyển đổi điểm sang frame "map"
                point_map = listener.transformPoint("map", point_lidar)
                points.append((point_map.point.x, point_map.point.y))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue  # Bỏ qua điểm nếu không thể chuyển đổi

    return points

def map_callback(msg):
    global map_data, map_width, map_height, resolution
    map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    map_width = msg.info.width
    map_height = msg.info.height
    resolution = msg.info.resolution  # Cập nhật độ phân giải từ bản đồ

def draw_points(canvas):
    canvas.delete("all")  # Xóa các điểm cũ

    # Vẽ bản đồ
    if map_data is not None:
        for y in range(map_height):
            for x in range(map_width):
                value = map_data[y][x]
                if value == 0:  # Không có vật cản
                    continue
                elif value == 100:  # Vật cản
                    canvas.create_rectangle(x * scale, 
                                            -y * scale,
                                            (x + 1) * scale, 
                                            -(y + 1) * scale,
                                            fill='black')

    # Vẽ các điểm từ f_scan (màu xanh)
    for x, y in f_points:
        canvas_x = (x / resolution) + (canvas.winfo_width() / 2)
        canvas_y = -(y / resolution) + (canvas.winfo_height() / 2)  # Đảo y để phù hợp với canvas
        canvas.create_oval(canvas_x - 2, canvas_y - 2, canvas_x + 2, canvas_y + 2, fill='green')

    # Vẽ các điểm từ b_scan (màu đỏ)
    for x, y in b_points:
        canvas_x = (x / resolution) + (canvas.winfo_width() / 2)
        canvas_y = -(y / resolution) + (canvas.winfo_height() / 2)  # Đảo y để phù hợp với canvas
        canvas.create_oval(canvas_x - 2, canvas_y - 2, canvas_x + 2, canvas_y + 2, fill='red')

def update_gui():
    draw_points(canvas)
    root.after(10, update_gui)  # Cập nhật GUI mỗi 100ms

# ROS node và subscriber
rospy.init_node('lidar_display')
listener = tf.TransformListener()  # Khởi tạo listener TF
rospy.Subscriber('/f_scan', LaserScan, f_scan_callback)
rospy.Subscriber('/b_scan', LaserScan, b_scan_callback)
rospy.Subscriber('/map', OccupancyGrid, map_callback)
rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)  # Lắng nghe vị trí và hướng của robot

# GUI Tkinter
root = tk.Tk()
root.title("Lidar Points Display")
canvas = tk.Canvas(root, width=800, height=800, bg='white')
canvas.pack()

scale = 1  # Tỉ lệ hiển thị
update_gui()
root.mainloop()

