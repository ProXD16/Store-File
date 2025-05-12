#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk

# Hàm điều khiển robot
def send_velocity(linear_x, angular_z):
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z
    pub.publish(vel_msg)

# Hàm cho từng chế độ di chuyển
def move_straight():
    speed = float(speed_entry.get())
    send_velocity(speed, 0.0)

def move_circle():
    linear_speed = float(speed_entry.get())
    angular_speed = float(angular_entry.get())
    send_velocity(linear_speed, angular_speed)

def move_arc():
    linear_speed = float(speed_entry.get())
    radius = float(radius_entry.get())
    if radius != 0:
        angular_speed = linear_speed / radius
        send_velocity(linear_speed, angular_speed)
    else:
        rospy.logwarn("Bán kính không thể bằng 0!")

def stop_robot():
    send_velocity(0.0, 0.0)

# ROS Node
rospy.init_node('mir_control_gui', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Tkinter GUI
root = tk.Tk()
root.title("Điều khiển Robot MiR")

# Nhãn và ô nhập cho tốc độ
tk.Label(root, text="Tốc độ tuyến tính (m/s):").grid(row=0, column=0, padx=10, pady=10)
speed_entry = tk.Entry(root)
speed_entry.grid(row=0, column=1, padx=10, pady=10)
speed_entry.insert(0, "0.5")  # Giá trị mặc định

tk.Label(root, text="Tốc độ góc (rad/s):").grid(row=1, column=0, padx=10, pady=10)
angular_entry = tk.Entry(root)
angular_entry.grid(row=1, column=1, padx=10, pady=10)
angular_entry.insert(0, "0.2")  # Giá trị mặc định

tk.Label(root, text="Bán kính cung tròn (m):").grid(row=2, column=0, padx=10, pady=10)
radius_entry = tk.Entry(root)
radius_entry.grid(row=2, column=1, padx=10, pady=10)
radius_entry.insert(0, "1.0")  # Giá trị mặc định

# Các nút điều khiển
ttk.Button(root, text="Đi thẳng", command=move_straight).grid(row=3, column=0, padx=10, pady=10)
ttk.Button(root, text="Đi đường tròn", command=move_circle).grid(row=3, column=1, padx=10, pady=10)
ttk.Button(root, text="Đi cung tròn", command=move_arc).grid(row=4, column=0, padx=10, pady=10)
ttk.Button(root, text="Dừng", command=stop_robot).grid(row=4, column=1, padx=10, pady=10)

# Chạy giao diện
root.mainloop()
