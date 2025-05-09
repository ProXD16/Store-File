# import tkinter as tk
# from geometry_msgs.msg import Twist
# import rospy
# from tkinter import ttk

# class TeleopControl:
#     def __init__(self, master, cmd_vel_publisher, linear_speed=0.5, angular_speed=0.3):
#         self.master = master
#         self.cmd_vel_publisher = cmd_vel_publisher
#         self.linear_speed = linear_speed
#         self.angular_speed = angular_speed
#         self.twist = Twist()
#         self.is_moving = False # Thêm biến để theo dõi trạng thái di chuyển
#         self.create_widgets()

#     def create_widgets(self):
#         # Frame chứa các nút điều khiển
#         control_frame = ttk.Frame(self.master)

#         # Nút lên
#         self.up_button = ttk.Button(control_frame, text="↑", width=3)
#         self.up_button.grid(row=0, column=1, padx=5, pady=5)
#         self.up_button.bind("<ButtonPress-1>", self.start_move_forward) # Sử dụng hàm start
#         self.up_button.bind("<ButtonRelease-1>", self.stop)

#         # Nút xuống
#         self.down_button = ttk.Button(control_frame, text="↓", width=3)
#         self.down_button.grid(row=2, column=1, padx=5, pady=5)
#         self.down_button.bind("<ButtonPress-1>", self.start_move_backward) # Sử dụng hàm start
#         self.down_button.bind("<ButtonRelease-1>", self.stop)

#         # Nút trái
#         self.left_button = ttk.Button(control_frame, text="←", width=3)
#         self.left_button.grid(row=1, column=0, padx=5, pady=5)
#         self.left_button.bind("<ButtonPress-1>",  self.start_turn_left) # Sử dụng hàm start
#         self.left_button.bind("<ButtonRelease-1>", self.stop)

#         # Nút phải
#         self.right_button = ttk.Button(control_frame, text="→", width=3)
#         self.right_button.grid(row=1, column=2, padx=5, pady=5)
#         self.right_button.bind("<ButtonPress-1>", self.start_turn_right) # Sử dụng hàm start
#         self.right_button.bind("<ButtonRelease-1>", self.stop)

#         # Nút Dừng khẩn cấp
#         self.stop_button = ttk.Button(control_frame, text="Dừng", width=5, command=self.stop, style="Emergency.TButton")
#         self.stop_button.grid(row=1, column=1, padx=5, pady=5)

#         control_frame.grid(row = 7, column = 0, columnspan = 2, pady = 5)

#         # Định nghĩa style cho nút dừng khẩn cấp
#         style = ttk.Style()
#         style.configure("Emergency.TButton", background="red", foreground="white")

#     # Hàm để bắt đầu di chuyển liên tục
#     def start_move_forward(self, event=None):
#         self.is_moving = True
#         self.move_forward()
#         self.continuous_move() # Gọi hàm di chuyển liên tục

#     def start_move_backward(self, event=None):
#         self.is_moving = True
#         self.move_backward()
#         self.continuous_move()

#     def start_turn_left(self, event=None):
#         self.is_moving = True
#         self.turn_left()
#         self.continuous_move()

#     def start_turn_right(self, event=None):
#          self.is_moving = True
#          self.turn_right()
#          self.continuous_move()

#     # Hàm để di chuyển liên tục
#     def continuous_move(self):
#         if self.is_moving:
#             self.publish_velocity() # Gửi vận tốc
#             self.master.after(50, self.continuous_move) # Lặp lại sau 50ms

#     def move_forward(self):
#         self.twist.linear.x = self.linear_speed
#         self.twist.angular.z = 0.0

#     def move_backward(self):
#         self.twist.linear.x = -self.linear_speed
#         self.twist.angular.z = 0.0

#     def turn_left(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = self.angular_speed

#     def turn_right(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = -self.angular_speed

#     def stop(self, event = None):
#         self.is_moving = False # Dừng di chuyển
#         self.twist.linear.x = 0.0
#         self.twist.linear.y = 0.0
#         self.twist.linear.z = 0.0
#         self.twist.angular.x = 0.0
#         self.twist.angular.y = 0.0
#         self.twist.angular.z = 0.0
#         self.publish_velocity()

#     def publish_velocity(self):
#         self.cmd_vel_publisher.publish(self.twist)

# # import tkinter as tk
# # from geometry_msgs.msg import Twist
# # import rospy
# # from tkinter import ttk

# # class TeleopControl:
# #     def __init__(self, master, cmd_vel_publisher, linear_speed=0.5, angular_speed=0.3):
# #         self.master = master
# #         self.cmd_vel_publisher = cmd_vel_publisher
# #         self.linear_speed = linear_speed
# #         self.angular_speed = angular_speed
# #         self.twist = Twist()
# #         self.is_moving = False  # Thêm biến để theo dõi trạng thái di chuyển
# #         self.control_frame = ttk.Frame(self.master)
# #         self.create_widgets()

# #     def create_widgets(self):
# #         # Nút lên
# #         self.up_button = ttk.Button(self.control_frame, text="↑", width=3)
# #         self.up_button.grid(row=0, column=1, padx=5, pady=5)
# #         self.up_button.bind("<ButtonPress-1>", self.start_move_forward)  # Sử dụng hàm start
# #         self.up_button.bind("<ButtonRelease-1>", self.stop)

# #         # Nút xuống
# #         self.down_button = ttk.Button(self.control_frame, text="↓", width=3)
# #         self.down_button.grid(row=2, column=1, padx=5, pady=5)
# #         self.down_button.bind("<ButtonPress-1>", self.start_move_backward)  # Sử dụng hàm start
# #         self.down_button.bind("<ButtonRelease-1>", self.stop)

# #         # Nút trái
# #         self.left_button = ttk.Button(self.control_frame, text="←", width=3)
# #         self.left_button.grid(row=1, column=0, padx=5, pady=5)
# #         self.left_button.bind("<ButtonPress-1>", self.start_turn_left)  # Sử dụng hàm start
# #         self.left_button.bind("<ButtonRelease-1>", self.stop)

# #         # Nút phải
# #         self.right_button = ttk.Button(self.control_frame, text="→", width=3)
# #         self.right_button.grid(row=1, column=2, padx=5, pady=5)
# #         self.right_button.bind("<ButtonPress-1>", self.start_turn_right)  # Sử dụng hàm start
# #         self.right_button.bind("<ButtonRelease-1>", self.stop)

# #         # Nút Dừng khẩn cấp
# #         self.stop_button = ttk.Button(self.control_frame, text="Dừng", width=5, command=self.stop,
# #                                         style="Emergency.TButton")
# #         self.stop_button.grid(row=1, column=1, padx=5, pady=5)

# #         # Định nghĩa style cho nút dừng khẩn cấp
# #         style = ttk.Style()
# #         style.configure("Emergency.TButton", background="red", foreground="white")

# #     # Hàm để bắt đầu di chuyển liên tục
# #     def start_move_forward(self, event=None):
# #         self.is_moving = True
# #         self.move_forward()
# #         self.continuous_move()  # Gọi hàm di chuyển liên tục

# #     def start_move_backward(self, event=None):
# #         self.is_moving = True
# #         self.move_backward()
# #         self.continuous_move()

# #     def start_turn_left(self, event=None):
# #         self.is_moving = True
# #         self.turn_left()
# #         self.continuous_move()

# #     def start_turn_right(self, event=None):
# #         self.is_moving = True
# #         self.turn_right()
# #         self.continuous_move()

# #     # Hàm để di chuyển liên tục
# #     def continuous_move(self):
# #         if self.is_moving:
# #             self.publish_velocity()  # Gửi vận tốc
# #             self.master.after(50, self.continuous_move)  # Lặp lại sau 50ms

# #     def move_forward(self):
# #         self.twist.linear.x = self.linear_speed
# #         self.twist.angular.z = 0.0

# #     def move_backward(self):
# #         self.twist.linear.x = -self.linear_speed
# #         self.twist.angular.z = 0.0

# #     def turn_left(self):
# #         self.twist.linear.x = 0.0
# #         self.twist.angular.z = self.angular_speed

# #     def turn_right(self):
# #         self.twist.linear.x = 0.0
# #         self.twist.angular.z = -self.angular_speed

# #     def stop(self, event=None):
# #         self.is_moving = False  # Dừng di chuyển
# #         self.twist.linear.x = 0.0
# #         self.twist.linear.y = 0.0
# #         self.twist.linear.z = 0.0
# #         self.twist.angular.x = 0.0
# #         self.twist.angular.y = 0.0
# #         self.twist.angular.z = 0.0
# #         self.publish_velocity()

# #     def publish_velocity(self):
# #         self.cmd_vel_publisher.publish(self.twist)

# #     def get_control_frame(self):
# #         return self.control_frame

import tkinter as tk
from geometry_msgs.msg import Twist
import rospy
from tkinter import ttk

class TeleopControl:
    def __init__(self, master, cmd_vel_publisher, linear_speed=0.5, angular_speed=0.3):
        self.master = master
        self.cmd_vel_publisher = cmd_vel_publisher
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.twist = Twist()
        self.is_moving = False  # Thêm biến để theo dõi trạng thái di chuyển
        self.create_widgets()

    def create_widgets(self):
        # Frame chứa các nút điều khiển
        control_frame = ttk.Frame(self.master)

        # Nút lên (tiến)
        self.up_button = ttk.Button(control_frame, text="↑", width=3)
        self.up_button.grid(row=0, column=2, padx=5, pady=5)
        self.up_button.bind("<ButtonPress-1>", self.start_move_forward)
        self.up_button.bind("<ButtonRelease-1>", self.stop)

        # Nút xuống (lùi)
        self.down_button = ttk.Button(control_frame, text="↓", width=3)
        self.down_button.grid(row=2, column=2, padx=5, pady=5)
        self.down_button.bind("<ButtonPress-1>", self.start_move_backward)
        self.down_button.bind("<ButtonRelease-1>", self.stop)

        # Nút trái
        self.left_button = ttk.Button(control_frame, text="←", width=3)
        self.left_button.grid(row=1, column=1, padx=5, pady=5)
        self.left_button.bind("<ButtonPress-1>", self.start_turn_left)
        self.left_button.bind("<ButtonRelease-1>", self.stop)

        # Nút phải
        self.right_button = ttk.Button(control_frame, text="→", width=3)
        self.right_button.grid(row=1, column=3, padx=5, pady=5)
        self.right_button.bind("<ButtonPress-1>", self.start_turn_right)
        self.right_button.bind("<ButtonRelease-1>", self.stop)

        # Nút lên trái (tiến trái)
        self.up_left_button = ttk.Button(control_frame, text="↖", width=3)
        self.up_left_button.grid(row=0, column=1, padx=5, pady=5)
        self.up_left_button.bind("<ButtonPress-1>", self.start_move_forward_left)
        self.up_left_button.bind("<ButtonRelease-1>", self.stop)

        # Nút lên phải (tiến phải)
        self.up_right_button = ttk.Button(control_frame, text="↗", width=3)
        self.up_right_button.grid(row=0, column=3, padx=5, pady=5)
        self.up_right_button.bind("<ButtonPress-1>", self.start_move_forward_right)
        self.up_right_button.bind("<ButtonRelease-1>", self.stop)

        # Nút xuống trái (lùi trái)
        self.down_left_button = ttk.Button(control_frame, text="↙", width=3)
        self.down_left_button.grid(row=2, column=1, padx=5, pady=5)
        self.down_left_button.bind("<ButtonPress-1>", self.start_move_backward_left)
        self.down_left_button.bind("<ButtonRelease-1>", self.stop)

        # Nút xuống phải (lùi phải)
        self.down_right_button = ttk.Button(control_frame, text="↘", width=3)
        self.down_right_button.grid(row=2, column=3, padx=5, pady=5)
        self.down_right_button.bind("<ButtonPress-1>", self.start_move_backward_right)
        self.down_right_button.bind("<ButtonRelease-1>", self.stop)

        # Nút Dừng khẩn cấp
        self.stop_button = ttk.Button(control_frame, text="Dừng", width=5, command=self.stop, style="Emergency.TButton")
        self.stop_button.grid(row=1, column=2, padx=5, pady=5)

        control_frame.grid(row=7, column=1, columnspan=2, pady=5)

        # Định nghĩa style cho nút dừng khẩn cấp
        style = ttk.Style()
        style.configure("Emergency.TButton", background="red", foreground="white")

    # Các hàm để bắt đầu di chuyển
    def start_move_forward(self, event=None):
        self.is_moving = True
        self.move_forward()
        self.continuous_move()

    def start_move_backward(self, event=None):
        self.is_moving = True
        self.move_backward()
        self.continuous_move()

    def start_turn_left(self, event=None):
        self.is_moving = True
        self.turn_left()
        self.continuous_move()

    def start_turn_right(self, event=None):
        self.is_moving = True
        self.turn_right()
        self.continuous_move()

    def start_move_forward_left(self, event=None):
        self.is_moving = True
        self.move_forward_left()
        self.continuous_move()

    def start_move_forward_right(self, event=None):
        self.is_moving = True
        self.move_forward_right()
        self.continuous_move()

    def start_move_backward_left(self, event=None):
        self.is_moving = True
        self.move_backward_left()
        self.continuous_move()

    def start_move_backward_right(self, event=None):
        self.is_moving = True
        self.move_backward_right()
        self.continuous_move()

    # Hàm để di chuyển liên tục
    def continuous_move(self):
        if self.is_moving:
            self.publish_velocity()  # Gửi vận tốc
            self.master.after(50, self.continuous_move)  # Lặp lại sau 50ms

    # Các hàm thiết lập vận tốc
    def move_forward(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0

    def move_backward(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = 0.0

    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = self.angular_speed

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -self.angular_speed

    def move_forward_left(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed

    def move_forward_right(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = -self.angular_speed

    def move_backward_left(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = self.angular_speed

    def move_backward_right(self):
        self.twist.linear.x = -self.linear_speed
        self.twist.angular.z = -self.angular_speed

    def stop(self, event=None):
        self.is_moving = False  # Dừng di chuyển
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.publish_velocity()

    def publish_velocity(self):
        self.cmd_vel_publisher.publish(self.twist)