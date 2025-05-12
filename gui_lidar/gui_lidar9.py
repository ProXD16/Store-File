#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import tkinter as tk
import threading
import subprocess

class MapDisplay:
    def __init__(self, master):
        self.master = master
        self.goal_position = None
        self.is_moving = False
        self.nav_goal_mode = False

        # Create map display area (matplotlib canvas can be used here as before)
        self.create_widgets()

    def create_widgets(self):
        # Create Entry widgets for robot position (x, y, z, w)
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

        # Add custom text box to show output or status
        self.custom_textbox = tk.Text(self.master, height=4, width=30)
        self.custom_textbox.grid(row=4, column=0, columnspan=2, pady=10)

        # Send Goal Button
        send_button = tk.Button(self.master, text="Send Goal", command=self.send_goal_thread)
        send_button.grid(row=5, column=0, columnspan=2, pady=10)

        # 2D Nav Goal Button
        self.nav_goal_button = tk.Button(self.master, text="2D Nav Goal", command=self.toggle_nav_goal_mode)
        self.nav_goal_button.grid(row=6, column=0, columnspan=2)

        # Position Label to display the robot's position
        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=7, column=0, columnspan=2)

        # Add a button for custom functionality (demonstration)
        self.custom_button = tk.Button(self.master, text="Custom Function", command=self.custom_function)
        self.custom_button.grid(row=8, column=0, columnspan=2, pady=10)

    def custom_function(self):
        # Example of a custom function where you add something to the textbox
        self.custom_textbox.insert(tk.END, "Custom Function Triggered\n")

    def send_goal_thread(self):
        # Simulate sending a goal in a separate thread
        threading.Thread(target=self.send_goal).start()

    def send_goal(self):
        # For demonstration, let's just print the goal
        goal_x = self.entry_x.get()
        goal_y = self.entry_y.get()
        goal_z = self.entry_z.get()
        goal_w = self.entry_w.get()
        self.custom_textbox.insert(tk.END, f"Goal set to X: {goal_x}, Y: {goal_y}, Z: {goal_z}, W: {goal_w}\n")

    def toggle_nav_goal_mode(self):
        # Toggle the navigation goal mode
        self.nav_goal_mode = not self.nav_goal_mode
        if self.nav_goal_mode:
            self.nav_goal_button.config(bg='green')
        else:
            self.nav_goal_button.config(bg='default')
def connect_to_ros():
    # Command to connect to ROS using launch file
    command = "roslaunch mir_driver mir.launch mir_hostname:=192.168.0.172"
    
    try:
        # Execute the roslaunch command
        subprocess.run(command, shell=True, check=True)
        print("Kết nối tới ROS thành công!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Lỗi khi kết nối tới ROS: {e}")
        return False

if __name__ == '__main__':
    root = tk.Tk()
    root.title("Map Display")

    connect_button = tk.Button(root, text="Kết nối tới ROS", command=lambda: connect_to_ros())
    connect_button.pack(pady=20)

    if connect_to_ros():  # Check if ROS connection is successful
        app = MapDisplay(root)

    root.mainloop()

