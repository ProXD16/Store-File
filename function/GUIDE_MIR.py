#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tf
import threading
import requests
import json
import matplotlib.pyplot as plt
import subprocess
from tkinter import messagebox
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import os
import signal
from matplotlib.widgets import Slider
from matplotlib.backend_bases import MouseButton
from geometry_msgs.msg import Pose
import sympy

global ip
ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

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
        self.is_moving = False
        self.drag_start = None 
        self.path_points = []
        self.is_on = True
        self.zoom_level = 1.0
        self.pan_start = None
        self.equation = "x" # default equation
        self.circle_radius = 1.0
        
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2, sticky="nsew")
        
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_goal_callback)
        self.path_subscriber = rospy.Subscriber("/move_base_node/SBPLLatticePlanner/plan", Path, self.path_callback)
        self.front_scan_subscriber = rospy.Subscriber("/f_scan", LaserScan, self.f_scan_callback)
        self.back_scan_subscriber = rospy.Subscriber("/b_scan", LaserScan, self.b_scan_callback)
        self.tf_listener = tf.TransformListener()
	
        self.fig = Figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)

        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_drag)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect('scroll_event', self.zoom_event)

        self.create_widgets()
        self.update_position_continuously()
        self.pin()
        
        #Thêm Slider
        self.zoom_slider_ax = self.fig.add_axes([0.15, 0.01, 0.65, 0.03])
        self.zoom_slider = Slider(self.zoom_slider_ax, 'Zoom', 0.1, 5.0, valinit=1.0)
        self.zoom_slider.on_changed(self.update_zoom)
        self.path_publisher = rospy.Publisher("/path_from_equation", Path, queue_size=10)
    
    def create_widgets(self):
        self.on_button = tk.Button(self.master, text="OFF", command=self.stop_mir_launch, bg="red", fg="white")
        self.on_button.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        tk.Label(self.master, text="Address IP:").grid(row=1, column=0, padx=10, pady=10, sticky="w")   
        self.entry_ip = tk.Entry(self.master)
        self.entry_ip.grid(row=2, column=0, padx=10, pady=10, sticky="w")
        self.entry_ip.insert(0, "192.168.0.172")

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
        
        tk.Label(self.master, text="Radius:").grid(row=6, column=0, padx=10, pady=10, sticky="w")
        self.entry_radius = tk.Entry(self.master)
        self.entry_radius.grid(row=7, column=0, padx=10, pady=10, sticky="w")
        self.entry_radius.insert(0, "1.0")
        
        tk.Label(self.master, text="Equation:").grid(row=8, column=0, padx=10, pady=10, sticky="w")
        self.entry_equation = tk.Entry(self.master)
        self.entry_equation.grid(row=9, column=0, padx=10, pady=10, sticky="w")
        self.entry_equation.insert(0, "x")


        send_button = tk.Button(self.master, text="Send Goal", command=self.send_goal_thread)
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        self.nav_goal_button = tk.Button(self.master, text="2D Nav Goal", command=self.toggle_nav_goal_mode)
        self.nav_goal_button.grid(row=4, column=1)
        
        self.circle_button = tk.Button(self.master, text="Send Circle", command=self.send_circle_thread)
        self.circle_button.grid(row=7, column=1, pady=10)
        
        self.equation_button = tk.Button(self.master, text="Send Equation", command=self.send_equation_thread)
        self.equation_button.grid(row=9, column=1, pady=10)


        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=10, column=0, columnspan=2)

        self.nav_goal_mode = False  

        self.battery_percentage_value = tk.StringVar()
        self.battery_percentage_display = tk.Label(self.master, textvariable=self.battery_percentage_value, relief="solid", width=10, height=2)
        self.battery_percentage_display.grid(row=11, column=0, columnspan=2, pady=10)

    def stop_mir_launch(self):
        global ip
        ip = self.entry_ip.get()

        if not self.is_on:
            messagebox.showinfo("Info", "MiR driver chưa được khởi chạy!")
            return

        try:
            ros_processes = subprocess.check_output(["pgrep", "-f", "ros"]).decode("utf-8").strip().split("\n")
            shell_processes = subprocess.check_output(["pgrep", "-f", "bash"]).decode("utf-8").strip().split("\n")
            all_processes = set(ros_processes + shell_processes)
            for pid in all_processes:
                try:
                    os.kill(int(pid), signal.SIGINT)  
                except ProcessLookupError:
                    continue 


            self.is_on = False
            messagebox.showinfo("Info", "Đã ngắt kết nối, đóng các terminal và tắt giao diện!")

        except subprocess.CalledProcessError:
            messagebox.showinfo("Info", "Không tìm thấy tiến trình terminal nào liên quan!")
        except Exception as e:
            messagebox.showerror("Error", f"Lỗi khi đóng ứng dụng và các terminal:\n{e}")
        if messagebox.askyesno("Confirm Exit", "Bạn có chắc muốn dừng chương trình không?"):
            rospy.signal_shutdown("User requested shutdown.")
            self.master.quit()  
            os._exit(0)

    def toggle_nav_goal_mode(self):
        self.nav_goal_mode = not self.nav_goal_mode
        button_color = "green" if self.nav_goal_mode else "lightgray"
        self.nav_goal_button.config(bg=button_color)

    def on_click(self, event):
      if event.button == MouseButton.LEFT:
          if event.inaxes == self.ax and self.nav_goal_mode:
              self.drag_start = (event.xdata, event.ydata)
      elif event.button == MouseButton.RIGHT and event.inaxes == self.ax :
            self.pan_start = (event.xdata, event.ydata)

    def on_drag(self, event):
        if self.drag_start is not None and event.inaxes == self.ax:
            x_start, y_start = self.drag_start
            x_end, y_end = event.xdata, event.ydata

            self.entry_x.delete(0, tk.END)
            self.entry_x.insert(0, str(x_start))
            self.entry_y.delete(0, tk.END)
            self.entry_y.insert(0, str(y_start))

            self.ax.clear()
            self.draw_map_and_lidar()
            
            if (x_end is not None and y_end is not None):
                self.ax.annotate('', xy=(x_end, y_end), xytext=(x_start, y_start),
                                arrowprops=dict(arrowstyle='->', color='red', linewidth=2))
            self.canvas.draw()
        
        elif self.pan_start is not None and event.inaxes == self.ax:
            x_start, y_start = self.pan_start
            x_end, y_end = event.xdata, event.ydata
            dx = x_end - x_start
            dy = y_end - y_start
            
            self.pan_start = (x_end,y_end)
            
            xmin, xmax = self.ax.get_xlim()
            ymin, ymax = self.ax.get_ylim()
            
            new_xmin = xmin - dx
            new_xmax = xmax - dx
            new_ymin = ymin - dy
            new_ymax = ymax - dy
            
            self.ax.set_xlim(new_xmin, new_xmax)
            self.ax.set_ylim(new_ymin, new_ymax)
            self.canvas.draw()

    def on_release(self, event):
        if self.drag_start is not None and event.inaxes == self.ax and self.nav_goal_mode:
            x_end, y_end = event.xdata, event.ydata
            z, w = self.calculate_orientation(self.drag_start, (x_end, y_end))

            self.entry_z.delete(0, tk.END)
            self.entry_z.insert(0, str(z))
            self.entry_w.delete(0, tk.END)
            self.entry_w.insert(0, str(w))
            self.send_goal_thread()
        self.drag_start = None
        self.pan_start = None

    def calculate_orientation(self, start, end):
        orientation = np.arctan2(end[1] - start[1], end[0] - start[0])
        z = np.sin(orientation / 2)
        w = np.cos(orientation / 2)
        return z, w
    
    def send_equation_thread(self):
        threading.Thread(target=self.send_equation).start()
    
    def send_equation(self):
       self.equation = self.entry_equation.get()
       self.calculate_and_publish_path(equation_mode = True)

    def send_circle_thread(self):
        threading.Thread(target=self.send_circle).start()

    def send_circle(self):
        try:
            self.circle_radius = float(self.entry_radius.get())
            self.calculate_and_publish_path(circle_mode = True)
        except ValueError:
            messagebox.showerror("Error", "Bán kính phải là một số!")
        except Exception as e:
            print(f"Error during circle path calculation or publishing: {e}")
            messagebox.showerror("Error", f"Lỗi khi tính toán đường tròn:\n{e}")
    
    def calculate_and_publish_path(self,circle_mode = False, equation_mode = False):
         try:
             if circle_mode:
                path_msg = self.generate_circle_path()
             elif equation_mode:
                 path_msg = self.generate_path_from_equation(self.equation)
             else:
                return
             self.path_publisher.publish(path_msg)
             self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
             self.draw_map_and_lidar()
         except Exception as e:
            print(f"Error during path calculation or publishing: {e}")
            messagebox.showerror("Error", f"Lỗi khi tính toán đường đi:\n{e}")

    def generate_path_from_equation(self, equation_str, step=0.1, range_start=-5, range_end=5):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        try:
            x = sympy.symbols('x')
            equation = sympy.sympify(equation_str)
            
            x_vals = np.arange(range_start, range_end, step)
            for x_val in x_vals:
              y_val = equation.evalf(subs={x: x_val})
              pose = PoseStamped()
              pose.header.frame_id = "map"
              pose.header.stamp = rospy.Time.now()
              pose.pose.position.x = float(x_val)
              pose.pose.position.y = float(y_val)
              pose.pose.orientation.w = 1.0
              path_msg.poses.append(pose)
           
        except Exception as e:
             print(f"Error evaluating equation: {e}")
             raise
        return path_msg
    
    def generate_circle_path(self, step = 0.1):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        try:
            (trans, _) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            center_x, center_y = trans[0], trans[1]
            
            for theta in np.arange(0, 2 * np.pi, step):
                 x = center_x + self.circle_radius * np.cos(theta)
                 y = center_y + self.circle_radius * np.sin(theta)

                 pose = PoseStamped()
                 pose.header.frame_id = "map"
                 pose.header.stamp = rospy.Time.now()
                 pose.pose.position.x = x
                 pose.pose.position.y = y
                 pose.pose.orientation.w = 1.0
                 path_msg.poses.append(pose)
        except Exception as e:
             print(f"Error generating circle path: {e}")
             raise
        return path_msg

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
                    point_map = self.tf_listener.transformPoint("map", point_lidar)
                    points.append((point_map.point.x, point_map.point.y))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

        return points
    
    def path_callback(self, path_msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self.draw_map_and_lidar()


    def pin(self):
        try:
            b = requests.get(host + '/status', headers=headers)

            if b.status_code == 200:
                a = json.loads(b.content)
                p = a['battery_percentage']

                self.battery_percentage_display.configure(background="lightgray")
                if p <= 15:
                    self.battery_percentage_display.configure(foreground="red")
                else:
                    self.battery_percentage_display.configure(foreground="blue")

                value = round(p, 2)
                c = f'{value}%'
                self.battery_percentage_value.set(c)
            else:
                print("Failed to retrieve battery status.")
        
        except Exception as e:
            print(f"Error occurred: {e}")

        self.master.after(5000, self.pin)

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
            x, y = self.robot_position

            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                x, y = trans[0], trans[1]
                _, _, yaw = tf.transformations.euler_from_quaternion(rot)
                rect_length = 0.6  
                rect_width = 0.4  
                corners = [
                    (x + rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
                    y + rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw)),
                    (x + rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
                    y + rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
                    (x - rect_length / 2 * np.cos(yaw) + rect_width / 2 * np.sin(yaw),
                    y - rect_length / 2 * np.sin(yaw) - rect_width / 2 * np.cos(yaw)),
                    (x - rect_length / 2 * np.cos(yaw) - rect_width / 2 * np.sin(yaw),
                    y - rect_length / 2 * np.sin(yaw) + rect_width / 2 * np.cos(yaw))
                ]
                robot_shape = plt.Polygon(corners, closed=True, color='white', alpha=0.4)
                self.ax.add_patch(robot_shape)
                tri_side = 0.3
                triangle_points = [
                    (x + tri_side * np.cos(yaw), y + tri_side * np.sin(yaw)),  
                    (x - tri_side / 2 * np.cos(yaw) + (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                    y - tri_side / 2 * np.sin(yaw) - (tri_side * np.sqrt(3) / 2) * np.cos(yaw)),  
                    (x - tri_side / 2 * np.cos(yaw) - (tri_side * np.sqrt(3) / 2) * np.sin(yaw),
                    y - tri_side / 2 * np.sin(yaw) + (tri_side * np.sqrt(3) / 2) * np.cos(yaw))   
                ]
                direction_triangle = plt.Polygon(triangle_points, closed=True, color='blue', alpha=0.7)
                self.ax.add_patch(direction_triangle)

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        if self.f_points:
            x_vals, y_vals = zip(*self.f_points)
            self.ax.plot(x_vals, y_vals, "r.")

        if self.b_points:
            x_vals, y_vals = zip(*self.b_points)
            self.ax.plot(x_vals, y_vals, "r.")

        if self.path_points:
                path_x, path_y = zip(*self.path_points)
                self.ax.plot(path_x, path_y, 'g--', label='Path')
                
        self.update_axes_limits()
        self.canvas.draw()
    
    def update_axes_limits(self):
        if self.map_data:
            map_width = self.map_data.info.width * self.resolution
            map_height = self.map_data.info.height * self.resolution
            
            center_x = self.origin[0] + map_width / 2
            center_y = self.origin[1] + map_height / 2

            width = map_width / self.zoom_level
            height = map_height / self.zoom_level

            xmin = center_x - width / 2
            xmax = center_x + width / 2
            ymin = center_y - height / 2
            ymax = center_y + height / 2

            self.ax.set_xlim(xmin, xmax)
            self.ax.set_ylim(ymin, ymax)

    def update_zoom(self, val):
         self.zoom_level = val
         self.draw_map_and_lidar()

    def zoom_event(self,event):
        if event.inaxes == self.ax:
          
          zoom_factor = 1.1 if event.button == 'up' else 1/1.1
          self.zoom_slider.set_val(self.zoom_slider.val * zoom_factor)
    
    def update_position_continuously(self):
        threading.Thread(target=self.update_position, daemon=True).start()

    def update_position(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                self.robot_position = (trans[0], trans[1])
                self.position_label.config(text=f"Robot Position: ({trans[0]:.2f}, {trans[1]:.2f})")
                self.draw_map_and_lidar()
                rospy.sleep(0.1)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                continue

class FirstDisplay:
    def __init__(self, master):
        self.master = master
        self.is_on = False
        self.map_data = None
        self.resolution = 0.0
        self.origin = (0.0, 0.0)
        self.robot_position = (0.0, 0.0)
        self.goal_position = None
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2, sticky="nsew")
        self.fig = Figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas.get_tk_widget().grid(row=5, column=0, columnspan=2)
        self.create_widgets()
    
    def create_widgets(self):
        self.on_button = tk.Button(self.master, text="ON", command=self.run_mir_launch, bg="green", fg="white")
        self.on_button.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        tk.Label(self.master, text="Address IP:").grid(row=1, column=0, padx=10, pady=10, sticky="w")
        self.entry_ip = tk.Entry(self.master)
        self.entry_ip.grid(row=2, column=0, padx=10, pady=10, sticky="w")
        self.entry_ip.insert(0, "192.168.0.172")
        
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

        send_button = tk.Button(self.master, text="Send Goal")
        send_button.grid(row=4, column=0, columnspan=2, pady=10)

        self.nav_goal_button = tk.Button(self.master, text="2D Nav Goal")
        self.nav_goal_button.grid(row=4, column=1)

        self.position_label = tk.Label(self.master, text="Robot Position: (0.00, 0.00)")
        self.position_label.grid(row=6, column=0, columnspan=2)

        self.nav_goal_mode = False  

    def run_mir_launch(self):
        global ip
        ip = self.entry_ip.get()

        if self.is_on:
            messagebox.showinfo("Info", "MiR driver đã chạy trước đó!")
            return

        try:
            subprocess.Popen(['roslaunch', 'mir_driver', 'mir.launch', f'mir_hostname:={ip}'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.Popen(['rosrun', 'tf', 'tf_monitor'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            self.is_on = True
            messagebox.showinfo("Info", "MiR driver và tf_monitor đã được khởi chạy thành công!")
            self.switch_to_next()

        except Exception as e:
            messagebox.showerror("Error", f"Failed to start MiR driver:\n{e}")

    def switch_to_next(self):
        self.master.after(5000, self.perform_switch)

    def perform_switch(self):
        for widget in self.master.winfo_children():
            widget.destroy()
        rospy.init_node('robot_gui', anonymous=True)
        MapDisplay(self.master)


if __name__ == '__main__':
    root = tk.Tk()
    app = FirstDisplay(root)
    root.mainloop()