#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tkinter as tk

def movebase_client(x, y, z, w):
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

def send_goal():
    x = float(entry_x.get())
    y = float(entry_y.get())
    z = float(entry_z.get())
    w = float(entry_w.get())
    result = movebase_client(x, y, z, w)
    if result:
        rospy.loginfo("Goal execution done!")

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    win = tk.Tk()

    tk.Label(win, text="X:").grid(row=0, column=0)
    entry_x = tk.Entry(win)
    entry_x.grid(row=0, column=1)

    tk.Label(win, text="Y:").grid(row=1, column=0)
    entry_y = tk.Entry(win)
    entry_y.grid(row=1, column=1)

    tk.Label(win, text="Z:").grid(row=2, column=0)
    entry_z = tk.Entry(win)
    entry_z.grid(row=2, column=1)

    tk.Label(win, text="W:").grid(row=3, column=0)
    entry_w = tk.Entry(win)
    entry_w.grid(row=3, column=1)

    send_button = tk.Button(win, text="Send Goal", command=send_goal)
    send_button.grid(row=4, column=0, columnspan=2, pady=10)

    win.mainloop()
