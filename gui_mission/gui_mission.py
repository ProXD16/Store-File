#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tkinter import *
import tkinter as tk
from tkinter import ttk
def movebase_client(x):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    if x==1:
        goal.target_pose.pose.position.x = 6.85
        goal.target_pose.pose.position.y = 21.90
        goal.target_pose.pose.orientation.z = -51.116
        goal.target_pose.pose.orientation.w = 20
    elif x==2:
        goal.target_pose.pose.position.x = 10.300
        goal.target_pose.pose.position.y = 18.100
        goal.target_pose.pose.orientation.z = -132.709
        goal.target_pose.pose.orientation.w = 20
    else:
        goal.target_pose.pose.position.x = 12.950
        goal.target_pose.pose.position.y = 9.900
        goal.target_pose.pose.orientation.z = -48.013
        goal.target_pose.pose.orientation.w = 9.900



   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()
def goto_goal():
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.loginfo("goto_goal("+str(1)+"):")
        result = movebase_client(1)
        rospy.loginfo("Goal execution done!")
        rospy.sleep(5)
        if result:
            rospy.loginfo("Goto goal Goal execution done!"+str(result))
def goto_goal1():
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.loginfo("goto_goal("+str(1)+"):")
        result = movebase_client(1)
        rospy.loginfo("Goal execution done!")
        rospy.sleep(1)
        if result:
            rospy.loginfo("Goto goal Goal execution done!"+str(result))
def goto_goal2():
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.loginfo("goto_goal("+str(2)+"):")
        result = movebase_client(2)
        rospy.loginfo("Goal execution done!")
        rospy.sleep(1)
        if result:
            rospy.loginfo("Goto goal Goal execution done!"+str(result))
def goto_goal3():
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.loginfo("goto_goal("+str(3)+"):")
        result = movebase_client(3)
        rospy.loginfo("Goal execution done!")
        rospy.sleep(1)
        if result:
            rospy.loginfo("Goto goal Goal execution done!"+str(result))
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        rate = rospy.Rate(10) # 10hz
        win = tk.Tk()
        button_go1 = tk.Button(win, text="Goal 1", command=goto_goal1)
        button_go2 = tk.Button(win, text="Goal 2", command=goto_goal2)
        button_go3 = tk.Button(win, text="Goal 3", command=goto_goal3)
        button_go = tk.Button(win, text="GO", command=goto_goal)
        button_go1.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        button_go2.grid(row=0, column=1, sticky="ew", padx=5, pady=5)
        button_go3.grid(row=0, column=2, sticky="ew", padx=5, pady=5)
        button_go.grid(row=4, column=4, sticky="ew", padx=5, pady=5)
        win.mainloop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
