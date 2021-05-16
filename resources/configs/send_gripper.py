#!/usr/bin/python

import rospy
import actionlib
import control_msgs.msg
import time
 
print("starting")

rospy.init_node('gripper_control')
print("node intiialized")
 
# Create an action client
client = actionlib.SimpleActionClient(
	'/gripper_controller/gripper_cmd',  # namespace of the action topics
	control_msgs.msg.GripperCommandAction # action type
)
print("action client created, waiting for server")
	
# Wait until the action server has been started and is listening for goals
client.wait_for_server()

print("server started (new). Sending a goal")
 
# Create a goal to send (to the action server)
goal = control_msgs.msg.GripperCommandGoal()
goal.command.position = 0.6 # From 0.0 to 0.8
goal.command.max_effort = -1.0  # Do not limit the effort
client.send_goal(goal)
client.wait_for_result()
print("Opened gripper. Now closing:")
time.sleep(3)
goal.command.position = 0.0 # From 0.0 to 0.8
goal.command.max_effort = -1.0  # Do not limit the effort
client.send_goal(goal)
client.wait_for_result()
print("got results")
