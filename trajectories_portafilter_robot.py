#!/usr/bin/env python3
import time
import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from std_msgs.msg import String , Int16
from moveit_commander.conversions import pose_to_list


######## Variables ####
cups_robot_reached_dispenser = 0 

global PLC_cup_is_dropped 

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('traj_node',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group2_name = "R_xarm6"
group2= moveit_commander.MoveGroupCommander(group2_name)
display_traj_pub = display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
printed = False
def callback(data):
	cups_robot_state = data.data
	print("waiting signal from cups_robot")
	while True:
		if cups_robot_state ==1 :
			cups_robot_state =2 ;
			portafilter_robot_press_the_button()
		


def sub_func():
    rospy.Subscriber('cups_robot_states', Int16, callback)
    rospy.spin()

def portafilter_robot_main(): #right robot
#wait signal from PLC to start

	group2.set_named_target("right_home")
	rospy.sleep(0.1)
	plan = group2.go(wait=True)
	group2.stop()

	to_espresso_to_take_the_portafilter()

def to_espresso_to_take_the_portafilter():
	joint_goal = group2.get_current_joint_values()
	joint_goal[0] = math.radians(-51)
	joint_goal[1] = math.radians(39)
	joint_goal[2] = math.radians(-89)
	joint_goal[3] = math.radians(94)
	joint_goal[4] = math.radians(67)
	joint_goal[5] = math.radians(74)
	plan = group2.go(joint_goal, wait=True)
	group2.stop()
	before_grinder_and_press()

def before_grinder_and_press (): 
	joint_goal = group2.get_current_joint_values()
	joint_goal[0] = math.radians(-115)
	joint_goal[1] = math.radians(25)
	joint_goal[2] = math.radians(-13)
	joint_goal[3] = math.radians(154)
	joint_goal[4] = math.radians(96)
	joint_goal[5] = math.radians(0)
	group2.go(joint_goal, wait=True)
	group2.stop()

	to_grinder_and_press()

def to_grinder_and_press():
	joint_goal = group2.get_current_joint_values()
	joint_goal[0] = math.radians(-101)
	joint_goal[1] = math.radians(17)
	joint_goal[2] = math.radians(-19)
	joint_goal[3] = math.radians(-172)
	joint_goal[4] = math.radians(81)
	joint_goal[5] = math.radians(7)
	group2.go(joint_goal, wait=True)
	group2.stop()

#delay 
	time.sleep(2)

#to  espresso again to put the portafilter 
	to_espresso_to_take_the_portafilter()
	print("	wait signal from cups_robot to press the button")
	#wait signal from cups_robot to press the button
	sub_func()

def portafilter_robot_press_the_button():
	print("	pressssssss")

	group2.set_named_target("right_home")
	plan1 = group2.plan()
	rospy.sleep(0.1)
	plan = group2.go(wait=True)









try:
	while not rospy.is_shutdown(): 
		func = input("enter where to move,available options (home/r1start/x/y/z/gripper):\n")

	# input from plc 
		if func == "r2start":
			portafilter_robot_main()		
		
		elif func =="x":
			cartisian_movement_in_x()
		elif func =="y":
			cartisian_movement_in_y()
		elif func =="z":
			cartisian_movement_in_z()
		elif func =="test":
			test()
		else :
			print("wrong input")

	
	moveit_commander.roscpp_shutdown()

except KeyboardInterrupt:
    print("killed")


