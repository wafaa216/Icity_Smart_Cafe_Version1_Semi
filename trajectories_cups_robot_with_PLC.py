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
var1 = 0  
cups_robot_reached_dispenser = 0 

global PLC_cup_is_dropped 
global cups_rob_state
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('traj_node',
                anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "L_xarm6"
group = moveit_commander.MoveGroupCommander(group_name)
group2_name = "R_xarm6"
group2= moveit_commander.MoveGroupCommander(group2_name)
display_traj_pub = display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)
cups_robot_states_pub = rospy.Publisher('cups_robot_states', Int16, queue_size=10)

printed = False

def callback(data):
	PLC_cup_is_dropped = data.data
	print("waiting signal from plc")
	while True:
		if PLC_cup_is_dropped ==1 :
			cups_robot_espresso_plate(PLC_cup_is_dropped)
		elif PLC_cup_is_dropped == 2 :
			cups_robot_to_deleivery_door(PLC_cup_is_dropped)


def sub_func():
    rospy.Subscriber('PLC_vars_topic', Int16, callback)
    rospy.spin()

def cups_robot_main():
#wait signal from PLC to start
	group.set_named_target("left_home")
	plan1 = group.plan()
	rospy.sleep(0.1)
	plan = group.go(wait=True)

#to cup dispenser  
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = math.radians(-29)
	joint_goal[1] = math.radians(0)
	joint_goal[2] = math.radians(-13)
	joint_goal[3] = math.radians(-178)
	joint_goal[4] = math.radians(73)
	joint_goal[5] = math.radians(86)
	group.go(joint_goal, wait=True)


# Send Signal to PLC 
	cups_robot_reached_dispenser = 100 
	print("Cups_robot_reached_dispenser")
	print(cups_robot_reached_dispenser)

#wait signal from PLC to continue
	sub_func()



def cups_robot_espresso_plate(d):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = math.radians(17)
	joint_goal[1] = math.radians(29)
	joint_goal[2] = math.radians(-65)
	joint_goal[3] = math.radians(-162)
	joint_goal[4] = math.radians(62)
	joint_goal[5] = math.radians(69)
	group.go(joint_goal, wait=True)
	print("Robot Put the Cup on the Espresso Machine")
	cups_rob_state = 1
	cups_robot_states_pub .publish(cups_rob_state)
	sub_func()

def cups_robot_to_deleivery_door(x):
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = math.radians(152)
	joint_goal[1] = math.radians(27)
	joint_goal[2] = math.radians(-26)
	joint_goal[3] = math.radians(137)
	joint_goal[4] = math.radians(107)
	joint_goal[5] = math.radians(84)
	print("Robot Reached Deleivery Door")
	group.go(joint_goal, wait=True)




def test():
	joint_goal = group.get_current_joint_values()
	joint_goal[0] = math.radians(152)
	joint_goal[1] = math.radians(27)
	joint_goal[2] = math.radians(-26)
	joint_goal[3] = math.radians(137)
	joint_goal[4] = math.radians(107)
	joint_goal[5] = math.radians(84)
	group.go(joint_goal, wait=True)

def cartisian_movement_in_x():
	x = input(" enter x val:\n")
	waypoints = []
	wpose = group.get_current_pose().pose
	wpose.position.x -=  float(x) 
	waypoints.append(copy.deepcopy(wpose))
	group.set_pose_target(wpose)
	rospy.sleep(1)
	plan = group.go()	
	#group . execute ( plan , wait = True )

def cartisian_movement_in_y():
	y = input(" enter y val:\n")
	waypoints = []
	wpose = group.get_current_pose().pose
	wpose.position.y +=  float(y)  # First move up (z)
	waypoints.append(copy.deepcopy(wpose))
	group.set_pose_target(wpose)
	(plan, fraction) = group.compute_cartesian_path(waypoints,0.01, 0.0) 
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	# Publish
	display_traj_pub.publish(display_trajectory);
#	plan = group.go()
	rospy.sleep(1)

	#group . execute ( plan , wait = True )
	plan = group.go()

def cartisian_movement_in_z():
	z = input(" enter z val:\n")
	waypoints = []
	wpose = group.get_current_pose().pose
	wpose.position.z += float(z)  # First move up (z)
	waypoints.append(copy.deepcopy(wpose))
	group.set_pose_target(wpose)
	rospy.sleep(1)
	plan = group.go()
	#group.stop()
try:
	while not rospy.is_shutdown(): 
		func = input("enter where to move,available options (home/r1start/x/y/z/gripper):\n")

	# input from plc 
		if func == "r1start":
			cups_robot_main()		
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


