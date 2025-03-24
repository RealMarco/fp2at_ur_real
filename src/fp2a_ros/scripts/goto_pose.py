#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  4 15:46:35 2023

@author: marco
"""
import socket
import struct
import rospy
import threading
import numpy as np
from scipy.spatial.transform import Rotation
import sys
import moveit_commander
import geometry_msgs.msg
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped
import copy

HOST= "172.20.172.102"
PORT = 30003  # socket
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)


def get_pos():
    data = 	s.recv(1220)  # 包的长度
    while(len(data)<492):
        print('re-receive data of 1st arm, got ', len(data))
        data =         s.recv(1220)  # 包的长度

    mx = struct.unpack('!d', data[444:452])[0]
    my = struct.unpack('!d', data[452:460])[0]
    mz = struct.unpack('!d', data[460:468])[0]
    rx = struct.unpack('!d', data[468:476])[0]
    ry = struct.unpack('!d', data[476:484])[0]
    rz = struct.unpack('!d', data[484:492])[0]
    return [mx, my, mz, rx, ry, rz]

def goto_pose(target, a = 0.3, v = 0.08):    # absolute target(x, y, z, r, p, q); method 'j' is movej
    	
	# with threading.Lock():
	
	[mx, my, mz, rx, ry, rz] = 	get_pos()
	with lock:
        # move rotations at first
		urscript = "movej(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(mx, my, mz, target[3], target[4], target[5], a, v) #  joint planning movement, fatest
		s.send(urscript.encode('utf8'))
        # then move positions
		rospy.sleep(2)
		urscript = "movel(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(target[0], target[1], target[2], target[3], target[4], target[5], a, v) #straight line movement 
	#        urscript = "movep(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(target[0], target[1], target[2], target[3], target[4], target[5], a, v) # uniform-speed movement of end-effector
		s.send(urscript.encode('utf8'))
	
class UR5MotionPlanner:
    def __init__(self, group_name, reference_frame, end_effector_link):
        
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #TODO tf buffer length
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer) # essential for subcribe the tf info
		# Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Instantiate a RobotCommander object to interface with the robot
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object to interact with the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object to plan and execute motions
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame for motion planning
        self.move_group.set_pose_reference_frame(reference_frame)

        # Set the end-effector link
        self.move_group.set_end_effector_link(end_effector_link)
        
        self.move_group.set_max_velocity_scaling_factor(0.01)  # Set maximum velocity scaling factor to 50% of the default
        self.move_group.set_max_acceleration_scaling_factor(0.01)



    def goto_pose(self, target_pose):
        # Set the start state
        waypoints= []
        current_pose =  self.move_group.get_current_pose() # in the format of PoseStamped() w.r.t. base_link
#        print("Pose before executing:", current_pose)
#        self.move_group.set_start_state_to_current_state()
        
		# Plan the motion
#        waypoints.append(copy.deepcopy(current_pose.pose))
        waypoints.append(copy.deepcopy(target_pose))
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  #,avoid_collisions = True, eef_step 0.01m, 0.005m  # jump_threshold 0.0
		# move_group.plan and move_group.go are for joint motion planning
		
#        print("Plan: ", plan)
		# Execute the motion
        self.move_group.execute(plan, wait=True)
		
        # Clear targets
        self.move_group.clear_pose_targets()

    def shutdown(self):
        # Shutdown MoveIt commander
        moveit_commander.roscpp_shutdown()
        
    def get_tf(self, target_frame, source_frame): # it works for UR
        transform = self.tf2_buffer.lookup_transform(target_frame, # The frame that data should be transformed to
                                                	 source_frame, # The frame where the data originated from
                                                	 rospy.Time(0), #get the tf at first available time
                                                	 rospy.Duration(1.0)) #wait for 1 second
        
        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z

        pose_stamped.pose.orientation.x = transform.transform.rotation.x
        pose_stamped.pose.orientation.y = transform.transform.rotation.y
        pose_stamped.pose.orientation.z = transform.transform.rotation.z
        pose_stamped.pose.orientation.w = transform.transform.rotation.w

        return pose_stamped


def euler_to_quat_to_euler(gripper_pose):
	print('untransformed target gripper_pose', gripper_pose)
	for i in range(3,6): #  re-range from [-pi, pi] to [0, 2pi]
		gripper_pose[i] = gripper_pose[i]+2*np.pi if gripper_pose[i] <0 else gripper_pose[i]
	print('gripper_pose in 0-2pi', gripper_pose)
	quat = Rotation.from_euler('XYZ', np.array(gripper_pose[3:]), degrees=False).as_quat() # -np.pi
	print('quat:', quat)
	
	#TODO keep it or not; pre-process from 	_get_action() in launch_utils.py
	quat = np.array(quat) / np.linalg.norm(quat, axis=-1, keepdims=True) # orientations
	if quat[-1] < 0: # maybe useless
		quat = -quat
	print('Pre-pocessed quat:', quat)
	
	rot = Rotation.from_quat(quat).as_euler('XYZ', degrees=False) # + np.pi # in (0,2pi)  
	rot[rot>np.pi] = rot[rot>np.pi] -2*np.pi #  re-range from [0, 2pi] to [-pi, pi] for execution
	assert np.min(rot) >= -np.pi and np.max(rot) <= np.pi
	print('transfromed Euler angles:', rot)
	
	return [gripper_pose[0],gripper_pose[1],gripper_pose[2],rot[0], rot[1], rot[2]]

def main():
	rospy.init_node('ur5_motion_planning', anonymous=True)
	rospy.sleep(0.5)
	#TODO list
	# use base_link instead of base
	# 
	
	# MoveIt goto
	action = (-0.07486150, -0.202575, 0.27085,-0.369427, -0.929156, 0.01345, 0.003359)  # initail pose
	action2 = (-0.193, -0.729, 0.278,-0.506, 0.589, -0.548, -0.313)
	action3 = (-0.160, -0.180, 0.165,0.353, 0.935, 0.011, 0.025)
	
	
	target_pose_1 = geometry_msgs.msg.Pose()
	target_pose_1.position.x = action[0]
	target_pose_1.position.y = action[1]
	target_pose_1.position.z = action[2]
	target_pose_1.orientation.x = action[3]
	target_pose_1.orientation.y = action[4]
	target_pose_1.orientation.z = action[5]
	target_pose_1.orientation.w = action[6]
	
	target_pose_2 = geometry_msgs.msg.Pose()
	target_pose_2.position.x = action2[0]
	target_pose_2.position.y = action2[1]
	target_pose_2.position.z = action2[2]
	target_pose_2.orientation.x = action2[3]
	target_pose_2.orientation.y = action2[4]
	target_pose_2.orientation.z = action2[5]
	target_pose_2.orientation.w = action2[6]

	target_pose_3 = geometry_msgs.msg.Pose()
	target_pose_3.position.x = action3[0]
	target_pose_3.position.y = action3[1]
	target_pose_3.position.z = action3[2]
	target_pose_3.orientation.x = action3[3]
	target_pose_3.orientation.y = action3[4]
	target_pose_3.orientation.z = action3[5]
	target_pose_3.orientation.w = action3[6]
	
	# Create UR5MotionPlanner object
	planner = UR5MotionPlanner("manipulator", "base", "tool0") # manipulator
	
	
    # Call the function to plan and execute motions
	planner.goto_pose(target_pose_1)
    # Shutdown the planner
	planner.shutdown()
    # End the ROS node
	print('Pose after executing:',planner.get_tf("base", "tool0"))
	rospy.signal_shutdown("Motion planning completed.")



if __name__ == '__main__':
	try:
		lock = threading.Lock()
		main()
	except rospy.ROSInterruptException:
		pass