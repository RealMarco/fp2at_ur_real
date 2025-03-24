#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 16 18:24:15 2023

@author: marco
"""
import os
import sys
import numpy as np
import pprint
import shutil

import cv2
import rospy
import copy
import tf
import tf2_ros
#import tf2_geometry_msgs
#import threading
import pickle
from multiprocessing import Process, Manager

import hydra
#from hydra import compose, initialize
#from omegaconf import OmegaConf

from cv_bridge import CvBridge #, CvBridgeError
from geometry_msgs.msg import PoseStamped, Wrench
from sensor_msgs.msg import Joy, Image, CameraInfo, JointState
#from std_msgs.msg import Float32MultiArray
#from visualization_msgs.msg import Marker

#from dvnets_franka_msgs.srv import GotoPose, SetGripper, Home
from rlbench.backend.observation import Observation
#from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.demo import Demo

from fp2a_ros.msg import _Robotiq2FGripper_robot_output as outputMsg # Robotiq2FGripper_robot_output
from fp2a_ros.msg import _Robotiq2FGripper_robot_input as inputMsg
import socket
import struct
#from mecheye_ros_interface.srv import CaptureColorMap, CaptureDepthMap, CaptureColorPointCloud
from rlbench.backend import utils

# for RM gripper
import motormaster

#from helpers import utils
#from scipy.spatial.transform import Rotation
'''
# sensor_msgs/JointState.msg
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort

# sensor_msgs/Joy.msg
std_msgs/Header header
float32[] axes
int32[] buttons
'''

HOST= "172.20.172.102"
PORT = 30003  # socket

class PeractDemoInterface:

	def __init__(self, cfg):
		self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.s.connect((HOST,PORT))
		urscript = "set_tcp(p[{}, {}, {}, {}, {}, {}])\n".format(0, 0, 0.260, 0, 0, 0) # 0.214 For Robotiq Gripper, 0.260 for RM gripper
		self.s.send(urscript.encode('utf8'))

		# RM gripper
		self.axis = motormaster.create_axis_modbus_rtu('/dev/ttyUSB3', 115200, 0)
		self.axis.go_home()
#		self.axis.move_to(0)

		rospy.sleep(1)

		self.cfg = cfg

		# setup
		self.loop_rate = rospy.Rate(cfg['ros']['loop_rate'])
		self.base_frame =  self.cfg['frame']['base'] # 'base'  #  'base_link'
		self.ee_frame = self.cfg['frame']['end_effector'] #  'tool0' # 'wrist_3_link','wrist_3' whether to add a tool link or not

		# tools
		self.cv_bridge = CvBridge()
		self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) # store transformations for up to 1200 seconds
		self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer) # essential for subcribe the tf info
		self.mp = Manager()

		# data
		self.curr_data = self.mp.dict({
		    	'front_rgb': None,
		    	'front_depth': None,
		    	'front_camera_info': None,

		    	'overhead_rgb': None,
		    	'overhead_depth': None,
		    	'overhead_camera_info': None,

		    	'joy_states': Joy(), # includes all states of buttons and axes
		    	'joy_pose': None,

		    	'joint_states': None,
		    	'target_pose': None,
		    	'gripper_pose': self.get_tf(self.base_frame, self.ee_frame),
		    	'gripper_states': [1.0, 0.03, 0.03], # None for robotiq
		    	'tcp_force': None,
		})

#		print("self.curr_data['joy_states']", self.curr_data['joy_states'])  # buttons: [] , axes: []
		# states
		self.state = self.mp.dict({
		    	'prev_joy_states': Joy(),
		    	'prev_pose': self.get_tf(self.base_frame, self.ee_frame), # PoseStamped(),  # save previous pose for replay
		    	'new': False,
		    	'keypoint_done': False,
		    	'record': False,
		})

		# keypoint data
		self.keypoint_data = self.mp.list()
		self.keypoint_idxs = self.mp.list([0])

		# topics
		self.front_rgb_sub = rospy.Subscriber(self.cfg['topics']['front_rgb'], Image, self.front_rgb_cb)
		self.front_depth_sub = rospy.Subscriber(self.cfg['topics']['front_depth'], Image, self.front_depth_cb)
		self.front_camera_info_sub = rospy.Subscriber(self.cfg['topics']['front_camera_info'], CameraInfo, self.front_camera_info_cb)

		self.overhead_rgb_sub = rospy.Subscriber(self.cfg['topics']['overhead_rgb'], Image, self.overhead_rgb_cb)
		self.overhead_depth_sub = rospy.Subscriber(self.cfg['topics']['overhead_depth'], Image, self.overhead_depth_cb)
		self.overhead_camera_info_sub = rospy.Subscriber(self.cfg['topics']['overhead_camera_info'], CameraInfo, self.overhead_camera_info_cb)

		#		self.target_pose_sub = rospy.Subscriber(self.cfg['topics']['target_pose'], Marker, self.target_pose_cb) # use marker to indicate target pose...
		# TODO control Roboitiq gripper
		# for robotiq
		'''
		self.gripper_state_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10) #
		self.command = outputMsg.Robotiq2FGripper_robot_output()
		self.active() # activate gripper
		'''

		self.joy_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.joy_state_cb)
#		self.joy_pose_sub = rospy.Subscriber(self.cfg['topics']['joy_pose'], PoseStamped, self.joy_pose_cb)

		#for robotiq
#		self.gripper_state_sub = rospy.Subscriber(self.cfg['topics']['gripper_state'], inputMsg.Robotiq2FGripper_robot_input, self.gripper_state_cb)
		# for RM gripper
		self.gripper_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.gripper_state_cb)
#		self.gripper_pose_sub = rospy.Subscriber(self.cfg['topics']['gripper_pose'],Float32MultiArray, self.gripper_pose_cb) # Method2

		self.joint_states_sub = rospy.Subscriber(self.cfg['topics']['joint_states'], JointState, self.joint_states_cb) #

		#Peract, go to target pose in Rviz, send msg of target_pose, goto target pose in real-world, record the current pose
		#TODO Ours, goto target pose in real-world, record the current pose, directly ask to save in step() (or send msg of the mentioned pose or use a flag)

		self.tcp_force_sub = rospy.Subscriber(self.cfg['topics']['tcp_force'], Wrench, self.tcp_force_cb)

		# controller  #TODO, ur move and gripper control
#		rospy.wait_for_service('franka_goto_pose')
#		self._franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
#		rospy.wait_for_service('franka_set_gripper')
#		self._franka_set_gripper = rospy.ServiceProxy('franka_set_gripper', SetGripper)

#		self.capture_mecheye_images() # capture the initial images from mecheye

		# set language goal by keyboard input
		self.lang_goal = input("Language Goal: ")  #

		rospy.spin()

	'''
	Callbacks
	'''
#	def capture_mecheye_images(self):
#		try:
#		    capture_color_map_service = rospy.ServiceProxy('/capture_color_map', CaptureColorMap)
#		    capture_depth_map_service = rospy.ServiceProxy('/capture_depth_map', CaptureDepthMap)
#		    capture_color_map_service()
#		    capture_depth_map_service()
##		    if response.errorCode ==0:
##		    	rospy.loginfo(f'call for the service /capture_color_map successfully')
##		    else:
##		    	rospy.loginfo(f'Error: call for the service /capture_color_map UNsuccessfully')
#		except rospy.ServiceException as e:
#		    print("Service call failed:", str(e))

	def front_rgb_cb(self, msg):
		self.curr_data['front_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8") #TODO why not rgb8 as agent_interface; for write

	def front_depth_cb(self, msg):
#		self.curr_data['front_depth'] = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")

		img_real_depth = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
#		print('self.curr_data[overhead_depth].shape()', img_real_depth.shape)

#		img_real_depth = np.clip(img_real_depth, 0, 1088)

		# inpaint depth image, but it will increase the noise
#		mask = np.where(img_real_depth == 0, 1, 0).astype(np.uint8)
#		img_real_depth = cv2.inpaint(img_real_depth, mask, 1, cv2.INPAINT_NS) #  fill in pixel value==0, missing value
#
#		#  output most frequently-observed pixel value
#		unique_items, item_counts = np.unique(img_real_depth, return_counts=True)
#		max_count_index = np.argmax(item_counts)
#		most_frequent_item = unique_items[max_count_index]
#		most_frequent_count = item_counts[max_count_index]
#		print("Front: Most frequent item:", most_frequent_item, "Count:", most_frequent_count)

#		print('img_real_depth', np.min(img_real_depth[img_real_depth>0]), np.max(img_real_depth[img_real_depth<2**16-1]))

		# assign 0 to max
#		img_real_depth_copy = np.copy(img_real_depth)
#		img_real_depth_copy[img_real_depth_copy == 0] = 1080
#		img_real_depth = img_real_depth_copy

		img_real_depth = np.clip(img_real_depth, 0, 1088) # 1004, 1088

		img_real_depth_norm =  (img_real_depth- 0)/(img_real_depth.max())
		real_depth3 = utils.float_array_to_rgb_image(img_real_depth_norm, 2**24-1)  # PIL.Image format
		real_depth3_np =np.array(real_depth3)

		self.curr_data['front_depth']  = cv2.cvtColor(real_depth3_np, cv2.COLOR_RGB2BGR) # real_depth3_cv

	def front_camera_info_cb(self, msg):
		self.curr_data['front_camera_info'] = msg

	def overhead_rgb_cb(self, msg):
		self.curr_data['overhead_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

	def overhead_depth_cb(self, msg):
		img_real_depth = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
#		print('self.curr_data[overhead_depth].shape()', img_real_depth.shape)

##		img_real_depth = np.clip(img_real_depth, 0, 925)
#		mask = np.where(img_real_depth == 0, 1, 0).astype(np.uint8)
#		img_real_depth = cv2.inpaint(img_real_depth, mask, 1, cv2.INPAINT_NS) #  fill in pixel value==0, missing value
#
#		unique_items, item_counts = np.unique(img_real_depth, return_counts=True)
#		max_count_index = np.argmax(item_counts)
#		most_frequent_item = unique_items[max_count_index]
#		most_frequent_count = item_counts[max_count_index]
#		print("Overhead: Most frequent item:", most_frequent_item, "Count:", most_frequent_count)

#		print('img_real_depth', np.min(img_real_depth[img_real_depth>0]), np.max(img_real_depth[img_real_depth<2**16-1]))

#		img_real_depth_copy = np.copy(img_real_depth)
#		img_real_depth_copy[img_real_depth_copy == 0] = 920
#		img_real_depth = img_real_depth_copy

		img_real_depth = np.clip(img_real_depth, 0, 925) # 858, 925
		img_real_depth_norm =  (img_real_depth- 0)/(img_real_depth.max())
		real_depth3 = utils.float_array_to_rgb_image(img_real_depth_norm, 2**24-1)  # PIL.Image format
		real_depth3_np =np.array(real_depth3)

		self.curr_data['overhead_depth'] = cv2.cvtColor(real_depth3_np, cv2.COLOR_RGB2BGR) # real_depth3_cv

	def overhead_camera_info_cb(self, msg):
		self.curr_data['overhead_camera_info'] = msg

	def joy_state_cb(self, msg): #joy state and previous state
		rospy.sleep(0.01)

		self.state['prev_joy_states'] = self.curr_data['joy_states'] #
		self.curr_data['joy_states'] = msg

#		print("self.curr_data['joy_states']", self.curr_data['joy_states'])

		if msg.buttons[6] == 1 and msg.buttons[7] == 0:
#		    self.clows(0) # open max, for robotiq
			self.axis.move_to(0) # for RM gripper
		elif msg.buttons[6] == 0 and msg.buttons[7] == 1:
#		    self.clows(255) #close, the value are recommended to be the same as those in gripper_state_cb for each task, for robotiq
			self.axis.push(9,31,20) # put_in_drawer; for RM gripper, force, distance, vel, self.axis.push(10,20,20) for insert_round_hole
		elif msg.buttons[6] == 0 and msg.buttons[7] == 0:
		    pass
		else:
		    rospy.loginfo('Invalid command for the gripper status, no action')
		    pass

		self.step()


	def joy_pose_cb(self, msg): # useless
		self.curr_data['joy_pose'] = msg
#       # may transform from euler angles to quaternion via tf.transformations.quaternion_from_euler, maybe not
#		self.curr_data['gripper_pose'] = self.get_tf(self.base_frame, self.ee_frame) #the x,y,z == get_pose from urscript, but the r,p,y didn't

	def joint_states_cb(self, msg):
		self.curr_data['joint_states'] = msg

	def target_pose_cb(self, msg): # useless
		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = self.base_frame
		pose_stamped.pose = msg.pose
		# TODO align current pose as the last frame's target pose for UR system
		self.curr_data['target_pose'] = pose_stamped
		self.state['new'] = True

	def gripper_state_cb(self, msg): # subscribe gripper status
		# for Robotiq Gripper
		'''
		gripper_s = [1.0, 0.0425, 0.0425] # 0.0425
		gripper_open_amount = 0
		if (msg.gPR<5 and msg.gOBJ==3) or msg.gOBJ==1:  # if (msg.gPO < 5 and msg.gOBJ==3) or msg.gOBJ==1:
			gripper_s[0] = 1.0						       # open   # position of fingers 0-255, 0 is open max
		elif (msg.gPR>250 and msg.gOBJ==3) or msg.gOBJ==2: # (msg.gPO > 228 and msg.gOBJ==3) or msg.gOBJ==2:
			gripper_s[0] = 0.0 #close
		else:
			pass

		gripper_open_amount = (255 - msg.gPO)/255*0.085 # gPR is requested position, gPO is actual position
		gripper_s[1] = gripper_s[2] = gripper_open_amount/2

		self.curr_data['gripper_states'] = gripper_s
		'''
		# RM Gripper
		gripper_s = [1.0, 0.03, 0.03] # 0.0425
		gripper_open_amount = 0.06- self.axis.position()/1000  # gPR is requested position, gPO is actual position
		if self.axis.position() < 0.2: # 0 for open, TODO
		    gripper_s[0] = 1.0 # open
		else:
		    gripper_s[0] = 0.0 #close

		gripper_s[1] = gripper_s[2] = gripper_open_amount/2

		self.curr_data['gripper_states'] = gripper_s
#		print("curr_data['gripper_states']", self.curr_data['gripper_states'] )

		# Get gripper pose. Method1 get_tf
		self.curr_data['gripper_pose'] = self.get_tf(self.base_frame, self.ee_frame)
		# Method2 gripper_pose_sub subscribe from the topic 'tcp_pose' publish by learning_joy, results is the same as get_pose()
		'''
		# Method3 get_pose()
		gripper_pose  = self.get_pose() # [mx, my, mz, rx, ry, rz]
		for i in range(3,6): #  re-range from [-pi, pi] to [0, 2pi] for agent input
		    	gripper_pose[i] = gripper_pose[i]+2*np.pi if gripper_pose[i] <0 else gripper_pose[i]
		#transform from euler angles to quaternion
#		quat = utils.discrete_euler_to_quaternion(np.array(gripper_pose[3:])*180/np.pi,1)
		quat = Rotation.from_euler('xyz', np.array(gripper_pose[3:])-np.pi, degrees=False).as_quat()

		pose_stamped = PoseStamped()
		pose_stamped.header.frame_id = self.base_frame
		pose_stamped.pose.position.x = gripper_pose[0]
		pose_stamped.pose.position.y = gripper_pose[1]
		pose_stamped.pose.position.z = gripper_pose[2]
		pose_stamped.pose.orientation.x = quat[0]
		pose_stamped.pose.orientation.y = quat[1]
		pose_stamped.pose.orientation.z = quat[2]
		pose_stamped.pose.orientation.w = quat[3]
		self.curr_data['gripper_pose'] = pose_stamped
		'''

		# get touch force

	def tcp_force_cb(self, msg):
		self.curr_data['tcp_force'] = msg


	'''
	Helper Funcs
	'''
	def get_tf(self, target_frame, source_frame): # it works for UR
		transform = self.tf2_buffer.lookup_transform(target_frame, # The frame that data should be transformed to
											      	source_frame, # The frame where the data originated from
    			    			    			    rospy.Time(0), #get the tf at first available time
    			    			    			    rospy.Duration(1.0)) #wait for 1 second before failing

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

	def pose_to_4x4mat(self, pose):
		basetrans = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
		baserot = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
		return np.matmul(basetrans, baserot)

#	def goto_pose(self, ee_pose): #TODO use move of URScript or moveit of UR instead; useless
#		ee_cmd = copy.deepcopy(ee_pose)
#
#		# weird Franka 45 deg shift
#		offset_45 = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(45.0))
#		target_ee_quat = [ee_cmd.pose.orientation.x,
#		    			    	  ee_cmd.pose.orientation.y,
#		    			    	  ee_cmd.pose.orientation.z,
#		    			    	  ee_cmd.pose.orientation.w]
#		rotated_target_ee_quat = tf.transformations.quaternion_multiply(target_ee_quat, offset_45)
#
#		# norm = np.linalg.norm(np.array(rotated_target_ee_quat), ord=2)
#		ee_cmd.pose.orientation.x = rotated_target_ee_quat[0]
#		ee_cmd.pose.orientation.y = rotated_target_ee_quat[1]
#		ee_cmd.pose.orientation.z = rotated_target_ee_quat[2]
#		ee_cmd.pose.orientation.w = rotated_target_ee_quat[3]
#
#		# self.controller.goto(ee_cmd)
#		succces = self._franka_goto(ee_cmd)

#	def get_pose(self):
#		data = self.s.recv(1220)  # 包的长度
#		while(len(data)<492):
#		    print('re-receive data of 1st arm, got ', len(data))
#		    data = self.s.recv(1220)  # 包的长度
#
#		mx = struct.unpack('!d', data[444:452])[0]
#		my = struct.unpack('!d', data[452:460])[0]
#		mz = struct.unpack('!d', data[460:468])[0]
#		rx = struct.unpack('!d', data[468:476])[0]
#		ry = struct.unpack('!d', data[476:484])[0]
#		rz = struct.unpack('!d', data[484:492])[0]
#		return [mx, my, mz, rx, ry, rz]



	'''
	Joystick Funcs
	'''
	def record_pose_cond(self): # use buttons on the joystick to control the recording,
		joy = self.curr_data['joy_states']
#		prev_joy = self.state['prev_joy_states']
#		print(joy.buttons[8])
		return (joy.buttons[8] == 1) # True or False according to conditions # joy.buttons[3] == 1 and joy.buttons[4] == 1

	def record_grasp_cond(self): #useless
		joy = self.curr_data['joy_states']
		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[1] > 0.8 or joy.axes[1] < -0.8)) and prev_joy.buttons[4] == 1

	def goto_last_keypoint_pose(self): #useless
		joy = self.curr_data['joy_states']
#		prev_joy = self.state['prev_joy_states']
		return (joy.buttons[2] == 1 and (joy.axes[2] < -0.8)) \
		    		and (joy.buttons[4] == 1)

	'''
	Main Funcs
	'''
	def record_goto(self): #
		start_idx = len(self.keypoint_data)
		rospy.loginfo("Recording now ...")
		while self.state['record']: #TODO
			self.keypoint_data.append(copy.deepcopy(self.curr_data)) # record curr_data as keypoint_data
			rospy.sleep(0.01)
		end_idx = len(self.keypoint_data) - 1
		self.keypoint_idxs.append(end_idx)
		rospy.loginfo(f"Recorded {end_idx - start_idx + 1} frames.")


	def record_grasp(self): #useless only record 1 frame, especially for gripper status change
		start_idx = len(self.keypoint_data)
		rospy.loginfo("Recording now ...")
		self.keypoint_data.append(copy.deepcopy(self.curr_data))
		end_idx = len(self.keypoint_data) - 1
		self.keypoint_idxs[-1] = end_idx
		rospy.loginfo(f"Recorded 1 frame.")


	def get_obs(self, frame, misc): # observations for collecting data and voxelization
		# TODO  change it back
#		print('frame', frame)
		finger_positions = np.array(frame['gripper_states'][-2:]) # subscribe gripper staus for finger_positions
		gripper_open = frame['gripper_states'][0]
		print('finger_positions',finger_positions)
		print('gripper_open',gripper_open)

		# current gripper pose for local voxelgrid. It's directly utilized when testing.
		# while the gripper_pose of the last observations will be automatically exploited in _add_keypoints_to_replay() of fusion_peract_bc/launch_utils.py when training.
		# Not required to explicitlly add previous or next gripper pose when collecting data
		gripper_pose = np.array([
	    	frame['gripper_pose'].pose.position.x,
	    	frame['gripper_pose'].pose.position.y,
	    	frame['gripper_pose'].pose.position.z,
	    	frame['gripper_pose'].pose.orientation.x,
	    	frame['gripper_pose'].pose.orientation.y,
	    	frame['gripper_pose'].pose.orientation.z,
	    	frame['gripper_pose'].pose.orientation.w,
		])


		tcp_force = np.array([
    		frame['tcp_force'].force.x,
    		frame['tcp_force'].force.y,
    		frame['tcp_force'].force.z,
    		frame['tcp_force'].torque.x,
    		frame['tcp_force'].torque.y,
    		frame['tcp_force'].torque.z,
    		])

		print('tcp_force', tcp_force)

		obs = Observation(
	    	left_shoulder_rgb=None,
	    	left_shoulder_depth=None,
	    	left_shoulder_point_cloud=None,
	    	right_shoulder_rgb=None,
	    	right_shoulder_depth=None,
	    	right_shoulder_point_cloud=None,
	    	overhead_rgb=None,
	    	overhead_depth=None,
	    	overhead_point_cloud=None,
	    	wrist_rgb=None,
	    	wrist_depth=None,
	    	wrist_point_cloud=None,
	    	front_rgb=None,
	    	front_depth=None,
	    	front_point_cloud=None,
	    	left_shoulder_mask=None,
	    	right_shoulder_mask=None,
	    	overhead_mask=None,
	    	wrist_mask=None,
	    	front_mask=None,
	    	joint_velocities=np.array(frame['joint_states'].velocity)[:6], # For key action discovery; None for Peract on RLBench
	    	joint_positions=np.array(frame['joint_states'].position)[:6], # in rad, with the order of elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
	    	joint_forces=np.array(frame['joint_states'].effort)[:6], # None for  Peract on RLBench
	    	gripper_open=gripper_open,# (1.0 if (gripper_open_amount > 0.0385 + 0.0385) else 0.0), # 0.334-0.394 for Franka Hand
	    	gripper_pose=gripper_pose, # TCP pose,
	    	gripper_matrix=self.pose_to_4x4mat(frame['gripper_pose'].pose), # useless
	    	gripper_touch_forces=tcp_force, #  filled in with TCP 6d force for FusionPerAct; None for Peract
	    	gripper_joint_positions=finger_positions,
	    	task_low_dim_state=None, # expandable for multi-modal FusionPerAct
	    	ignore_collisions=True, # TODO: fix
	    	misc=misc,
		)

#		rospy.loginfo(f"joint_velocities: {np.array(frame['joint_states'].velocity)[:6]}")
#		rospy.loginfo(f'gripper pose: {gripper_pose}')
#		rospy.loginfo(f"gripper_matrix: {self.pose_to_4x4mat(frame['gripper_pose'].pose)}")

		return obs


	def save_keypoint(self): # figure out the alignment between step() and save_keypoint()
		# make directories
		def check_and_mkdirs(dir_path):
			if not os.path.exists(dir_path):
				os.makedirs(dir_path, exist_ok=True)

		save_path = os.path.join(self.cfg['demo']['save_path'], self.cfg['demo']['task'])
		episode_idx = self.cfg['demo']['episode']
		variation_idx = self.cfg['demo']['variation']

		episode_path = os.path.join(save_path, 'all_variations', 'episodes', f"episode{episode_idx}")
		check_and_mkdirs(episode_path)

		front_rgb_path = os.path.join(episode_path, 'front_rgb')
		check_and_mkdirs(front_rgb_path)
		front_depth_path = os.path.join(episode_path, 'front_depth')
		check_and_mkdirs(front_depth_path)

		overhead_rgb_path = os.path.join(episode_path, 'overhead_rgb')
		check_and_mkdirs(overhead_rgb_path)
		overhead_depth_path = os.path.join(episode_path, 'overhead_depth')
		check_and_mkdirs(overhead_depth_path)

		# misc (camera_info etc)
		misc =dict()
		frame0 = self.keypoint_data[0] # curr_data is recorded as keypoint_data
		misc['front_camera_intrinsics'] = np.array(frame0['front_camera_info'].K).reshape(3,3)
		#TODO check if camera_color_optical_frame  is the right frame
		# $ rosrun tf tf_echo /base /camera_color_optical_frame; camera_color_optical_frame exists with parent camera_aligned_depth_to_color_frame;

		#TODO temporally comment out, solve the error: the transform tree bwtween the robot and camera are not connected
		misc['front_camera_extrinsics'] = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['front_camera']).pose) # transformation between base and cam
		misc['front_camera_near'] = self.cfg['demo']['front_cam_near']  #TODO 0.6, 0.5 for kinectv2 # operative measuring range
		misc['front_camera_far'] = self.cfg['demo']['front_cam_far'] # 6, https://www.intelrealsense.com/depth-camera-d455/ # 4.5 for kinect v2

		misc['overhead_camera_intrinsics'] = np.array(frame0['overhead_camera_info'].K).reshape(3,3)
		misc['overhead_camera_extrinsics'] = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['overhead_camera']).pose)
		misc['overhead_camera_near'] = self.cfg['demo']['overhead_cam_near']
		misc['overhead_camera_far'] = self.cfg['demo']['overhead_cam_far']

		#TODO may use a second cam for better performance
		misc['keypoint_idxs'] = np.array(list(self.keypoint_idxs))[1:]

		observations = []
		for f_idx, frame in enumerate(self.keypoint_data):
			save_idx = f_idx

			front_rgb = frame['front_rgb']
			front_depth = frame['front_depth']
			front_rgb_filename = os.path.join(front_rgb_path, f'{f_idx}.png')
			front_depth_filename = os.path.join(front_depth_path, f'{f_idx}.png')
			cv2.imwrite(front_rgb_filename, front_rgb)
			cv2.imwrite(front_depth_filename, front_depth)

			overhead_rgb = frame['overhead_rgb']
			overhead_depth = frame['overhead_depth']
			overhead_rgb_filename = os.path.join(overhead_rgb_path, f'{f_idx}.png')
			overhead_depth_filename = os.path.join(overhead_depth_path, f'{f_idx}.png')
			cv2.imwrite(overhead_rgb_filename, overhead_rgb)
			cv2.imwrite(overhead_depth_filename, overhead_depth)

			observations.append(self.get_obs(frame, misc))

		demo = Demo(observations, random_seed=self.cfg['demo']['random_seed']) # low dimentional data
		demo.variation_number = variation_idx

		low_dim_obs_path = os.path.join(episode_path, 'low_dim_obs.pkl')
		with open(low_dim_obs_path, 'wb') as f:
			pickle.dump(demo, f)

		variation_number_path = os.path.join(episode_path, 'variation_number.pkl')
		with open(variation_number_path, 'wb') as f:
			pickle.dump(variation_idx, f)

		descriptions = self.lang_goal.split(",")
		descriptions_path = os.path.join(episode_path, 'variation_descriptions.pkl')
		with open(descriptions_path, 'wb') as f:
			pickle.dump(descriptions, f)

		rospy.loginfo(f"Saved {len(self.keypoint_data)} frames to {save_path}")


	def undo_keypoint(self): # useless
		# import pdb; pdb.set_trace()
		if len(self.keypoint_data) > 0:
			self.keypoint_data = self.mp.list(self.keypoint_data[:self.keypoint_idxs[-2] + 1])
			self.keypoint_idxs = self.mp.list(self.keypoint_idxs[:-1])


	def step(self): #TODO dive in. For control both execution and saving when collect data

		#TODO the keypoint_discovery has been used in launch_utils.py

		# Start recording
		# start only if new target pose was received
		try:
			print('record_pose_cond', self.record_pose_cond())
			if self.record_pose_cond(): # record selected key steps use buttons on the joystick to control the recording
	    													  # we can also do it like this or not
				rospy.loginfo('Start collecting demos')

#				self.capture_mecheye_images()

	    		# TODOm use it or not, start recording when joint velocity are close to zero
#		    		joint_velocities=np.array(self.curr_data['joint_states'].velocity)[:6]
#		    		while np.any(joint_velocities > 0.008):
#		    			joint_velocities=np.array(self.curr_data['joint_states'].velocity)[:6]
#		    			rospy.loginfo(f"Waiting for joint velocities decreasing to a small value")

			# TODO self.state['prev_pose']=current pose instead of target pose
	    		# go to last recorded pose
#		    	prev_pose = self.state['prev_pose']
#		    	if self.cfg['settings']['replay_from_prev_pose']: # False in defult
#		    		self.goto_pose(prev_pose)

				self.state['record'] = True
				self.state['keypoint_done'] = False

				rec_process = Process(target=self.record_goto, args=()) # record but not save
				rec_process.start() # start recording
				rospy.sleep(0.01)
#		    	target_pose = self.curr_data['target_pose']
#		    	self.goto_pose(target_pose)

			# ------move via joystick control-----
			# ------change gripper status via joystick control------

				self.state['record'] = False # flags to stop recording
				self.state['keypoint_done'] = True
				rec_process.join() # block the main process (rather than sub process) until rec_process finished

#		    	self.state['new'] = False

	    		# ask to save # default == False and use the joystick to control recording
				if self.cfg['settings']['ask_to_save']: #TODO Ask to save to avoid saving too much redundant key actions, or use joystick buttons
					resp = input('Save keypoint trajectory? (y/n)\n')
					if resp == 'y' or 'Y':
						self.save_keypoint() # save recorded
#		    			self.state['prev_pose'] = target_pose
#		    		else:
#		    			self.undo_keypoint()
				else:
					self.save_keypoint()
#		    		self.state['prev_pose'] = target_pose

	    	# undo pose, to improve the program robustness
#		if self.goto_last_keypoint_pose():
#		    	prev_pose = self.state['prev_pose']
#		    	self.goto_pose(prev_pose)

	    	# record grasp change, seperated from record (arm 6D) pose change
#		if self.record_grasp_cond():
#
#		    	self.state['record'] = True
#		    	self.state['keypoint_done'] = False
#
#		    	ax = self.curr_data['joy_states'].axes[1]
#		    	gripper_state = 1.0 if ax > 0.8 else 0.0
#		    	self._franka_set_gripper(gripper_state) # open gripper
#		    	self.record_grasp() # change the gripper status at first then record
#
#		    	if self.cfg['settings']['ask_to_save']:
#		    		resp = input('Save keypoint trajectory? (y/n)\n')
#		    		if resp == 'y':
#		    			self.save_keypoint()
#		    		else:
#		    			self.undo_keypoint()
#		    	else:
#		    		self.save_keypoint()
		except KeyboardInterrupt:
			rospy.loginfo("Shutting down demo interface.")

	def keyboard_interrupt_handler(signal, frame):
		rospy.signal_shutdown("Keyboard interrupt detected")

    # For Roboitiq gripper

#	def clows(self, d):
#
#		self.command.rPR = d
#		self.gripper_state_pub.publish(self.command)
##		rospy.sleep(0.1)
#
#	def reset(self):
#		self.command.rACT = 0
#		self.gripper_state_pub.publish(self.command)
##		rospy.sleep(0.1)
#
#	def active(self):
#		self.command.rACT = 1
#		self.command.rGTO = 1
#		self.command.rSP  = 128
#		self.command.rFR  = 128
#		self.gripper_state_pub.publish(self.command)
##		rospy.sleep(0.1)



@hydra.main(config_path="../cfgs", config_name="peract_demo")
def main(cfg):
	# initialize(config_path="../cfgs", job_name="peract_demo")
	# cfg = compose(config_name="peract_demo")
	pprint.pprint(dict(cfg)) # pretty print

	save_path = os.path.join(cfg['demo']['save_path'], cfg['demo']['task'])
	episode_idx = cfg['demo']['episode']
	variation_idx = cfg['demo']['variation']
	episode_path = os.path.join(save_path, 'all_variations', 'episodes', f"episode{episode_idx}")
	if os.path.exists(episode_path):
		resp = input(f"{cfg['demo']['task']} - Episode {episode_idx} already exists. Overwrite? (y/n)\n")
		if resp == 'y':
			shutil.rmtree(episode_path)
		else:
			sys.exit()

	rospy.init_node('peract_demo', anonymous=True)
#	rospy.wait_for_service('/capture_color_map')  # for mecheye images
#	rospy.wait_for_service('/capture_depth_map')
#	rospy.wait_for_service('/capture_point_cloud')
	interface = PeractDemoInterface(cfg)
	rospy.sleep(0.1)

#	while not rospy.is_shutdown(): #TODO in a loop or use rospy.spin() subscriber
#		try:
#
#		    	print('start steps in the while loop')
#		    	interface.step()
#		    	interface.loop_rate.sleep() # 10Hz in default
#
#		except KeyboardInterrupt:
#		    	rospy.loginfo("Shutting down demo interface.")



if __name__ == '__main__':
	main()
