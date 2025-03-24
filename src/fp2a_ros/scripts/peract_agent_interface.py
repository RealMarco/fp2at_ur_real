#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep 16 18:24:15 2023

@author: marco
"""
import os
import sys
ARM_PATH = '/home/mshr/cliport_ndp/cliport_ndp'
sys.path.append(ARM_PATH)

import numpy as np
import pprint
#import shutil
import pickle

import cv2
import rospy
import copy
import tf
import tf2_ros
#import tf2_geometry_msgs
#import threading
from multiprocessing import  Manager, Process
import torch
#torch.set_default_dtype(torch.float32) #for RVT3

import hydra
#from hydra import compose, initialize
from omegaconf import OmegaConf

from cv_bridge import CvBridge #, CvBridgeError
from geometry_msgs.msg import PoseStamped, Wrench # Pose,
from sensor_msgs.msg import Joy, Image, CameraInfo, JointState
from visualization_msgs.msg import Marker

#from dvnets_franka_msgs.srv import GotoPose, SetGripper # , Home
#from rlbench.backend.observation import Observation
#from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.demo import Demo
from pyrep.objects import VisionSensor
from rlbench.backend.utils import image_to_float_array, float_array_to_rgb_image #, rgb_handles_to_mask

#from arm import franka_c2farm_perceiver_lang_bc  #TODO change to FP
#from arm.clip.core.clip import tokenize
#from arm import utils

import socket
import struct
import threading
from agents import fusion_peract_bc  # pip install -e (fusion) peract at first
from helpers.clip.core.clip import tokenize # build_model, load_clip,
from helpers import utils
#from scipy.spatial.transform import Rotation
from fp2a_ros.msg import _Robotiq2FGripper_robot_output as outputMsg # Robotiq2FGripper_robot_output
from fp2a_ros.msg import _Robotiq2FGripper_robot_input as inputMsg
import moveit_commander
#from mecheye_ros_interface.srv import CaptureColorMap, CaptureDepthMap, CaptureColorPointCloud
#import moveit_msgs.msg

# for RM gripper
import motormaster
# for RVT3
#import rvt.models.rvt_agent_real as rvt_agent
#from rvt.mvt.mvt_real import MVT
from rvt.eval_real import load_agent


HOST= "172.20.172.102"
PORT = 30003  # socket


class PeractAgentInterface:

	def __init__(self, cfg):

#	    self.s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#	    self.s.connect((HOST,PORT))
#	    urscript = "set_tcp(p[{}, {}, {}, {}, {}, {}])\n".format(0, 0, 0.260, 0, 0, 0)
#	    self.s.send(urscript.encode('utf8'))
#	    rospy.sleep(1)

	    self.lock = threading.Lock()

	    self.cfg = cfg

	    # setup
	    self.loop_rate = rospy.Rate(cfg['ros']['loop_rate'])
	    self.base_frame =  self.cfg['frame']['base'] # 'base'  #  'base_link'
	    self.ee_frame = self.cfg['frame']['end_effector'] #  'tool0'

	    # MoveIt Planner
	    moveit_commander.roscpp_initialize(sys.argv) # Initialize MoveIt commander
	    # Instantiate a RobotCommander object to interface with the robot
	    self.robot = moveit_commander.RobotCommander() # get information about the robot's current state, such as joint positions and joint limits
	    # Instantiate a PlanningSceneInterface object to interact with the world
	    self.scene = moveit_commander.PlanningSceneInterface() # represents the environment includes information about objects, their positions, and collision detection
	    # Instantiate a MoveGroupCommander object to plan and execute motions
	    self.group_name = "manipulator"
	    self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
	    # Set the reference frame for motion planning
	    self.move_group.set_pose_reference_frame(self.base_frame)
	    # Set the end-effector link
	    self.move_group.set_end_effector_link(self.ee_frame)
	    self.move_group.set_max_velocity_scaling_factor(0.01)  # Set maximum velocity scaling factor to 1% of the default
	    self.move_group.set_max_acceleration_scaling_factor(0.01)
#	    print("get_goal_orientation_tolerance",self.move_group.get_goal_orientation_tolerance()) # 0.001 rad
#	    print("get_goal_position_tolerance:", self.move_group.get_goal_position_tolerance()) # 0.0001m

	    # initial pose
	    action = (-0.07486150, -0.202575, 0.27085,-0.369427, -0.929156, 0.01345, 0.003359)  # initail pose
	    self.initial_pose = PoseStamped()
	    self.initial_pose.header.frame_id = self.base_frame
	    self.initial_pose.pose.position.x = action[0]
	    self.initial_pose.pose.position.y = action[1]
	    self.initial_pose.pose.position.z = action[2]
	    self.initial_pose.pose.orientation.x = action[3]
	    self.initial_pose.pose.orientation.y = action[4]
	    self.initial_pose.pose.orientation.z = action[5]
	    self.initial_pose.pose.orientation.w = action[6]

	    # tools
	    self.cv_bridge = CvBridge()
	    self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
	    self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
	    self.mp = Manager()

	    # data topics
	    self.curr_data = self.mp.dict({
	    	'front_rgb': None,
	    	'front_depth': None,
	    	'front_camera_info': None,

	    	'overhead_rgb': None,
	    	'overhead_depth': None,
	    	'overhead_camera_info': None,

	    	'joy_states': Joy(), #None,
	    	'joy_pose': None,

	    	'joint_states': None,  #
	    	'target_pose': None,
	    	'gripper_pose':self.get_tf(self.base_frame, self.ee_frame), # TCP 6D pose, position+quaternion
	    	'gripper_states': [1.0, 0.03, 0.03], # None for robotiq
	    	'tcp_force': None,
	    })

	    # states
	    self.state = self.mp.dict({
	    	'step': 0,
	    	'prev_joy_states': Joy(), # None,
	    	'prev_pose':self.get_tf(self.base_frame, self.ee_frame),# PoseStamped(),
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

	    ''' #for robotiq
	    self.gripper_state_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
	    self.command = outputMsg.Robotiq2FGripper_robot_output()
	    self.active() # activate gripper
	    '''

	    # RM gripper
	    self.axis = motormaster.create_axis_modbus_rtu('/dev/ttyUSB3', 115200, 0)
	    self.axis.go_home()

	    self.joy_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.joy_state_cb)
#	    self.joy_pose_sub = rospy.Subscriber(self.cfg['topics']['joy_pose'], PoseStamped, self.joy_pose_cb)

	    #for robotiq
#	    self.gripper_state_sub = rospy.Subscriber(self.cfg['topics']['gripper_state'], inputMsg.Robotiq2FGripper_robot_input, self.gripper_state_cb)
	    # for RM gripper
	    self.gripper_state_sub = rospy.Subscriber(self.cfg['topics']['joy_state'], Joy, self.gripper_state_cb)

	    self.joint_states_sub = rospy.Subscriber(self.cfg['topics']['joint_states'], JointState, self.joint_states_cb) #  joint angles in radians of the robot. represent the rotation from its zero position
#	    self.target_pose_sub = rospy.Subscriber(self.cfg['topics']['target_pose'], Marker, self.target_pose_cb)
	    self.tcp_force_sub = rospy.Subscriber(self.cfg['topics']['tcp_force'], Wrench, self.tcp_force_cb)

	    self.pred_voxel_pub = rospy.Publisher(self.cfg['topics']['pred_voxel'], Image, latch=True, queue_size=10)

	    # visualization   #TODO visualize the end-effector or not
#	    self.peract_action_marker = self.get_ee_marker("package://franka_description/meshes/visual/hand.dae", [1.0, 0.0, 0.0, 0.6])
#	    self.peract_action_pub = rospy.Publisher('peract_action', Marker, queue_size=10)

	    # controller #TODO, ur move
#	    rospy.wait_for_service('franka_goto_pose')
#	    self._franka_goto = rospy.ServiceProxy('franka_goto_pose', GotoPose)
#	    rospy.wait_for_service('franka_set_gripper')
#	    self._franka_set_gripper = rospy.ServiceProxy('franka_set_gripper', SetGripper)

	    # agent
	    self.device = torch.device('cuda:0')
	    self._load_agent()
	    self.act_result = None

	    # language
	    self.lang_goal = input("Language Goal: ")

#	    self.capture_mecheye_images()

	    rospy.spin()  #TODO


	def _load_agent(self):
	    # for fp2at
	    '''
	    # load config # TODO comfirm the parameters
#	    scene_bounds = self.cfg['agent']['scene_bounds']
#	    camera_resolution = self.cfg['agent']['camera_resolution']

	    # cfg = OmegaConf.create(OmegaConf.to_yaml(conf, resolve=True))
	    cfg_path = os.path.join(self.cfg['agent']['seed_path'], 'config.yaml')
	    cfg = OmegaConf.load(cfg_path)
	    self._rotation_resolution = cfg.method.rotation_resolution
	    print('self._rotation_resolution', self._rotation_resolution)
	    # cfg['method']['num_latents'] = 256
	    print(cfg) # TODO check if the scene_bounds is correct

	    # load agent, may use agents.peract_bc.launch_utils.create_agent instead
#	    self.agent = franka_c2farm_perceiver_lang_bc.launch_utils.create_agent(
#	    	cfg, None, depth_0bounds=scene_bounds, cam_resolution=camera_resolution)

	    self.agent = fusion_peract_bc.launch_utils.create_agent(cfg)
	    self.agent.build(training=False, device=self.device)  #TODO, use it or not

	    # load pre-trained weights
	    weights_path = os.path.join(self.cfg['agent']['seed_path'], 'weights',
	    	    	    	    	str(self.cfg['agent']['weight']))
	    self.agent.load_weights(weights_path)

	    '''

	    # for RVT3
#	    rvt = MVT(
#	    	    renderer_device=device,
#	    	    **mvt_cfg,
#		    )
#	    # detect_model = get_detect_model(device)
#	    agent = rvt_agent.RVTAgent(
#	    	    # detect_model=detect_model.eval(),
#	    	    network=rvt.to(device),
#	    	    image_resolution=[IMAGE_SIZE, IMAGE_SIZE],
#	    	    add_lang=mvt_cfg.add_lang,
#	    	    stage_two=mvt_cfg.stage_two,
#	    	    rot_ver=mvt_cfg.rot_ver,
#	    	    scene_bounds=SCENE_BOUNDS,
#	    	    cameras=CAMERAS,
#	    	    log_dir=f"{eval_log_dir}/eval_run",
#	    	    **exp_cfg.peract,
#	    	    **exp_cfg.rvt,
#		    )
	    # for RVT3
	    weights_path="/media/marco/ubuntu_data/RVT3/rvt/runs/rvt2_E_199_PA.transform_augmentation_T_bs_32_rvt.place_with_mean_T_ST_F/model_last.pth"
	    self.agent = load_agent(
	    	    model_path=weights_path,
	    	    exp_cfg_path=None,
	    	    mvt_cfg_path=None,
	    	    eval_log_dir=None,
	    	    device='0',
	    	    use_input_place_with_mean=False,
		    )
	    self.agent.eval()
	    self.agent.load_clip()


	    print("Loaded: " + weights_path)

	'''
	Callbacks
	'''

#	def capture_mecheye_images(self):
#	    try:
#		    capture_color_map_service = rospy.ServiceProxy('/capture_color_map', CaptureColorMap)
#		    capture_depth_map_service = rospy.ServiceProxy('/capture_depth_map', CaptureDepthMap)
#		    capture_color_map_service()
#		    capture_depth_map_service()
#	#		    if response.errorCode ==0:
#	#		    	rospy.loginfo(f'call for the service /capture_color_map successfully')
#	#		    else:
#	#		    	rospy.loginfo(f'Error: call for the service /capture_color_map UNsuccessfully')
#	    except rospy.ServiceException as e:
#		    print("Service call failed:", str(e))

	def front_rgb_cb(self, msg):
	    self.curr_data['front_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8") # TODO why not bgr8 as demo_interface # for read

	def front_depth_cb(self, msg):
#	    self.curr_data['front_depth'] = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")

	    img_real_depth = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
	    img_real_depth = np.clip(img_real_depth, 0, 1088) #

	    img_real_depth_norm =  (img_real_depth- 0)/(img_real_depth.max())
	    self.curr_data['front_depth']=img_real_depth_norm
#	    real_depth3 = float_array_to_rgb_image(img_real_depth_norm, 2**24-1)  # PIL.Image format
#	    real_depth3_np =np.array(real_depth3)

#	    self.curr_data['front_depth']  = cv2.cvtColor(real_depth3_np, cv2.COLOR_RGB2BGR) # real_depth3_cv


	def front_camera_info_cb(self, msg):
	    self.curr_data['front_camera_info'] = msg

	def overhead_rgb_cb(self, msg):
	    self.curr_data['overhead_rgb'] = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")

	def overhead_depth_cb(self, msg):
	    img_real_depth = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")

	    img_real_depth = np.clip(img_real_depth, 0, 925) #
	    img_real_depth_norm =  (img_real_depth- 0)/(img_real_depth.max())
	    self.curr_data['overhead_depth']=img_real_depth_norm
#	    real_depth3 = float_array_to_rgb_image(img_real_depth_norm, 2**24-1)  # PIL.Image format
#	    real_depth3_np =np.array(real_depth3)

#	    self.curr_data['overhead_depth'] = cv2.cvtColor(real_depth3_np, cv2.COLOR_RGB2BGR) # real_depth3_cv

	def overhead_camera_info_cb(self, msg):
	    self.curr_data['overhead_camera_info'] = msg

	def joy_state_cb(self, msg):
	    rospy.sleep(0.01)

	    self.state['prev_joy_states'] = self.curr_data['joy_states']
	    self.curr_data['joy_states'] = msg


#	    if msg.buttons[6] == 1 and msg.buttons[7] == 0:
#	    	self.clows(0) # open max
#	    elif msg.buttons[6] == 0 and msg.buttons[7] == 1:
#	    	self.clows(255) #close, the value are recommended to be the same as those in gripper_state_cb for each task
#	    elif msg.buttons[6] == 0 and msg.buttons[7] == 0:
#	    	pass
#	    else:
#	    	rospy.loginfo('Invalid command for the gripper status, no action')
#	    	pass

	    #TODO check if step() in joy_state_cb() + rospy.spin() in __init__() or while loop in main function works
	    self.step()

	def joy_pose_cb(self, msg):
	    self.curr_data['joy_pose'] = msg
#	    self.curr_data['gripper_pose'] = self.get_tf(self.base_frame, self.ee_frame)

	def joint_states_cb(self, msg):
	    self.curr_data['joint_states'] = msg

	def target_pose_cb(self, msg):
	    pose_stamped = PoseStamped()
	    pose_stamped.header.frame_id = self.base_frame
	    pose_stamped.pose = msg.pose

	    self.curr_data['target_pose'] = pose_stamped
	    self.state['new'] = True

	def gripper_state_cb(self, msg): # subscribe gripper status
	    # for Robotiq Gripper
	    '''
	    gripper_s = [1.0, 0.0425, 0.0425] # 0.0425
	    gripper_open_amount = 0
	    if (msg.gPR<5 and msg.gOBJ==3) or msg.gOBJ==1:  # if (msg.gPO < 5 and msg.gOBJ==3) or msg.gOBJ==1:
	    	gripper_s[0] = 1.0	    	    		       # open   # position of fingers 0-255, 0 is open max
	    elif (msg.gPR>250 and msg.gOBJ==3) or msg.gOBJ==2: # (msg.gPO > 228 and msg.gOBJ==3) or msg.gOBJ==2:
	    	gripper_s[0] = 0.0 #close
	    else:
	    	pass

	    gripper_open_amount = (255 - msg.gPO)/255*0.085 # gPR is requested position, gPO is actual position
	    gripper_s[1] = gripper_s[2] = gripper_open_amount/2

	    self.curr_data['gripper_states']	 = gripper_s
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

	    # Get gripper pose. Method1 get_tf
	    self.curr_data['gripper_pose'] = self.get_tf(self.base_frame, self.ee_frame)
	    # Method2 gripper_pose_sub subscribe from the topic 'tcp_pose' publish by learning_joy, results is the same as get_pose()
	    '''
	    # Method3 get_pose()
	    gripper_pose  = self.get_pose() # [mx, my, mz, rx, ry, rz]
	    for i in range(3,6): #  re-range from [-pi, pi] to [0, 2pi] for agent input
	    	gripper_pose[i] = gripper_pose[i]+2*np.pi if gripper_pose[i] <0 else gripper_pose[i]
	    #transform from euler angles to quaternion
#	    quat = utils.discrete_euler_to_quaternion(np.array(gripper_pose[3:])*180/np.pi,1)
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
	def get_ee_marker(self, mesh_resource, color): # end-effector marker
	    marker = Marker()

	    marker = Marker()
	    marker.header.frame_id = self.base_frame
	    marker.header.stamp  = rospy.get_rostime()
	    marker.ns = "robot"
	    marker.id = 0
	    marker.type = 10 # mesh
	    marker.mesh_resource = mesh_resource
	    marker.action = 0
	    marker.scale.x = 1.0
	    marker.scale.y = 1.0
	    marker.scale.z = 1.0

	    marker.color.r = color[0]
	    marker.color.g = color[1]
	    marker.color.b = color[2]
	    marker.color.a = color[3]

	    return marker

	def get_tf(self, target_frame, source_frame):
	    transform = self.tf2_buffer.lookup_transform(target_frame,
	    	    	    	    	    	    	 source_frame,
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

	def pose_to_4x4mat(self, pose):
	    basetrans = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))
	    baserot = tf.transformations.quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
	    return np.matmul(basetrans, baserot)

#	def get_pose(self):
#	    data = self.s.recv(1220)  # 包的长度
#	    while(len(data)<492):
#		    print('re-receive data of 1st arm, got ', len(data))
#		    data = self.s.recv(1220)  # 包的长度
#
#	    mx = struct.unpack('!d', data[444:452])[0]
#	    my = struct.unpack('!d', data[452:460])[0]
#	    mz = struct.unpack('!d', data[460:468])[0]
#	    rx = struct.unpack('!d', data[468:476])[0]
#	    ry = struct.unpack('!d', data[476:484])[0]
#	    rz = struct.unpack('!d', data[484:492])[0]
#	    return [mx, my, mz, rx, ry, rz]

#	def goto_pose(self, ee_pose): # For Franka
#	    ee_cmd = copy.deepcopy(ee_pose)
#
#	    # weird Franka 45 deg shift
#	    offset_45 = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(45.0))
#	    target_ee_quat = [ee_cmd.pose.orientation.x,
#	    	    	      ee_cmd.pose.orientation.y,
#	    	    	      ee_cmd.pose.orientation.z,
#	    	    	      ee_cmd.pose.orientation.w]
#	    rotated_target_ee_quat = tf.transformations.quaternion_multiply(target_ee_quat, offset_45)
#
#	    # norm = np.linalg.norm(np.array(rotated_target_ee_quat), ord=2)
#	    ee_cmd.pose.orientation.x = rotated_target_ee_quat[0]
#	    ee_cmd.pose.orientation.y = rotated_target_ee_quat[1]
#	    ee_cmd.pose.orientation.z = rotated_target_ee_quat[2]
#	    ee_cmd.pose.orientation.w = rotated_target_ee_quat[3]
#
#	    # self.controller.goto(ee_cmd)
#	    succces = self._franka_goto(ee_cmd)

	def goto_pose(self, target_pose):
	    # Set the start state
	    waypoints= []
#	    current_pose =  self.move_group.get_current_pose() # in the format of PoseStamped() w.r.t. base_link
	    # Plan the motion
#	    waypoints.append(copy.deepcopy(current_pose.pose))
	    waypoints.append(copy.deepcopy(target_pose))
	    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  #,avoid_collisions = True, path_constraints=None, eef_step 0.01m, 0.005m  # jump_threshold 0.0
	    # move_group.plan and move_group.go are for joint motion planning

	    # Execute the motion
	    self.move_group.execute(plan, wait=True)

	    # Clear targets
	    self.move_group.clear_pose_targets()


	def shutdown_planner(self):
	    moveit_commander.roscpp_shutdown() # Shutdown MoveIt commander

	# control by socket will easily fail to sigularity;  use moveit of UR instead

	'''
	# For UR, via URscripts
	def goto_pose(self, target, a = 0.3, v = 0.08):    # absolute target(x, y, z, r, p, q); method 'j' is movej

	    trans = np.array([target.pose.position.x, target.pose.position.y, target.pose.position.z])
	    quat = np.array([
	    	    target.pose.orientation.x,
	    	    target.pose.orientation.y,
	    	    target.pose.orientation.z,
	    	    target.pose.orientation.w])
	    #TODO keep it or not; pre-process from 	_get_action() in launch_utils.py
	    quat = np.array(quat) / np.linalg.norm(quat, axis=-1, keepdims=True) # equivalent to utils.normalize_quaternion(quat)
	    if quat[-1] < 0: # maybe useless
	    	quat = -quat

	    # transform from quaternion to euler angles
#	    rot = utils.quaternion_to_discrete_euler(quat,1)/180 * np.pi # precison loss due to discrete value, inevitable for out net configurations
#	    for i in range(0,3):
#	    	rot[i] = rot[i]-2*np.pi if rot[i] > np.pi else rot[i]
	    rot = Rotation.from_quat(quat).as_euler('xyz', degrees=False) + np.pi # in (0,2pi)
	    rot[rot>np.pi] = rot[rot>np.pi] -2*np.pi #  re-range from [0, 2pi] to [-pi, pi] for execution
	    assert np.min(rot) >= -np.pi and np.max(rot) <= np.pi

	    [mx, my, mz, rx, ry, rz] = self.get_pose() # curent gripper (tcp) pose
	    with self.lock:
	    	# move rotations at first
	    	urscript = "movej(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(mx, my, mz, rot[0], rot[1], rot[2], a, v) #  joint planning movement, fatest
	    	self.s.send(urscript.encode('utf8'))
#	    	rospy.sleep(1) # waiting for the rotating finished,
	    	# then move positions
	    	urscript = "movel(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], a, v) #straight line movement
	#	    urscript = "movep(p[{}, {}, {}, {}, {}, {}],a={},v={})\n".format(target[0], target[1], target[2], target[3], target[4], target[5], a, v) # uniform-speed movement of end-effector
	    	self.s.send(urscript.encode('utf8'))
	'''


	'''
	Joystick Funcs
	'''

	def new_lang_cond(self): # set a new language condition
	    joy = self.curr_data['joy_states']
#	    prev_joy = self.state['prev_joy_states']
	    return (joy.buttons[6] == 1 and joy.buttons[7] == 1)

	def next_cond(self): # visualize next action
	    joy = self.curr_data['joy_states']
#	    prev_joy = self.state['prev_joy_states']
	    return (joy.buttons[6] == 1 and joy.buttons[7] == 0)

	def execute_cond(self): # execute the next action
	    joy = self.curr_data['joy_states']
#	    prev_joy = self.state['prev_joy_states']
	    return (joy.buttons[6] == 0 and joy.buttons[7] == 1)

	def record_pose_cond(self): # use buttons on the joystick to control the recording,
	    joy = self.curr_data['joy_states']
#	    prev_joy = self.state['prev_joy_states']
#	    print(joy.buttons[8])
	    return (joy.buttons[8] == 1)

	def record_goto(self): #
	    start_idx = len(self.keypoint_data)
	    rospy.loginfo("Recording now ...")
	    while self.state['record']: #TODO
	    	self.keypoint_data.append(copy.deepcopy(self.curr_data)) # record curr_data as keypoint_data
	    	rospy.sleep(0.01)
	    end_idx = len(self.keypoint_data) - 1
	    self.keypoint_idxs.append(end_idx)
	    rospy.loginfo(f"Recorded {end_idx - start_idx + 1} frames.")


	'''
	Main Funcs
	'''
	def get_obs(self):  # Real camera observations
	    obs = {}

	    # front_camera. # obs['%s_camera_extrinsics' % camera_name], camera_name is from [front,left_shoulder,right_shoulder,overhead]
	    front_camera_intrinsics = np.array(self.curr_data['front_camera_info'].K).reshape(3,3)
	    obs['front_camera_intrinsics'] = torch.tensor([front_camera_intrinsics], device=self.device).unsqueeze(0)

	    front_camera_extrinsics = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['front_camera']).pose) # kinect_front_link
	    obs['front_camera_extrinsics'] = torch.tensor([front_camera_extrinsics], device=self.device).unsqueeze(0)

	    front_rgb = torch.tensor([copy.deepcopy(self.curr_data['front_rgb'])], device=self.device)
	    front_rgb = front_rgb.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0) # TODO figure out
	    obs['front_rgb'] = front_rgb #torch.flip(front_rgb, dims=[2])

	    front_depth = copy.deepcopy(self.curr_data['front_depth'])
#	    front_depth = image_to_float_array(front_depth, 2**24 - 1) # TODO check the corectness, refer to rlbench/utils.py # 1000.0

	    front_near = self.cfg['agent']['front_cam_near']   # TODO transfer to depth in meters or not
	    front_far = self.cfg['agent']['front_cam_far']
	    front_depth = front_near + front_depth * (front_far - front_near)
	    print('front_depth', front_depth,front_depth.shape)
	    print('front_depth', front_depth[240, 320],front_depth.shape)

#	    obs[i].overhead_depth = obs_config.overhead_camera.depth_noise.apply(d) #TODO apply noise or not

	    front_point_cloud = VisionSensor.pointcloud_from_depth_and_camera_params(
	    	    	    	    	    	    	    	front_depth,
	    	    	    	    	    	    	    	front_camera_extrinsics,
	    	    	    	    	    	    	    	front_camera_intrinsics)
#	    front_point_cloud = torch.tensor([front_point_cloud], device=self.device) # for FP2AT
	    front_point_cloud = torch.tensor([front_point_cloud], device=self.device, dtype=torch.float32) # for RVT3
	    front_point_cloud = front_point_cloud.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0)
	    obs['front_point_cloud'] = front_point_cloud  #
	    # TODO maybe visualize the voxelgrid in step() for validation


#
	    # TODO overhead camera observation
	    overhead_camera_intrinsics = np.array(self.curr_data['overhead_camera_info'].K).reshape(3,3)
	    obs['overhead_camera_intrinsics'] = torch.tensor([overhead_camera_intrinsics], device=self.device).unsqueeze(0)

	    overhead_camera_extrinsics = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['overhead_camera']).pose) # kinect_overhead_link
	    obs['overhead_camera_extrinsics'] = torch.tensor([overhead_camera_extrinsics], device=self.device).unsqueeze(0)

	    overhead_rgb = torch.tensor([copy.deepcopy(self.curr_data['overhead_rgb'])], device=self.device)
	    overhead_rgb = overhead_rgb.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0) # TODO figure out
	    obs['overhead_rgb'] = overhead_rgb #torch.flip(overhead_rgb, dims=[2])

	    overhead_depth = copy.deepcopy(self.curr_data['overhead_depth'])
#	    overhead_depth = image_to_float_array(overhead_depth, 2**24 - 1) # 1000.0

	    oh_near = self.cfg['agent']['overhead_cam_near']   # TODO transfer to depth in meters or not
	    oh_far = self.cfg['agent']['overhead_cam_far']
	    overhead_depth = oh_near + overhead_depth * (oh_far - oh_near)

	    overhead_point_cloud = VisionSensor.pointcloud_from_depth_and_camera_params(
	    	    	    	    	    	    	    	overhead_depth,
	    	    	    	    	    	    	    	overhead_camera_extrinsics,
	    	    	    	    	    	    	    	overhead_camera_intrinsics)
#	    overhead_point_cloud = torch.tensor([overhead_point_cloud], device=self.device) # for FP2AT
	    overhead_point_cloud = torch.tensor([overhead_point_cloud], device=self.device, dtype=torch.float32) # for RVT3
	    overhead_point_cloud = overhead_point_cloud.permute(0, 3, 1, 2).permute(0, 1, 3, 2).unsqueeze(0)
#	    print('overhead_point_cloud', overhead_point_cloud.dtype) # torch.float64
	    obs['overhead_point_cloud'] = overhead_point_cloud  #

	    # collision, oberservations
	    obs['ignore_collisions'] = torch.tensor([[[1.0]]], device=self.device)

	    # language
	    lang_goal_tokens = tokenize([self.lang_goal])[0].numpy()
	    lang_goal_tokens = torch.tensor([lang_goal_tokens], device=self.device).unsqueeze(0)
	    obs['lang_goal_tokens'] = lang_goal_tokens

	    # proprioception for Franka Panda: gripper, timestep
#	    finger_positions = np.array(self.curr_data['joint_states'].position)[-2:]
#	    gripper_open_amount = finger_positions[0] + finger_positions[1]
#	    gripper_open = (1.0 if (gripper_open_amount > 0.0385 + 0.0385) else 0.0)

	    # proprioception for UR
	    finger_positions =  np.array(self.curr_data['gripper_states'][-2:]) # subscribe gripper staus for finger_positions
	    gripper_open = self.curr_data['gripper_states'][0]

	    gripper_pose = np.array([
	    	self.curr_data['gripper_pose'].pose.position.x,
	    	self.curr_data['gripper_pose'].pose.position.y,
	    	self.curr_data['gripper_pose'].pose.position.z,
	    	self.curr_data['gripper_pose'].pose.orientation.x,
	    	self.curr_data['gripper_pose'].pose.orientation.y,
	    	self.curr_data['gripper_pose'].pose.orientation.z,
	    	self.curr_data['gripper_pose'].pose.orientation.w,
	    ])
	    obs['gripper_pose']=torch.tensor([[[gripper_pose]]], device=self.device)  # refer to _extract_obs() in yarr/envs/rlbench_env.py; for local voxelization
	    tcp_force = np.array([
	    	    self.curr_data['tcp_force'].force.x,
	    	    self.curr_data['tcp_force'].force.y,
	    	    self.curr_data['tcp_force'].force.z,
	    	    self.curr_data['tcp_force'].torque.x,
	    	    self.curr_data['tcp_force'].torque.y,
	    	    self.curr_data['tcp_force'].torque.z,
	    	    ])

	    time = (1. - (self.state['step'] / float(self.cfg['agent']['episode_length'] - 1))) * 2. - 1. #TODO

	    # for FP2AT, 30+ low_dim_state
#	    joint_velocities=np.array(self.curr_data['joint_states'].velocity)[:6]
#	    joint_positions=np.array(self.curr_data['joint_states'].position)[:6]
#	    joint_forces=np.array(self.curr_data['joint_states'].effort)[:6]
#	    low_dim_np =  np.concatenate(([gripper_open],finger_positions,gripper_pose,tcp_force,
#	    	    	    	    joint_forces, joint_positions,joint_velocities, [time]))
#	    low_dim_state = torch.tensor([[low_dim_np]], device=self.device,) # why three layers [[[]]]?

	    # For RVT3, PerAct
	    low_dim_state = torch.tensor([[[gripper_open,
	    	    	    	    	    finger_positions[0],
	    	    	    	    	    finger_positions[1],
	    	    	    	    	    time]]], device=self.device, dtype=torch.float32) # for RVT3 # dtype=torch.float64
	    print('low_dim_state',low_dim_state.device, low_dim_state)
	    obs['low_dim_state'] = low_dim_state  #  proprioception data

	    # import pdb; pdb.set_trace()

	    return obs

	    # use average length for tool; gripper_length = 149.3 - 162.8, force_sensor_length = 46.5-50.5, total_length=210-
	    # -0.214 for robotiq, -0.260 RM gripper
	def apply_gripper_offset(self, gripper_pose, offset=(0.0, 0.0, -0.260)): # -0.107 Tool Center Point to flange center point
	    trans_matrix = tf.transformations.translation_matrix([offset[0], offset[1], offset[2]])
	    rot_matrix = tf.transformations.quaternion_matrix([gripper_pose.orientation.x,
	    	    	    	    	    	    	       gripper_pose.orientation.y,
	    	    	    	    	    	    	       gripper_pose.orientation.z,
	    	    	    	    	    	    	       gripper_pose.orientation.w])
	    # To figure out
	    rotated_offset = np.dot(rot_matrix, trans_matrix)

	    gripper_trans_matrix = tf.transformations.translation_matrix([gripper_pose.position.x,
	    	    	    	    	    	    	    	    	  gripper_pose.position.y,
	    	    	    	    	    	    	    	    	  gripper_pose.position.z])

	    rotated_and_translated_matrix = np.dot(gripper_trans_matrix, rotated_offset)

	    final_quat = tf.transformations.quaternion_from_matrix(rotated_and_translated_matrix)
	    final_trans = tf.transformations.translation_from_matrix(rotated_and_translated_matrix)

	    final_pose = copy.deepcopy(gripper_pose)
	    final_pose.position.x = final_trans[0]
	    final_pose.position.y = final_trans[1]
	    final_pose.position.z = final_trans[2]
	    final_pose.orientation.x = final_quat[0]
	    final_pose.orientation.y = final_quat[1]
	    final_pose.orientation.z = final_quat[2]
	    final_pose.orientation.w = final_quat[3]
	    return final_pose

	def check_and_mkdirs(self, dir_path):
	    if not os.path.exists(dir_path):
	    	os.makedirs(dir_path, exist_ok=True)

	def step(self): # change or keep language goal, predict and visualize the next action first, then execution. Control the workflow via joystick
	    try:
#	    	self.capture_mecheye_images()
	    	if self.new_lang_cond(): # change language goals
	    	    self.lang_goal = input("Language Goal: ")
	    	    self.state['step'] = int(input("Step: "))

	    	elif self.next_cond(): # predict and visualize the next action
	    	    observation = self.get_obs()
	    	    print("current gripper pose:", observation['gripper_pose'])
	    	    self.act_result = self.agent.act(self.state['step'], observation,
		    	    	    	    		     deterministic=True)  # predict ?

	    	    # for RVT3
	    	    action = self.act_result.action  # continuous_action(continuous attention_coordinate, quaternion, gripper_open,ignore_collisions), check QAttentionStackAgent for details
	    	    print(f"Step: {self.state['step']}  | ", "Actions: ", " ".join(f"{x:.4f}" for x in action))

	    	    # For FP2AT-----------------------------
	    	    '''
		    	# visualize voxels
	    	    voxel_grid = self.act_result.info['voxel_grid_depth0'].detach().cpu().numpy()[0] # defined as 'voxel_grid_depth%d' % self._layer: vox_grid in fusion_peract_bc_agent
	    	    pred_q = self.act_result.info['q_depth0'].detach().cpu().numpy()[0] # 'q_depth%d' % self._layer: q_trans, softmaxed q values of translation
	    	    pred_trans_indices = self.act_result.info['voxel_idx_depth0'].detach().cpu().numpy()[0] # # xyz of q predition
	    	    pred_rot_grip = self.act_result.info['rot_and_grip_indicies0'].detach().cpu().numpy()[0] # rotation and gripper status of q attention
	    	    # print(voxel_grid.shape, pred_q.shape, pred_trans_indices.shape)

	    	    # TODO  change the visualise params
	    	    voxel_render = utils.visualise_voxel(voxel_grid,
	    	    	    	    	     pred_q,
	    	    	    	    	     highlight_coordinate= pred_trans_indices,
	    	    	    	    	     highlight_gt_coordinate=None,
	    	    	    	    	     orientation=pred_rot_grip[:3]*self._rotation_resolution,
	    	    	    	    	     gripper_open=pred_rot_grip[3],
	    	    	    	    	     highlight_alpha = 1.0,
	    	    	    	    	     show=True,
	    	    	    	    	     voxel_size= 0.1,
	    	    	    	    	     name_prefix='realvis_g_2',
	    	    	    	    	     rotation_amount=-np.pi*11/16,
	    	    	    	    	     alpha=0.9) # alpha=0.5, show=True,name_prefix='g_1'
		    	# print(voxel_render.shape)
		    	# TODO visualize local voxelgrid, maybe directly visualize and save local & global voxel grid in fusion_peract_bc_agent
	    	    self.pred_voxel_pub.publish(self.cv_bridge.cv2_to_imgmsg(voxel_render, encoding='rgb8')) #TODO publish for visualize voxelgrid when executing

	    	    action = self.act_result.action  # continuous_action(continuous attention_coordinate, quaternion, gripper_open,ignore_collisions), check QAttentionStackAgent for details

		    	# visualize action
	    	    action_pose = PoseStamped()
	    	    action_pose.header.frame_id = self.base_frame
	    	    action_pose.pose.position.x = action[0] 
	    	    action_pose.pose.position.y = action[1] 
	    	    action_pose.pose.position.z = action[2] 
	    	    action_pose.pose.orientation.x = action[3]
	    	    action_pose.pose.orientation.y = action[4]
	    	    action_pose.pose.orientation.z = action[5]
	    	    action_pose.pose.orientation.w = action[6]

#		    	action_pose.pose = self.apply_gripper_offset(action_pose.pose, offset=(0.0, 0.0, -0.214))
	#	    	self.peract_action_marker.pose = copy.deepcopy(action_pose.pose)  # gripper marker
	#	    	self.peract_action_pub.publish(self.peract_action_marker)
	    	    '''
	    	    # --------------------------------------------------

	    	    gripper_open = action[7]
	    	    rospy.loginfo(f"Step: {self.state['step']} | Gripper Open: {action[7]} {action[7] > 0.99} | Ignore Collisions: {action[8] > 0.99}")


	    	elif self.execute_cond(): # predict and execute the next action, Control the workflow via joystick
	    	    # for RVT3 -----------------------------
	    	    observation = self.get_obs()
	    	    self.act_result = self.agent.act(self.state['step'], observation,
		    	    	    	    		     deterministic=True)
	    	    # for both RVT3 and FP2AT -------------------------------------------

	    	    if self.act_result is not None:
	    	    	action = self.act_result.action
		    	    # execute
	    	    	action_pose = PoseStamped()
	    	    	action_pose.header.frame_id = self.base_frame
	    	    	action_pose.pose.position.x = action[0] 
	    	    	action_pose.pose.position.y = action[1] 
	    	    	action_pose.pose.position.z = action[2] 
	    	    	action_pose.pose.orientation.x = action[3]
	    	    	action_pose.pose.orientation.y = action[4]
	    	    	action_pose.pose.orientation.z = action[5]
	    	    	action_pose.pose.orientation.w = action[6]

#		    	    action_pose.pose = self.apply_gripper_offset(action_pose.pose, offset=(0.0, 0.0, -0.214)) # for Franka -0.107
	    	    	# Go/move to next key pose; quaternion to angles in it
	    	    	rospy.loginfo(f"Take next key action {action_pose.pose}")
	    	    	self.goto_pose(action_pose.pose) # GOTO, inputs a list with 3 position and 3 angles

	    	    	#keep it or not, change gripper status when joint velocity are close to zero
	    	    	joint_velocities=np.array(self.curr_data['joint_states'].velocity)[:6]
	    	    	while np.any(joint_velocities > 0.008):
	    	    	    joint_velocities=np.array(self.curr_data['joint_states'].velocity)[:6]
	    	    	    rospy.loginfo(f"Waiting for joint velocities decreasing to a small value")

	    	    	# (keep) opening or closing the gripper
	    	    	gripper_open = action[7]
	    	    	if gripper_open > 0.99:
	#	    	    	self._franka_set_gripper(gripper_open) # change gripper status
#	    	    	    self.clows(0) # open max, for robotiq
	    	    	    self.axis.move_to(0) # for RM gripper

	    	    	else:
#	    	    	    self.clows(255) #close gripper, for robotiq
	    	    	    self.axis.push(9,31,20) # put_in_drawer; for RM gripper, force, distance, vel, self.axis.push(10,20,20) for insert_round_hole

		    	    #save act_result to disk
	    	    	if self.cfg['logs']['save']:
	    	    	    log_path = self.cfg['logs']['log_path']
	    	    	    self.check_and_mkdirs(log_path)

	    	    	    result_path = os.path.join(log_path,
		    	    	    	    	    	   self.cfg['logs']['task'],
		    	    	    	    	    	   f"episode{self.cfg['logs']['episode_id']}",
		    	    	    	    	    	   'act_result')
	    	    	    self.check_and_mkdirs(result_path)


	    	    	    result_file = os.path.join(result_path,
		    	    	    	    	    	   f"{self.state['step']}.pkl")

	    	    	    results = {
		    	    	    'act_result': self.act_result,
		    	    	    'lang_goal': self.lang_goal,
		    	    	    'step': self.state['step']
		    	    	}
	    	    	    with open(result_file, 'wb') as f:
	    	    	    	pickle.dump(results, f)

	    	    	    rospy.loginfo(f"Saved step {self.state['step']} act_result: {result_file}")


	    	    	self.state['step'] += 1


	    except KeyboardInterrupt:
	    	self.agent.unload_clip()
	    	self.agent._network.free_mem()
	    	torch.cuda.empty_cache()
	    	self.goto_pose(self.initial_pose)  # return to the initial pose
	    	self.shutdown_planner()
	    	rospy.loginfo("Shutting down agent interface.")

	def save_keypoint(self): # figure out the alignment between step() and save_keypoint()
	    # make directories
	    def check_and_mkdirs(dir_path):
	    	if not os.path.exists(dir_path):
	    	    os.makedirs(dir_path, exist_ok=True)

	    save_path = os.path.join(self.cfg['agent']['save_path'], self.cfg['agent']['task'])
	    episode_idx = self.cfg['agent']['episode']
	    variation_idx = self.cfg['agent']['variation']

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

	    misc['front_camera_extrinsics'] = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['front_camera']).pose) # transformation between base and cam
	    misc['front_camera_near'] = self.cfg['agent']['front_cam_near']  #TODO 0.6, 0.5 for kinectv2 # operative measuring range
	    misc['front_camera_far'] = self.cfg['agent']['front_cam_far'] # 6, https://www.intelrealsense.com/depth-camera-d455/ # 4.5 for kinect v2

	    misc['overhead_camera_intrinsics'] = np.array(frame0['overhead_camera_info'].K).reshape(3,3)
	    misc['overhead_camera_extrinsics'] = self.pose_to_4x4mat(self.get_tf(self.base_frame, self.cfg['frame']['overhead_camera']).pose)
	    misc['overhead_camera_near'] = self.cfg['agent']['overhead_cam_near']
	    misc['overhead_camera_far'] = self.cfg['agent']['overhead_cam_far']

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

	    demo = Demo(observations, random_seed=self.cfg['agent']['random_seed']) # low dimentional data
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



@hydra.main(config_path="../cfgs", config_name="peract_agent")
def main(cfg):
	# initialize(config_path="../cfgs", job_name="peract_demo")
	# cfg = compose(config_name="peract_demo")
	pprint.pprint(dict(cfg))


	rospy.init_node('peract_agent', anonymous=True)
#	rospy.wait_for_service('/capture_color_map')
#	rospy.wait_for_service('/capture_depth_map')
#	rospy.wait_for_service('/capture_point_cloud')
	interface = PeractAgentInterface(cfg)
	rospy.sleep(0.1)

	#TODO check if step() in joy_state_cb() + rospy.spin() in __init__() or while loop in main function works
#	while not rospy.is_shutdown():
#	    try:
#	    	interface.step()
#	    	interface.loop_rate.sleep() # 10Hz in default
#
#	    except KeyboardInterrupt:
#	    	print("Shutting down demo interface.")

if __name__ == '__main__':
	main()
