# Real Robot Implementation of FP2AT

This is the code for deploy FP2AT on ROS machine. Including data collection, and robot experiments for UR robots.

**Author**: [Marco Yangjun Liu](https://github.com/RealMarco/),  
**Affiliation**:  Centre for Artificial Intelligence and Robotics, University of Macau; Shenzhen Institutes of Advanced Technology, Chinese Academy of Sciences

## Citation
If you are using this Robotic Grasping Simulation code, please add the following citation to your publication:
```
@ARTICLE{10874177,
  author={Liu, Yangjun and Liu, Sheng and Chen, Binghan and Yang, Zhi-Xin and Xu, Sheng},
  journal={IEEE Transactions on Robotics}, 
  title={Fusion-Perception-to-Action Transformer: Enhancing Robotic Manipulation With 3-D Visual Fusion Attention and Proprioception}, 
  year={2025},
  volume={41},
  pages={1553-1567},

```

![Examples](3x.gif)


Hardware: UR5, realsense camera, robotiq gripper (or RobustMotion)
Software: RLBench
## References 
- [franka_htc_teleop.zip](https://github.com/peract/peract/files/11362196/franka_htc_teleop.zip) 
- [move_group_python_interface_tutorial](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html)
- [MoveItjade  API](http://docs.ros.org/en/jade/api/moveit_commander/html/- classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html)
- [MoveIt jade source code](http://docs.ros.org/en/jade/api/moveit_commander/html/move__group_8py_source.html)
- **MoveIt Noetic Source Code**: /opt/ros/noetic/lib/python3/dist-packages/moveit_commander/move_group.py
## File Description
1. `peract_demo_interface.py` is for collecting data
2. `peract_agent_interface.py` is for executing trained models.  See [issue18](https://github.com/peract/peract/issues/18#issuecomment-1478827887) for more details on the setup, and [issue2](https://github.com/peract/peract/issues/2) for real-world data.
- may change arm.franka_c2farm_perceiver_lang_bc to agents.peract_bc

## Usage
0. ur_calibrate at first
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=172.20.172.102 target_filename:="/home/marco/robotic_sorting/src/Universal_Robots_ROS_Driver/ur_robot_driver/config/my_robot_calibration.yaml"
```
1. check the IP of 3090, robot1 and robot2
1. ROS communication between two computers
```
In the first computer with force sensor and gripper
$ roscore 
$ export ROS_IP=172.20.172.103 # inet IP of the first computer
# Connect to force sensor
cd ~/ur5_ws
source ./devel/setup.bash
sudo chmod +777 /dev/ttyUSB1
rosrun force_sensor get_sensor_data.py
# Connect to the gripper
cd ~/ur5_ws
source ./devel/setup.bash
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

In the second computer with vision
$ export ROS_MASTER_URI=http://172.20.172.103:11311
$ export ROS_IP=172.20.172.101 # inet IP of the second computer
$ rostopic list  # verify

```
### handeye calibration
1. Modify and roslaunch the eye_to_hand_calibration_official_driver.launch
```
$ roslaunch easy_handeye fp2a_front_eye_to_hand_calibration.launch
$ roslaunch easy_handeye fp2a_overhead_eye_to_hand_calibration.launch
```
2. set the correct ip and port for external control as 172.20.172.101:50002 (PC_IP, 50002 as default), load a program on PolyScope which contains the external_control (e.g., in E204 lab: 为机器人编程 -> 加载 ros_control (-> 安装设置 -> 设置external control))
3. calibrate the hand and eye following the workflow: Check starting pose -> (next pose -> plan -> execute -> taske sample) x17 -> compute -> save (aumatically save into ~/.ros/easy_handeye/xxx.yaml)
4. Modify the publish.launch accordingly.Then Check calibration results
```
$ roslaunch easy_handeye fp2a_two_cam_publish.launch
$ rosrun tf tf_echo /base /camera_color_optical_frame
$ rosrun tf tf_echo /base /camera_overhead_color_optical_frame
```
5. results for the front camera:


6. Tips for accuracy

- Maximize rotation between poses.
- Minimize the distance from the target to the camera of the tracking system.
- Minimize the translation between poses.
- Use redundant poses.
- Calibrate the camera intrinsics if necessary / applicable.
- Calibrate the robot if necessary / applicable. 
  
### when collecting demos
1. Start the UR driver, and check the node for publishing joint states
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=172.20.172.102  kinematics_config:=/home/marco/robotic_sorting/src/Universal_Robots_ROS_Driver/ur_robot_driver/config/my_robot_calibration.yaml
rostopic list
rostopic echo /joint_states
```
3. check the names of robot links
```
rosrun tf tf_echo /base /tool0 # to check the tf tree and frames
# check if the x,y,z of tf are the same as socket communication
rostopic echo tcp_pose # published by learning joy (ur_joy_controlnew.py), in translation + 3D rot vec format (the data acquired by socket)
rosrun two_ur first_arm.py 

# the tf results are the same as: $ rosrun ur_robot_driver  test_get_tf.py

$ rosrun rqt_tf_tree rqt_tf_tree # more intuitively to observe the tf tree
```
3. rqt, rqt_graph, 	catkin init, catkin build, catkin build pkg, catkin clean my_package
3. Start the camera
```
$ roslaunch mecheye_ros_interface start_camera.launch

$ roslaunch realsense2_camera rs_camera.launch align_depth:=true camera:=camera serial_no:=135222250811 enable_infra:=true enable_infra1:=true enable_infra2:=true initial_reset:=true filters:=colorizer 

$ roslaunch realsense2_camera rs_camera.launch align_depth:=true camera:=camera_overhead serial_no:=135222251592 filters:=pointcloud,spatial,temporal,hole_filling

# Other arguments
- filters:=pointcloud,spatial,temporal,hole_filling  #colorizer,   ; according to https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#launch-parameters
- initial_reset:=true

# config and start multiple cameras according to https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#work-with-multiple-cameras
$ rs-enumerate-devices | grep Serial    # # find the serial numbers of cameras
# launch multiple cameras 
$ roslaunch realsense2_camera rs_two_devices.launch camera1:=camera serial_no_camera1:=135222250811 camera2:=camera_overhead serial_no_camera2:=135222251592 filters:=spatial,temporal

pointcloud,hole_filling

```
4. $ roslaunch easy_handeye publish.launch
----------------------------------------------

4. Start the UR driver, Camera and transformation publisher together
```
$ roslaunch ur_robot_driver ur5_fp2a_demo.launch
```
4.  publish the extrinsics and cameras
``` 
roslaunch easy_handeye fp2a_two_cam_publish.launch
```
4. set the correct ip and port for external control as 172.20.172.101:50002 (PC_IP, 50002 as default), load a program on PolyScope which contains the external_control (e.g., in E204 lab: 为机器人编程 -> 加载 ros_control (-> 安装设置 -> 设置external control))
4. close the ur_joy and peract_demo_interface.py, then goto initial pose
```
rosrun fp2a_ros goto_initial_pose.py
```
4.
```
4. build learning_joy and Start the joy driver

if any error of lacking of Ur_move.h happens when catkin_make or catkin build, comment away the "add_executable" in CMaKeList.txt to catkin build the msg at first, then catkin build the entire package

$ roslaunch learning_joy ur_joy.launch

watch joy topics: $ rostopic echo joy

5.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT 
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT

# check the port of RM gripper (e.g., '/dev/ttyUSB1'), and modify the corresponding parts in peract_demo_interface.py 
rosrun fp2a_ros peract_demo_interface.py   
# or for robotiq: peract_demo_interface_robotiq.py
```

( **roslaunch fp2a_ros  fp2a_demo.launch** )
 
### When testing
2.  Start the UR driver and MoveIt,
```
$ roslaunch ur_robot_driver ur5_fp2a_agent.launch   # including ur5_bringup.launch
```
4. Start the Cameras
3. connect two ROS machine 
- set the correct ip and port for external control as 172.20.172.101:50002 (PC_IP, 50002 as default), load a program on PolyScope which contains the external_control (e.g., in E204 lab: 为机器人编程 -> 加载 ros_control (-> 安装设置 -> 设置external control))
- set ROS_MASTER_URI and ROS_IP in the computer with vision, set ROS_IP in the computer with force sensor and gripper
(5. roslaunch easy_handeye publish.launch)
4. rosrun fp2a_ros peract_agent_interface.py
5. use joy stick to control the workflow including execution, visualization and setting new language 
4. goto initial pose after each task


```

roslaunch ur_robot_driver ur5_fp2a_agent.launch

roslaunch easy_handeye fp2a_two_cam_publish.launch

rosrun fp2a_ros goto_initial_pose.py
roslaunch learning_joy agent_joy.launch  # for testing
roslaunch learning_joy ur_joy.launch  # for remote joystick control

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT 
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT

(
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
)

# check the port of RM gripper (e.g., '/dev/ttyUSB1'), and modify the corresponding parts in peract_agent_interface.py 
rosrun fp2a_ros peract_agent_interface.py
# or for robotiq: peract_agent_interface_robotiq.py

rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

```

### replay after demo
```
roslaunch learning_joy agent_joy.launch 
rosrun fp2a_ros peract_replay_interface.py
```

## Set tool offset for the UR
**Option 1:** (recommended)  
1. modify the urdf, (especially end-effector length, gripper length, 0.214m for robotiq,  0.260m for RM Gripper)   
2. set the offset for socket communication, especially for the movement when collecting data
3. directly get the gripper_pose via get_tf
4. directly go to gripper_pose via MoveIt when testing without extra apply_gripper_offset 

Option 2:
1. keep the original urdf without force sensor and gripper
2. use the original pose for socket communication without force sensor and gripper
3. add offset when collecting data, considering the consistency between voxelgrid and 3D space
4. when testing: add extra apply_gripper_offset  for inputting and remove offset for execution, since the flange end are controlled

Option 3: **Doesn't work due to limited planning ability of urscript**
1. set the offset for socket communication, use urscript to move the robot when collecting and executing


## Pay Attention to
1. the rotation acquired by $ rosrun tf tf_echo /base  /tool0, get_pose() via socket and the PolyScope (teach pad) are different from each other
2. To use ROS **tf** to get the pose of the TCP (tool0), we simply **modified** the **xyz** value (x=0.214) of **tool0** in **/opt/ros/noetic/share/ur_description/urdf/inc/ur_macro.xacro**. The **original** ur_macro.xacro was backed up as ur_macro.xacro.backup. **Pull it back when u wanna use a new setting of UR robots** (end-effector length, gripper length, 0.214m for robotiq,  0.260m for RM Gripper)
3. The MoveIt itself can achieve collision avoidance via the default motion planner OMPL (Open Motion Planning Library), avoiding both collision with the robot arm itself (see ur_description/config/ur5/physical_parameters.yaml) and the environment [move_group.compute_cartesian_path(self, waypoints, eef_step, jump_threshold, avoid_collisions = True, path_constraint=None,)]. Check details in https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#attaching-objects-to-the-robot , https://ros-planning.github.io/moveit_tutorials/doc/bullet_collision_checker/bullet_collision_checker.html ,  https://ros-planning.github.io/moveit_tutorials/doc/visualizing_collisions/visualizing_collisions_tutorial.html , https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html
4. The code below will be modified when deploying on real-robot
```
_check_add_types was canceled in add_final() and add() of YARR/yarr/replay_buffer/uniform_replay_buffer.py

```

## Pay Attention to ... when Collecting Demos...
1. **record after waiting for a second** since the real-time capablity of **subscribing 'joint_states'** topic by ROS is inperfect
2. check if the number of collected frames and thoes acquired by keyframe dicovery are the same
3. if any error of tf tree occurs, **re-publish calibration info** and then re-collecting.
4. To make full use of depth information, increase the layering of the object surface by labeling, using different colors, try training to check the voxelization, etc.  
5. collect demos in scene bounds and proper observing range of cameras
6. accurate changing of the gripper status, or change the gripper status with slight movement ahead. 
7. vary in grasp pose, intial position/pose, scene layout, (disturbance, color, size)?   
8. **goto intial_pose and open the gripper** before collecting one demo


## Pay Attention to ... when training...

1. choose proper scene bounding box for different tasks 

2. method.keypoint_method=**'all'** instead of 'heuristic' to simply recognize all the actions as key actions
3. do not visualise the **second** param (q_attention) of visualise_voxel() to avoid **strange red voxels**
4. must finetune grip_and_rot

## Pay Attention to ... when testing...
1. **Goto initial state** at first, 
2. May try to visualise the second param (q_attention) of visualise_voxel() abserve the q predictions in the 3D space
3. Things to record when testing
- the voxelization results at each step, for paper
- the videos of tasks for each tasks, for supply materials
- **save the test dataset at the same time** for later voxelization, re-set the joystick bottons
- The recordings should present the **scene & target variance**, and may show the other capabilities including **close-loop control, human disturbance, different initial 7D pose, and collision avoidance** when execution
- Maybe screenshot the trimesh.scene.show and link the videos with trimesh.scene.show
4. modify the saving directoty of voxelization .pngs at visualise_voxel() of peract/helpers/utils.py 


## Voxelization and depth images
1. the value range of mecheye's depth is 200-4000. The farther pixels have higher pixel values
2. the value range of realsense and kinect's depth is 0 - 65535 (2**16-1). As for realsense, The farther pixels have higher pixel values.
3. the depth images of RLBench is tranfer in an RGB images within 0-255, 3 channels, relative depth (near-far)

Steps to collect depth images:
Sample depth images: episode94
set the proper near and far value of the camera
clip and normalization to 0-1
fill in the pixels with the value of 0

## Things to check
1. image resolution


utils.float_array_to_rgb_image(img_real_norm, 2**24-1) 


# JoyStick Demonstration
## Dependencies
- sudo apt-get install ros-<distro>-joy

## build
1. if any error of lacking of Ur_move.h happens when catkin_make or catkin build, comment away the "add_executable" in CMaKeList.txt to catkin build the msg at fisrt, then catkin build the entire package

## Usage
## check the output value of axes and bottons of joy stick
- roscore
- rosrun joy joy_node
- rostopic echo /joy
- # press bottons and use axes to observe the output
### plug the xbox joystick and check the robot ip in ur_joy_controlnew.py
cd ~/rosproject/ur_joy/
source devel/setup.bash
roslaunch learning_joy ur_joy.launch
motor_control.launch and ur_motor_start.launch are for control the motor of guidewire

## Improvement
1. adapt the direction of XY translation
2. change the speed of XY translation
3. the translation limitation is 1m
3. the rotation bottons controls the rotations w.r.t. the TCP, i.e., roll, pitch, yaw
4. change the rotation limitation

## References
1. joy pakcage and joy_node: https://index.ros.org/p/joy/
2. set the functions in joy_get.py for every bottons and axes on the joy (our case uses 5.3 Microsoft Xbox 360 Wired Controller for Linux): http://wiki.ros.org/joy 

### Microsoft Xbox 360 Wired Controller for Linux
```
Table of index number of /joy.buttons:
Index	Button name on the actual controller
0	A
1	B
2	X
3	Y
4	LB
5	RB
6	back  # open gripper; visualize the prediction at first
7	start # close gripper; execute
8	power # record; 
6+7 #;  set new language goals when execution
9	Button stick left
10	Button stick right

Table of index number of /joy.axes:
Index	Axis name on the actual controller
0	Left/Right Axis stick left
1	Up/Down Axis stick left
2	LT
3	Left/Right Axis stick right
4	Up/Down Axis stick right
5	RT
6	cross key left/right
7	cross key up/down
```
