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
