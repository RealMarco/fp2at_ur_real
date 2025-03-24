#include <ros/ros.h>
#include "learning_joy/Ur_move.h"
#include "learning_joy/Servo_move.h"
#include <sensor_msgs/Joy.h>  
#include <iostream>
using namespace std;

// create the TeleopUR class and define the joyCallback function that will take a joy msg
class TeleopUR
{
public:
  TeleopUR();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int linear_, angular_;   // used to define which axes of the joystick will control our turtle
  double l_scale_, a_scale_,axes1_;
  int button1_;  
  int button2_;  
  int button3_;// 手柄按键定义
  ros::Publisher vel_pub_;
  ros::Publisher servo_pub_;
  ros::Subscriber joy_sub_;
};

TeleopUR::TeleopUR(): linear_(0), angular_(1), axes1_(1),l_scale_(0.001) // 代表手柄遥感， 按键，以及机械臂速度映射
{
  // create a publisher that will advertise on the command_velocity topic of the turtle
  vel_pub_ = nh_.advertise<learning_joy::Ur_move>("ur_move/command", 1);
  servo_pub_ = nh_.advertise<learning_joy::Servo_move>("servo_move/command",1);
  // subscribe to the joystick topic for the input to drive the turtle
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopUR::joyCallback, this);
}


void TeleopUR::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  learning_joy::Ur_move ur_msg;
  
  learning_joy::Servo_move servo_msg;
  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  // 
  ur_msg.x =l_scale_ *  joy->axes[angular_];
  ur_msg.y =l_scale_ *  joy->axes[linear_];
  // 按键定义
  // if(joy->buttons[button1_]==1)
  // {
  //   servo_msg.v = 2;
  // }else if(joy->buttons[button2_]==1)
  // {
  //   servo_msg.v = 1;
  // }  
  // else if(joy->buttons[button3_]==1)
  // {
  //   servo_msg.v = -6;
  // }
  // else{
  //   servo_msg.v = 0;
  // }
  servo_msg.v = joy->axes[axes1_];
  cout << ur_msg.x << " " << ur_msg.y << " buttons:" << servo_msg.v<< endl;
  vel_pub_.publish(ur_msg);   // 发送机械臂的消息
  servo_pub_.publish(servo_msg);  // 发送电机控制信息
}
int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_turtle, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_pub");
  TeleopUR teleop_turtle;
  
  ros::spin();
}