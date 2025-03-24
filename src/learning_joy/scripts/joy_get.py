#! /usr/bin/env python
import rospy
from learning_joy.msg import Ur_move
from sensor_msgs.msg import Joy


def move(msg):
    
    axes = msg.axes
    
    button = msg.buttons
    ur_axis = Ur_move()
    isAllZero = 0
    for i in button:
        if i!=0:
            isAllZero = i
            break
    if(isAllZero == 0):
        ur_axis.x = 0
        ur_axis.y = 0
        ur_axis.z = 0
        ur_axis.rx = 0
        ur_axis.ry = 0
        ur_axis.rz = 0

    if(button[0]!=0):
        # x-
        ur_axis.x = -1
    if(button[3]!=0):
        # x-
        ur_axis.x = 1

    if(button[1]!=0):
        # y-
        ur_axis.y = -1
    if(button[2]!=0):
        # y+
        ur_axis.y = 1

    if(button[4]!=0):
        # z-
        ur_axis.z = -1
    if(button[5]!=0):
        # z+
        ur_axis.z = 1

    # 定义旋转轴
    if(axes[6]==1):
        # rx+
        ur_axis.rx = 1
    if(axes[6]==-1):
        # rx-
        ur_axis.rx = -1
    
    if(axes[7]==1):
        # ry+
        ur_axis.ry = 1
    if(axes[7]==-1):
        # ry-
        ur_axis.ry = -1
    
    if(axes[2]<0):
        # rz+
        ur_axis.rz = 1
    if(axes[5]<0):
        # rz-
        ur_axis.rz = -1

    print(ur_axis)
    pub.publish(ur_axis)
    


if __name__ == "__main__":
#1. 初始化节点
    rospy.init_node("joy_get")
    sub = rospy.Subscriber("joy",Joy,move,queue_size=1); 
    pub = rospy.Publisher("axis",Ur_move,queue_size=1)
    rospy.spin()
