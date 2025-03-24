#! /usr/bin/env python
import socket
import time
from learning_joy.msg import Ur_move
from math import cos,sin,atan2,sqrt
import std_msgs
from std_msgs.msg import Float32MultiArray
import rospy
import threading
import asyncio
import struct

import time
# from pose_change import pose_trans

changeState = 0
recv_data = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
x=0.0
y=0.0
z=0.0
rx=0.0
ry=0.0
rz =0.0

#  读取当前机械臂的姿态
def getCurrentPose():
    data = s.recv(1220)  # 包的长度
    while(len(data)<492):  # Added
        print('re-receive data of 1st arm, got ', len(data))
        data = s.recv(1220)  # 包的长度
    mx = struct.unpack('!d', data[444:452])[0]
    my = struct.unpack('!d', data[452:460])[0]
    mz = struct.unpack('!d', data[460:468])[0]
    mrx = struct.unpack('!d', data[468:476])[0]
    mry = struct.unpack('!d', data[476:484])[0]
    mrz = struct.unpack('!d', data[484:492])[0]
    return mx,my,mz,mrx,mry,mrz


def move(p):
    pass
    global recv_data,x,y,z,rx,ry,rz
    # print("******一次运动*******")
    recv_data[0] = p.x
    recv_data[1] = p.y
    recv_data[2] = p.z
    recv_data[3] = p.rx
    recv_data[4] = p.ry
    recv_data[5] = p.rz
    recv_data[6] = p.state
    print(p.x,p.y,p.z,p.rx,p.ry,p.rz)
    mx = p.x
    my = p.y
    mz = p.z
    mrx = p.rx
    mry = p.ry
    mrz = p.rz

    #  如果状态位 1， 需要改变
    if(abs(p.x)+abs(p.y)+ abs(p.z) + abs(p.rx)+ abs(p.ry)+abs(p.rz) <0.5 or time.time()-start_time<1):
            urscript = '''
                def svt():
                    stopl(1)
                end
                '''
            print("停止运动")
            s.send(urscript.encode('utf8'))
    if(p.state != 0.0  ):
        print("进入运动")
        print(p.x,p.y,p.z,p.rx,p.ry,p.rz)

        if(abs(p.x)+abs(p.y)+ abs(p.z) + abs(p.rx)+ abs(p.ry)+abs(p.rz) <1):
            urscript = '''
                def svt():
                    stopl(1)
                end
                '''
            print("停止运动")
        else:
            with lock:
                # 得到新的点，移动过去 #   movej  movel a=0.12, v=0.1, in default; 0.12, v=0.03 for contact-rich mani
                urscript = '''
                    def svt():
                        pose = p[0,1,2,3,4,5]
                        movej(pose_add(p[{}, {}, {}, {}, {}, {}],p[{}, {}, {}, {}, {}, {}]),a=0.12, v=0.1, t=0, r=0)
                    end
                    '''.format(x,y,z,rx,ry,rz,0,0,0,mrx,mry,mrz) # rotation   # a=1, v=1.2
                s.send(urscript.encode('utf8'))
                urscript = '''
                    def svt():
                        pose = p[0,1,2,3,4,5]
                        movel(pose_add(p[{}, {}, {}, {}, {}, {}],p[{}, {}, {}, {}, {}, {}]),a=0.12, v=0.1, t=0, r=0)
                    end
                    '''.format(x,y,z,rx,ry,rz,mx,my,mz,mrx,mry,mrz) # translation, a=0.12, v=0.03
        s.send(urscript.encode('utf8'))

def joy_move():
    global x,y,z,rx,ry,rz
    global s
    while not rospy.is_shutdown():
        data = s.recv(1220)
        if not data:
            print("没有收到数据")
            break
        with lock:
            x = struct.unpack('!d', data[444:452])[0]
            y = struct.unpack('!d', data[452:460])[0]
            z = struct.unpack('!d', data[460:468])[0]
            rx = struct.unpack('!d', data[468:476])[0]
            ry = struct.unpack('!d', data[476:484])[0]
            rz = struct.unpack('!d', data[484:492])[0]
            data = Float32MultiArray()
            data.data = [x,y,z,rx,ry,rz]
            pub.publish(data)
            # x,y,z,rx,ry,rz = getCurrentPose()
            # x,y,z,rx,ry,rz = i,i,i,i,i,i
            # print("机械臂位姿：",x)
        #     # print("机械臂位姿：",x,y,z,rx,ry,rz)


if __name__ == "__main__":
    start_time = time.time()
    print(start_time)
    lock = threading.Lock()

	# 2、建立socket连接
    HOST= "172.20.172.102"
    PORT = 30003  # socket
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect((HOST,PORT))

	# Added # set TCP offset
    urscript = "set_tcp(p[{}, {}, {}, {}, {}, {}])\n".format(0, 0, 0.260, 0, 0, 0) # 0.214 for robotiq gripper, 0.260 for RM gripper
    s.send(urscript.encode('utf8'))
    rospy.sleep(0.5)

#    urscript = "get_actual_tcp_pose()\n"
#    s.send(urscript.encode('utf8'))
#    rospy.sleep(0.5)

    rospy.init_node("listener_person_p")
    sub = rospy.Subscriber("/urState",Ur_move,move,queue_size=2)
    pub = rospy.Publisher("tcp_pose",Float32MultiArray,queue_size=1)

    thread1 = threading.Thread(target=joy_move)
    thread1.start()
    rospy.spin()







