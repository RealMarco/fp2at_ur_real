#1.导包 
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from learning_joy.msg import Servo_move
import time
import threading 



import os

def deFile_if_exits(file_path):

    if os.path.exists(file_path):
        os.remove(file_path)
        print("文件已删除")
    else:
        print("文件不存在")

def append_data_to_file(filename, time, data):
    with open(filename, 'a') as file:
        file.write(str(time) + " " + str(data) + '\n')
    
# 指定文件名和数据
# filename = '../data/motodata.txt'
filename = '/home/znfs/fsx_ws/src/traj_follow/exp4 data/motordata.txt'


speed = 0
count_lock = threading.Lock()
start_time = time.time()

# servo_move/command
def doMsg(msg):
    global speed
    rospy.loginfo("the speed is %f",msg.v)
    with count_lock:
        speed = msg.v

def write_data():
     # 设置写入频率为10Hz
    frequency = 10.0  # Hz
    period = 1.0 / frequency  # 秒

    # 循环写入数据
    while not rospy.is_shutdown():
        # 写入数据到文件
        time_now = time.time()
        with count_lock:
            data = speed
            print("写入数据：",data)
            append_data_to_file(filename=filename,time=time_now-start_time,data=data)
        
        # 等待一段时间
        time.sleep(period)

if __name__ == "__main__":
    deFile_if_exits(file_path=filename)
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node("motor_speed_record")
    #3.实例化 订阅者 对象
    sub = rospy.Subscriber("servo_move/command",Servo_move,doMsg,queue_size=1)
    
    #4.处理订阅的消息(回调函数)
    #5.设置循环调用回调函数
    
    print("线程开始！")
    write_thread = threading.Thread(target=write_data)
    write_thread.start()
    rospy.spin()
   

    