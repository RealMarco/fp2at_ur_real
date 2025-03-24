import socket
import time

HOST= "192.168.1.219"
PORT = 30003  # socket 
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

s.connect((HOST,PORT))

# 发送urscript指令
# pos=pose_trans(get_actual_tcp_pose(),p[0.0, 0.0, 0.1, 0.0, 0.0, 0.0])\n
i = 100
while True:
    urscript = "movep(pose_trans(get_actual_tcp_pose(),p[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),a=1.2,v=0.03)\n"
    # urscript = "servoj(get_inverse_kin(p[-0.3,-0.6,0.4,2.231,-2.231,0]), t=0.008,lookahead_time=0.05, gain=500)\n"
    s.send(urscript.encode('utf8'))
    time.sleep(0.01)
    urscript = "movep(pose_trans(get_actual_tcp_pose(),p[0.0, -0.002, 0.0, 0.0, 0.0, 0.0]),a=1.2,v=0.03)\n"
    # urscript = "servoj(get_inverse_kin(p[-0.3,-0.6,0.4,2.231,-2.231,0]), t=0.008,lookahead_time=0.05, gain=500)\n"
    s.send(urscript.encode('utf8'))
    time.sleep(0.01)
    urscript = "movep(pose_trans(get_actual_tcp_pose(),p[-0.001, 0.0, 0.0, 0.0, 0.0, 0.0]),a=1.2,v=0.03)\n"
    # urscript = "servoj(get_inverse_kin(p[-0.3,-0.6,0.4,2.231,-2.231,0]), t=0.008,lookahead_time=0.05, gain=500)\n"
    s.send(urscript.encode('utf8'))
    time.sleep(0.01)
    
# while i>0:
#     urscript = "servoj(get_inverse_kin(p[-0.2,-0.6,0.5,2.231,-2.231,0]), t=0.008,lookahead_time=0.05, gain=500)\n"
#     s.send(urscript.encode('utf8'))
#     i-=1

# 关闭socket连接
s.close()