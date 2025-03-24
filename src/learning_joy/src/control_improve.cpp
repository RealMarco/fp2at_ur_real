#include <iostream>
#include "learning_joy/Servo_move.h"
#include "zmcaux.h"
#include "zmotion.h"
#include <unistd.h>
#include <ros/ros.h>
#include <pthread.h>

using namespace std;
/**
 * @brief 
 * 电机连接，一些参数使用
 */
ZMC_HANDLE handle = NULL;
int ret = 0;  // 返回值
char* ipaddr= (char*)"192.168.1.11"; // 电机的ip地址，一定要在局域网内
// 定义按键检测
int key_press = 0; // 检测按键三个值 0， 1， -1，代表静止，不同方向运动
int anix = 0;  // 确定是电控箱的哪个轴

pthread_mutex_t con_mutex;  // 定义锁

void commandCheckHandler(const char *command, int ret)
{
	if (ret)//非0则失败
	{
		printf("%s fail!return code is %d\n", command, ret);
	}
}



void  Set_key_mode(const learning_joy::Servo_move::ConstPtr& p)
    {   
        pthread_mutex_lock(&con_mutex);
        if(p->v > 0)
        {
            key_press = -1;
            ZAux_Direct_SetSpeed(handle, anix, p->v*3); //设置轴 0 速度为 200units/s
        }
        else if(p->v < 0)
        {
            key_press = 1;
            ZAux_Direct_SetSpeed(handle, anix, -p->v*5); //设置轴 0 速度为 200units/s
        }
        else{
            key_press = 0;
        }
        cout <<"callback set: " << key_press << endl;
        pthread_mutex_unlock(&con_mutex);
        
    }

void* move_continus(void* arg)
{
    int i = 5;
    while(ros::ok())
    {
        pthread_mutex_lock(&con_mutex);
        if(key_press ==-1)
            ret =ZAux_Direct_Single_MoveAbs( handle,  anix, -1);
        else if(key_press == 1)
            ret =ZAux_Direct_Single_MoveAbs( handle,  anix, 1);
        else
            ret =ZAux_Direct_Single_MoveAbs( handle,  anix, 0);
        cout <<  "电机运动中： "<<key_press << endl;
        pthread_mutex_unlock(&con_mutex);
        ros::spinOnce();  // 处理回调函数
        sleep(0.01);
    }
    return nullptr;
}



int main(int argc, char** argv)
{   

     /**
     * @brief 
     * ros 节点配置
     */
    

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Control_servo");
    ros::Subscriber sub;
    ros::NodeHandle _nh;
    // 消息订阅
    sub= _nh.subscribe<learning_joy::Servo_move>("/servo_move/command",1,Set_key_mode);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    /**
     * @brief 
     * 电机参数连接配置
     * 
     */
    // ip address connection
    ret =  ZAux_OpenEth(ipaddr,&handle);
    cout << ret << endl;
    if(ret!=ERR_SUCCESS)
    {
        cout <<  "error, failed connect controller!" << endl;
    }else
    {
        cout << "success connect the controller!" << endl;
    }
    
    ZAux_Direct_SetAtype(handle,anix,1); 
    ZAux_Direct_SetUnits(handle,anix,100);
    ZAux_Direct_SetSpeed(handle, anix, 0.5); //设置轴 0 速度为 200units/s
    ZAux_Direct_SetAccel(handle,anix,50);
    ZAux_Direct_SetDecel(handle,anix,50);
    ZAux_Direct_SetSramp(handle,anix,50); 

    ret = ZAux_Direct_SetDpos( handle,  anix,  0);//轴指令位置清0
    commandCheckHandler("ZAux_Direct_SetDpos", ret);//判断指令是否执行成功
    ret = ZAux_Direct_SetMpos( handle,  anix,  0);//编码器反馈位置位置清0
    commandCheckHandler("ZAux_Direct_SetMpos", ret);//判断指令是否执行成功
    ZAux_Trigger(handle);//示波器触发函数

   
    /**
     * @brief 
     * 创建电机运行线程
     */
    pthread_mutex_init(&con_mutex,NULL);
    // pthread_t tid;
    // // joy_con.move_continus();
    // int ret = pthread_create(&tid,NULL,move_continus,NULL);  // 创建一个子线程，运行电机运动
    ros::Rate loop_rate(125);
    while(ros::ok())
    {   
        pthread_mutex_lock(&con_mutex);
        if(key_press == 1)
            ret =ZAux_Direct_Single_Vmove( handle,  anix, 1);
        else if(key_press == -1)
            ret =ZAux_Direct_Single_Vmove( handle,  anix, -1);
        else
            ret =ZAux_Direct_Single_Cancel(handle, anix, 2);
        cout <<  "main,电机运动中： "<<key_press << endl;
        pthread_mutex_unlock(&con_mutex);
        ros::spinOnce(); // 处理消息
        loop_rate.sleep();
    }
   
    
    // pthread_detach(tid);  // 线程资源释放到系统
    pthread_exit(NULL);   //  主线程退出

    pthread_mutex_destroy(&con_mutex);
    return 0; 
}