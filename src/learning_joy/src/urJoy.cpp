#include <iostream>
#include "learning_joy/Ur_move.h"
#include "zmcaux.h"
#include "zmotion.h"
#include <unistd.h>
#include <ros/ros.h>
#include <pthread.h>
#include <fstream>
#include <time.h>

using namespace std;

float pre_state[7] = {0,0,0,0,0,0,0};
float cur_state[7] = {0,0,0,0,0,0,0};

pthread_mutex_t con_mutex;  // 定义锁

void  Set_key_mode(const learning_joy::Ur_move::ConstPtr& p)
    {   
        pthread_mutex_lock(&con_mutex);
        // std::cout << p << std::endl; 
        cur_state[0] = p->x;
        cur_state[1] = p->y;
        cur_state[2] = p->z;
        cur_state[3] = p->rx;
        cur_state[4] = p->ry;
        cur_state[5] = p->rz;
        cur_state[6] = p->state;
        
        
        pthread_mutex_unlock(&con_mutex);
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
    ros::Publisher pub;
    ros::NodeHandle _nh;
    // 消息订阅
    sub= _nh.subscribe<learning_joy::Ur_move>("/axis",1,Set_key_mode);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pub = _nh.advertise<learning_joy::Ur_move>("/urState",1);
    
    

    

   
    /**
     * @brief 
     * 创建电机运行线程
     * 同时将速度，时间写入文件
     */
    pthread_mutex_init(&con_mutex,NULL);
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        pthread_mutex_lock(&con_mutex);
        // 在这里处理数据
        learning_joy::Ur_move data;
        for(int i =0; i< 6; i++){
            if(cur_state[i] != pre_state[i]){
                // 如果不同，更新之前的值
                for(int j = 0; j < 6; j++){
                    pre_state[j] = cur_state[j];
                }
                pre_state[6] = 1.0;
                break;
            }else{
                pre_state[6] = 0.0;
            }
        }

        data.x = pre_state[0];
        data.y = pre_state[1];
        data.z = pre_state[2];
        data.rx = pre_state[3];
        data.ry = pre_state[4];
        data.rz = pre_state[5];
        data.state = pre_state[6];
        cout <<"发布的数据" << data << endl;
        pub.publish(data);
        pthread_mutex_unlock(&con_mutex);
        ros::spinOnce(); // 处理消息
        loop_rate.sleep();
    }
   
    
    // pthread_detach(tid);  // 线程资源释放到系统
    pthread_exit(NULL);   //  主线程退出

    pthread_mutex_destroy(&con_mutex);
    
    return 0; 
}