#include <iostream>

#include "zmcaux.h"
#include "zmotion.h"
#include <unistd.h>

using namespace std;

void commandCheckHandler(const char *command, int ret)
{
	if (ret)//非0则失败
	{
		printf("%s fail!return code is %d\n", command, ret);
	}

}

char* ipaddr= (char*)"192.168.1.11";


int main()
{
    ZMC_HANDLE handle = NULL;
    // ip address connection
    int ret =  ZAux_OpenEth(ipaddr,&handle);
    cout << ret << endl;
    if(ret!=ERR_SUCCESS)
    {
        cout <<  "error, failed connect controller!" << endl;
    }else{
        cout << "success connect the controller!" << endl;
    }
    
    int* state = NULL;
    int anix = 2;
    // move the anxis
    ZAux_Direct_SetAtype(handle,anix,1); // 
    ZAux_Direct_SetUnits(handle,anix,100);
    ZAux_Direct_SetSpeed(handle, anix, 10); //设置轴 0 速度为 200units/s
    ZAux_Direct_SetAccel(handle,anix,2000);
    ZAux_Direct_SetDecel(handle,anix,2000);
    ZAux_Direct_SetSramp(handle,anix,200); 

    ret = ZAux_Direct_SetDpos( handle,  0,  0);//轴指令位置清0
	commandCheckHandler("ZAux_Direct_SetDpos", ret);//判断指令是否执行成功
	ret = ZAux_Direct_SetMpos( handle,  0,  0);//编码器反馈位置位置清0
	commandCheckHandler("ZAux_Direct_SetMpos", ret);//判断指令是否执行成功

    ret = ZAux_Direct_Single_Move(handle,anix,10);
    commandCheckHandler("ZAux_Direct_SetDpos", ret);//判断指令是否执行成功


    ret = ZAux_Close(handle);
    // cout << ret << endl;
    if (ret)//非0则失败
	{
		printf("error , return code is %d\n", ret);
	}
    return 0;
}
