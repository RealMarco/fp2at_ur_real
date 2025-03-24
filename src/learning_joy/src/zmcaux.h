/********************************** ZMC系列控制器  ************************************************
**--------------文件信息--------------------------------------------------------------------------------
**文件名: zmcaux.h
**创建人: 郑孝洋
**时间: 20130621
**描述: ZMCDLL 辅助函数

**------------修订历史记录----------------------------------------------------------------------------

** 修改人: zxy
** 版  本: 1.1
** 日　期: 2014.5.11
** 描　述: ZMC_ExecuteNoAck 替换为 ZMC_Execute


** 修改人: zxy
** 版  本: 1.3
** 日　期: 2014.7.21
** 描　述: ZMC_Execute ZMC_DirectCommand 替换为ZAux_Execute ZAux_DirectCommand

增加 ZAux_SetParam  ZAux_GetParam  ZAux_Direct_SetParam  ZAux_Direct_GetParam

增加 ZAux_WriteUFile  ZAux_ReadUFile

** 修改人: wy
** 版  本: 1.5
** 日　期: 2016.6.6
** 描　述: 对所有BASIC指令进行封装，整合ZMC库到AUX库


  ** 修改人: wy
** 版  本: 2.1
** 日　期: 2018.8.24
** 描  述：添加PCI链接函数
**		   对所有BASIC指令运动指令进行封装，封装轴列表到函数
**		   增加部分总线指令
**		   增加部分MOVE_PARA指令
           增加位置比较输出指令
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/




#ifndef _ZMOTION_DLL_AUX_H
#define _ZMOTION_DLL_AUX_H


//ZAUX支持的最大轴数宏
#define MAX_AXIS_AUX   128
#define MAX_CARD_AUX   32


#ifdef __cplusplus
extern "C" {
#endif





/*********************************************************
数据类型定义
**********************************************************/

//typedef unsigned __int64   uint64;
//typedef __int64   int64;


//#define BYTE           INT8U
//#define WORD           INT16U
//#define DWORD          INT32U
typedef unsigned char  BYTE;
typedef unsigned short  WORD;
//typedef unsigned int  DWORD;
//#define 
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable     无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable        有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable     无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable         有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable     无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable         有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */
typedef unsigned int   uint;                  /* defined for unsigned 32-bits integer variable     无符号32位整型变量 */

typedef  void* ZMC_HANDLE;
/************************************************/
//错误码
/************************************************/
#define ERR_OK  0
#define ERROR_OK 0
#define ERR_SUCCESS  0

#define ERR_AUX_OFFSET       30000

#define ERR_NOACK               ERR_AUX_OFFSET      //无应答
#define ERR_ACKERROR            (ERR_AUX_OFFSET+1)  //应答错误
#define ERR_AUX_PARAERR         (ERR_AUX_OFFSET+2)  //参数错误
#define ERR_AUX_NOTSUPPORT      (ERR_AUX_OFFSET+3)  //不支持

#define ERR_AUX_FILE_ERROR      (ERR_AUX_OFFSET+4)  //参数错误
#define ERR_AUX_OS_ERR      (ERR_AUX_OFFSET+5)  //

#if 0
    //错误输出部分
#endif


//#define  ZAUX_DEBUG //不需要输出时注释掉这行
//#define  ZAUX_DEBUG_FILE "d:\\zauxout.txt"

#define ZAUX_COMMAND_SUM  100		//写命令数，调试时使用

#ifdef  ZAUX_DEBUG

#define ZAUX_TRACE(cText,a)     ZAux_TraceOut("%s,L:%u,"cText"\r\n",__FILE__,(unsigned int)__LINE__,a)
#define ZAUX_WARN(cText,a)      ZAux_TraceOut("%s,L:%u,"cText"\r\n",__FILE__,(unsigned int)__LINE__,a)
#define ZAUX_ERROR(cText,a)     ZAux_TraceOut("Error:%s,L:%u,"cText"\r\n",__FILE__,(unsigned int)__LINE__,a)
#define ZAUX_ERROR2(cText,a,b)     ZAux_TraceOut("Error:%s,L:%u,"cText"\r\n",__FILE__,(unsigned int)__LINE__,a,b)

#else

#define  ZAUX_TRACE(cText,a)
#define  ZAUX_WARN(cText,a)
#define  ZAUX_ERROR(cText,a)
#define  ZAUX_ERROR2(cText,a,b)

#endif

void  ZAux_TraceOut(const char * cText, ...);



/*********************************************************
函数声明
**********************************************************/


/*************************************************************
Description:    //封装 Excute 函数, 以便接收错误
Input:          //卡链接			handle
Input:          //字符串命令		pszCommand
Input:			//返回的字符长度	uiResponseLength
Output:         //返回的字符串		psResponse
Return:         //错误码
*************************************************************/
int32  ZAux_Execute(ZMC_HANDLE handle, const char* pszCommand, char* psResponse, uint32 uiResponseLength);


/*************************************************************
Description:    //封装 DirectCommand 函数, 以便接收错误
Input:          //卡链接			handle
Input:          //字符串命令		pszCommand
Input:			//返回的字符长度	uiResponseLength
Output:         //返回的字符串		psResponse
Return:         //错误码
*************************************************************/
int32  ZAux_DirectCommand(ZMC_HANDLE handle, const char* pszCommand, char* psResponse, uint32 uiResponseLength);

/*************************************************************
Description:    //命令跟踪设置.
Input:          //卡链接handle
bifTofile		0 关闭  1-只输出错误命令  2-只输出运动与设置命令  3输出全部命令
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_SetTraceFile(int bifTofile, const char *pFilePathName);




/*************************************************************
Description:    //与控制器建立链接， 串口方式.
Input:          //串口号COMId
Output:         //卡链接phandle
Return:         //错误码
*************************************************************/
int32  ZAux_OpenCom(uint32 comid, ZMC_HANDLE * phandle);

/*************************************************************
Description:    //快速控制器建立链接
Input:          //最小串口号uimincomidfind
Input:          //最大串口号uimaxcomidfind
Input:          //链接时间uims
Output:         //有效COM pcomid
Output:         //卡链接handle
Return:         //错误码
*************************************************************/
int32  ZAux_SearchAndOpenCom(uint32 uimincomidfind, uint32 uimaxcomidfind,uint* pcomid, uint32 uims, ZMC_HANDLE * phandle);

/*************************************************************
Description:    //可以修改缺省的波特率等设置
Input:          //dwBaudRate 波特率
                dwByteSize   数据位
                dwParity = NOPARITY,校验位
                dwStopBits = ONESTOPBIT停止位
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_SetComDefaultBaud(uint32 dwBaudRate, uint32 dwByteSize, uint32 dwParity, uint32 dwStopBits);

/*************************************************************
Description:    //修改控制器IP地址
Input:          //卡链接handle
Input:          //IP地址
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_SetIp(ZMC_HANDLE handle, char * ipaddress);

/*************************************************************
Description:    //与控制器建立链接
Input:          //IP地址，字符串的方式输入
Output:         //卡链接handle
Return:         //错误码
*************************************************************/
int32   ZAux_OpenEth(char *ipaddr, ZMC_HANDLE * phandle);


/*************************************************************
Description:    //快速检索IP列表
Input:          //uims 响应时间
Input:          //addrbufflength		最大长度
output:			//ipaddrlist		当前晚点IP列表
Return:         //错误码, ERR_OK表示有搜索到.
*************************************************************/
int32  ZAux_SearchEthlist(char *ipaddrlist, uint32 addrbufflength, uint32 uims);


/*************************************************************
Description:    //快速检索控制器
Input:          //控制器IP地址
Input:          //响应时间
Output:         //
Return:         //错误码, ERR_OK表示有搜索到.
*************************************************************/
int32  ZAux_SearchEth(const char *ipaddress,  uint32 uims);

/*************************************************************
Description:    //关闭控制器链接
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Close(ZMC_HANDLE  handle);

/*************************************************************
Description:    //暂停继续运行BAS项目
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Resume(ZMC_HANDLE handle);

/*************************************************************
Description:    //暂停控制器中BAS程序
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Pause(ZMC_HANDLE handle);

/*************************************************************
Description:    //单个BAS文件生成ZAR并且下载到控制器运行
Input:          //卡链接handle
Input:          //Filename BAS文件路径
Input:          //RAM-ROM  0-RAM  1-ROM
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BasDown(ZMC_HANDLE handle,const char *Filename,uint32 run_mode);




#if 0
    //IO指令
    // 可以使用 ZMC_GetIn ZMC_GetOutput 等
#endif
/*************************************************************
Description:    //读取输入信号
Input:          //卡链接handle
                ionum IN编号
Output:         //piValue 输入口状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetIn(ZMC_HANDLE handle, int ionum , uint32 *piValue);


/*************************************************************
Description:    //打开输出信号
Input:          //卡链接handle
                ionum 输出口编号
                iValue	输出口状态
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetOp(ZMC_HANDLE handle, int ionum, uint32 iValue);

/*************************************************************
Description:    //读取输出口状态
Input:          //卡链接handle
                ionum 输出口编号
Output:         //piValue 输出口状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetOp(ZMC_HANDLE handle, int ionum, uint32 *piValue);


/*************************************************************
Description:    //读取模拟量输入信号
Input:          //卡链接handle
                ionum AIN口编号
Output:         //pfValue 返回的模拟量值 4系列以下0-4095
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAD(ZMC_HANDLE handle, int ionum , float *pfValue);

/*************************************************************
Description:    //打开模拟量输出信号
Input:          //卡链接handle
                ionum DA输出口编号
                fValue 设定的模拟量值4系列以下0-4095
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDA(ZMC_HANDLE handle, int ionum, float fValue);


/*************************************************************
Description:    //读取模拟输出口状态
Input:          //卡链接handle
                ionum 模拟量输出口编号
Output:         //pfValue 读取的的模拟量值 4系列以下0-4095
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDA(ZMC_HANDLE handle, int ionum, float *pfValue);

/*************************************************************
Description:    //设置输入口反转
Input:          //卡链接handle
                 ionum 输入口编号
                 bifInvert 反转状态 0/1
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetInvertIn(ZMC_HANDLE handle, int ionum, int bifInvert);

/*************************************************************
Description:    //读取输入口反转状态
Input:          //卡链接handle
                ionum 输入口编号
Output:         //piValue 反转状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetInvertIn(ZMC_HANDLE handle, int ionum, int *piValue);

/*************************************************************
Description:    //设置pwm频率
Input:          //卡链接handle
                ionum PWM编号口
                fValue 频率 硬件PWM1M 软PWM 2K
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetPwmFreq(ZMC_HANDLE handle, int ionum, float fValue);

/*************************************************************
Description:    //读取pwm频率
Input:          //卡链接handle
                ionum PWM口编号
Output:         //pfValue 返回的频率
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetPwmFreq(ZMC_HANDLE handle, int ionum, float *pfValue);

/*************************************************************
Description:    //设置pwm占空比
Input:          //卡链接handle
                ionum PWM口编号
                fValue 占空变	0-1  0表示关闭PWM口
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetPwmDuty(ZMC_HANDLE handle, int ionum, float fValue);

/*************************************************************
Description:    //读取pwm占空比
Input:          //卡链接handle
                ionum PWM口编号
Output:         //pfValue 读取的占空比
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetPwmDuty(ZMC_HANDLE handle, int ionum, float *pfValue);

#if 0
    //通过modbus快速读取特殊寄存器
#endif

/*************************************************************
Description:    //参数 快速读取多个输入
Input:          //卡链接handle
                ionumfirst IN起始编号
                ionumend	IN结束编号
Output:         //pValueList 位状态 按位存储
Return:         //错误码
*************************************************************/
int32  ZAux_GetModbusIn(ZMC_HANDLE handle, int ionumfirst, int ionumend, uint8 *pValueList);


/*************************************************************
Description:    //参数 快速读取多个当前的输出状态
Input:          //卡链接handle
                ionumfirst IN起始编号
                ionumend	IN结束编号
Output:         //pValueList 位状态 按位存储
Return:         //错误码
*************************************************************/
int32  ZAux_GetModbusOut(ZMC_HANDLE handle, int ionumfirst, int ionumend, uint8 *pValueList);


/*************************************************************
Description:    //参数 快速读取多个当前的DPOS
Input:          //卡链接handle
                imaxaxises 轴数量
Output:         //pValueList 读取的坐标值 从轴0开始
Return:         //错误码
*************************************************************/
int32  ZAux_GetModbusDpos(ZMC_HANDLE handle, int imaxaxises, float *pValueList);


/*************************************************************
Description:    //参数 快速读取多个当前的MPOS
Input:          //卡链接handle
                imaxaxises 轴数量
Output:         //pValueList 读取的反馈坐标值 从轴0开始
Return:         //错误码
*************************************************************/
int32  ZAux_GetModbusMpos(ZMC_HANDLE handle, int imaxaxises, float *pValueList);


/*************************************************************
Description:    //参数 快速读取多个当前的速度
Input:          //卡链接handle
                imaxaxises 轴数量
Output:         //pValueList 读取的当前速度 从轴0开始
Return:         //错误码
*************************************************************/
int32  ZAux_GetModbusCurSpeed(ZMC_HANDLE handle, int imaxaxises, float *pValueList);



#if 0
    //采用ZMC_DirectCommand 来快速获取一些状态, ZMC_DirectCommand的执行比ZMC_Execute要快
    // 只有参数，变量，数组元素等能使用ZMC_DirectCommand
    // 20130901以后的版本，一些运动函数也可以调用ZMC_DirectCommand，当运动条件不满足的时候，会立刻返回失败。
    // ZMC_DirectCommand调用运动函数时，参数必须是具体的数值，不能是变量表达式。
#endif

/*************************************************************
Description:    //通用的参数修改函数 sParam: 填写参数名称
Input:          //卡链接handle
                sParam 轴参数名称 "DPOS" ...
                iaxis 轴号
                fset 设定值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetParam(ZMC_HANDLE handle,const char *sParam,int iaxis, float fset);

/*************************************************************
Description:    //参数 通用的参数读取函数, sParam:填写参数名称
Input:          //卡链接handle
                sParam 轴参数名称 "DPOS" ...
                iaxis 轴号
Output:         //pfValue  读取的返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetParam(ZMC_HANDLE handle,const char *sParam, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置加速度
Input:          //卡链接handle
                iaxis 轴号
                fValue  设定值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetAccel(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取加速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 加速度返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAccel(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取叠加轴
Input:          //卡链接handle
                iaxis 轴号
Output:         //读取的轴叠加轴号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAddax(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置轴告警信号
Input:          //卡链接handle
                iaxis 轴号
                iValue 报警信号输入口编号，取消时设定-1
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetAlmIn(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取告警信号
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 报警信号输入口返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAlmIn(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置轴类型
Input:          //卡链接handle
                iaxis 轴号
                iValue 轴类型
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetAtype(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取轴类型
Input:          //卡链接handle
                iaxis 轴号
Output:         //iValue 轴类型返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAtype(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取轴状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //轴状态返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAxisStatus(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置轴地址
Input:          //卡链接handle
                iaxis 轴号
                iValue 轴地址设定值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetAxisAddress(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取轴地址
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 轴地址返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAxisAddress(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置轴使能 （只针对总线控制器轴使用有效）
Input:          //卡链接handle
                iaxis 轴号
                iValue 状态 0-关闭 1- 打开
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetAxisEnable(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取轴使能状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的使能状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAxisEnable(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置链接速率
Input:          //卡链接handle
                iaxis 轴号
                fValue 同步连接速率
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetClutchRate(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取链接速率
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 连接速率返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetClutchRate(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置锁存触发的结束坐标范围点
Input:          //卡链接handle
                iaxis 轴号
                fValue 设定的范围值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetCloseWin(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取锁存触发的结束坐标范围点
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的范围值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetCloseWin(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置拐角减速
Input:          //卡链接handle
                iaxis 轴号
                iValue 拐角减速模式
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetCornerMode(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取拐角减速
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的拐角模式
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetCornerMode(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置回零爬行速度
Input:          //卡链接handle
                iaxis 轴号
                fValue设置的速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetCreep(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取回零爬行速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的爬行速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetCreep(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置原点信号   设定-1为取消原点设置
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的原点信号输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDatumIn(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取原点信号
Input:          //卡链接handle
                iaxis
Output:         //piValue 返回原点输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDatumIn(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置减速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的减速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDecel(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取减速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 设定的减速度返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDecel(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置拐角减速角度，开始减速角度，单位为弧度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的拐角减速角度
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDecelAngle(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取拐角开始减速角度，单位为弧度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的拐角减速角度
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDecelAngle(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置轴位置
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的坐标值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDpos(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取轴位置
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的命令位置坐标
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDpos(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取内部编码器值  （总线绝对值伺服时为绝对值位置）
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的内部编码器值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetEncoder(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取当前运动的最终位置
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的最终位置
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetEndMove(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取当前和缓冲中运动的最终位置，可以用于相对绝对转换
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的最终位置
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetEndMoveBuffer(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置SP运动的结束速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //fValue 设定的速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetEndMoveSpeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取SP运动的结束速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetEndMoveSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置错误标记，和AXISSTATUS做与运算来决定哪些错误需要关闭WDOG。
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetErrormask(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取错误标记，和AXISSTATUS做与运算来决定哪些错误需要关闭WDOG。
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的标记值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetErrormask(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置快速JOG输入
Input:          //卡链接handle
                iaxis 轴号
                iValue 快速JOG输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFastJog(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取快速JOG输入
Input:          //卡链接handle
                iaxis 轴号
Output:         //返回的JOG输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFastJog(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置快速减速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设定的快速减速度
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFastDec(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取快速减速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的快速减速度
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFastDec(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取随动误差
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的随动误差
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFe(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置最大允许的随动误差值
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的最大误差值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFeLimit(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取最大允许的随动误差值
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的设置最大误差值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFeLimit(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置报警时随动误差值
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的误差值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFRange(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取报警时的随动误差值
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的报警误差值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFeRange(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置保持输入
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFholdIn(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取保持输入
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回输入HOLDIN输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFholdIn(ZMC_HANDLE handle, int iaxis, int *piValue);


/*************************************************************
Description:    //设置轴保持速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFhspeed(ZMC_HANDLE handle, int iaxis, float pfValue);

/*************************************************************
Description:    //读取轴保持速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的保持速度
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFhspeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置SP运动的运行速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetForceSpeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取SP运动的运行速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回SP运动速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetForceSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置正向软限位		取消时设置一个较大值即可
Input:          //卡链接handle
                iaxis 轴号
                fValue 设定的限位值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFsLimit(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取正向软限位
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的正向限位坐标
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置小圆限速最小半径
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的最小半径
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFullSpRadius(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取小圆限速最小半径
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的限速半径
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFullSpRadius(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置正向硬限位输入  设置成-1时表示不设置限位
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的限位输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFwdIn(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取正向硬限位输入
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回正向限位输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFwdIn(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置正向JOG输入
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的JOG输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetFwdJog(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取正向JOG输入
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的JOG输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetFwdJog(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取轴是否运动结束
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回运行状态 0-运动中 -1 停止
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetIfIdle(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置脉冲输出模式
Input:          //卡链接handle
                iaxis 轴号
                iValue 设定的脉冲输出模式 脉冲+方向/双脉冲
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetInvertStep(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取脉冲输出模式
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的脉冲模式
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetInvertStep(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置插补时轴是否参与速度计算，缺省参与（1）。此参数只对直线和螺旋的第三个轴起作用
Input:          //卡链接handle
                iaxis	轴号
                iValue 模式 0-不参数 1-参与
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetInterpFactor(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取插补时轴是否参与速度计算，缺省参与（1）。此参数只对直线和螺旋的第三个轴起作用
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的速度计算模式
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetInterpFactor(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置JOG时速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //fValue 设定的速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetJogSpeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取JOG时速度
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue 返回的JOG速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetJogSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取当前链接运动的参考轴号
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回链接的参考轴号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetLinkax(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取当前除了当前运动是否还有缓冲
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回状态值  -1 没有剩余函数 0-还有剩余运动
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetLoaded(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置轴起始速度
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetLspeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取轴起始速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的起始速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetLspeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置回零反找等待时间
Input:          //卡链接handle
                iaxis 轴号
                iValue 回零反找等待时间 MS
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetHomeWait(ZMC_HANDLE handle, int iaxis, int fValue);

/*************************************************************
Description:    //读取回零反找等待时间
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的反找等待时间
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetHomeWait(ZMC_HANDLE handle, int iaxis, int *pfValue);


/*************************************************************
Description:    //读取编码器锁存示教返回状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue  返回的锁存触发状态 -1-锁存触发 0-未触发
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMark(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取编码器锁存b返回状态
Input:          //卡链接handle
iaxis 轴号
Output:         //piValue  返回的锁存触发状态 -1-锁存触发 0-未触发
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMarkB(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置脉冲输出最高频率
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的最高脉冲频率
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetMaxSpeed(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取脉冲输出最高频率
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的脉冲频率
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMaxSpeed(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置连续插补
Input:          //卡链接handle
                iaxis 轴号
                iValue	连续插补开关 0-关闭连续插补 1-打开连续插补
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetMerge(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取连续插补状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的连续插补开关状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMerge(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取当前被缓冲起来的运动个数
Input:          //卡链接handle
                iaxis 轴数
Output:         //piValue 缓冲运动数
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMovesBuffered(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取当前正在运动指令的MOVE_MARK标号
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 当前MARK标号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMoveCurmark(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置运动指令的MOVE_MARK标号 每当有运动进入轴运动缓冲时MARK自动+1
Input:          //卡链接handle
                iaxis 轴号
                iValue 设定的MARK值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetMovemark(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //设置反馈位置
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的反馈位置
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetMpos(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取反馈位置
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的轴反馈位置坐标
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMpos(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取反馈速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的编码器反馈速度
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMspeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取当前正在运动指令类型
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回当前的运动类型
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetMtype(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //读取当前正在进行的运动指令后面的第一条指令类型，当插补联动时，对从轴总是返回主轴的运动指令类型
Input:          //卡链接handle
                iaxis  轴号
Output:         //piValue 返回下一条指令的运动类型
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetNtype(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置修改偏移位置
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的偏移值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetOffpos(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取修改偏移位置
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的偏移坐标值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetOffpos(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置锁存触发的结束坐标范围点。
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的坐标值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetOpenWin(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取锁存触发的结束坐标范围点。
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的结束坐标值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetOpenWin(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取返回锁存的测量反馈位置(MPOS)
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 锁存的坐标位置
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRegPos(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取返回锁存的测量反馈位置(MPOS)
Input:          //卡链接handle
iaxis 轴号
Output:         //pfValue 锁存的坐标位置
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRegPosB(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取返回轴当前运动还未完成的距离
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的剩余距离
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRemain(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //参数  轴剩余的缓冲, 按直线段来计算
REMAIN_BUFFER为唯一一个可以加AXIS并用ZAux_DirectCommand获取的.
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 剩余的直线缓冲数量
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRemain_LineBuffer(ZMC_HANDLE handle, int iaxis,int * piValue);

/*************************************************************
Description:    //参数  轴剩余的缓冲, 按最复杂的空间圆弧来计算
REMAIN_BUFFER为唯一一个可以加AXIS并用ZAux_DirectCommand获取的.
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 剩余的缓冲数量
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRemain_Buffer(ZMC_HANDLE handle, int iaxis, int *piValue);


/*************************************************************
Description:    //设置根据REP_OPTION设置来自动循环轴DPOS和MPOS坐标。
Input:          //卡链接handle
                iaxis 轴号
                fValue	设置的坐标值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetRepDist(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取根据REP_OPTION设置来自动循环轴DPOS和MPOS坐标。
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue 返回的循环坐标值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRepDist(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置坐标重复设置
Input:          //卡链接handle
                iaxis 轴号
                iValue 模式
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetRepOption(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取坐标重复设置
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的模式
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRepOption(ZMC_HANDLE handle, int iaxis, int *piValue);


/*************************************************************
Description:    //设置负向硬件限位开关对应的输入点编号，-1无效。
Input:          //卡链接handle
                iaxis  轴号
                iValue 设置的输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetRevIn(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取负向硬件限位开关对应的输入点编号，-1无效。
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的负向限位输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRevIn(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置负向JOG输入对应的输入点编号，-1无效。
Input:          //卡链接handle
                iaxis 轴号
                iValue 设置的输入口编号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetRevJog(ZMC_HANDLE handle, int iaxis, int iValue);

/*************************************************************
Description:    //读取负向JOG输入对应的输入点编号，-1无效。
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的输入口编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRevJog(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置负向软限位位置。  设置一个较大的值时认为取消限位
Input:          //卡链接handle
                iaxis	轴号
                fValue  负向限位值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetRsLimit(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取负向软限位位置。
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue 设定的限位值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetRsLimit(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置轴速度，单位为units/s，当多轴运动时，作为插补运动的速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetSpeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取轴速度，单位为units/s，当多轴运动时，作为插补运动的速度
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置 S曲线设置。 0-梯形加减速
Input:          //卡链接handle
                iaxis 轴号
                fValue S曲线平滑时间MS
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetSramp(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取 S曲线设置。
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 平滑时间
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetSramp(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置 自定义速度的SP运动的起始速度
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的速度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetStartMoveSpeed(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取自定义速度的SP运动的起始速度
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue 返回的SP运动起始速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetStartMoveSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置 减速到最低的最小拐角 弧度制
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的角度值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetStopAngle(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取减速到最低的最小拐角 弧度制
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的拐角停止角度
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetStopAngle(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置 减速倒角半径
Input:          //卡链接handle
                iaxis	轴号
                fValue	倒角半径
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetZsmooth(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取倒角半径
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue	返回的倒角半径值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetZsmooth(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //设置 脉冲当量
Input:          //卡链接handle
                iaxis 轴号
                fValue 设置的当量值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetUnits(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取脉冲当量
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue 返回的脉冲当量
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetUnits(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取返回轴当前当前运动和缓冲运动还未完成的距离
Input:          //卡链接handle
                iaxis 轴号
Output:         //pfValue 返回的阵雨距离
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVectorBuffered(ZMC_HANDLE handle, int iaxis, float *pfValue);

/*************************************************************
Description:    //读取当前轴运行的命令速度
Input:          //卡链接handle
                iaxis	轴号
Output:         //pfValue	返回的当前速度值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVpSpeed(ZMC_HANDLE handle, int iaxis, float *pfValue);




/*************************************************************
Description:    //全局变量读取, 也可以是参数等等
Input:          //卡链接handle
                pname 全局变量名称/或者指定轴号的轴参数名称DPOS(0)
Output:         //pfValue 返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVariablef(ZMC_HANDLE handle, const char *pname, float *pfValue);


/*************************************************************
Description:    //全局变量读取, 也可以是参数等等
Input:          //卡链接handle
                pname 全局变量名称/或者指定轴号的轴参数名称DPOS(0)
Output:         //piValue 返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVariableInt(ZMC_HANDLE handle, const char *pname, int *piValue);

///////////////////////  只有下面的运动函数支持直接调用，并不是所有的指令都支持
///////////////////////  必须 20130901 以后的控制器版本支持

/*************************************************************
Description:    //BASE指令调用
仅仅修改在线命令的BASE列表，不对控制器的运行任务的BASE进行修改.
修改后，后续的所有MOVE等指令都是以这个BASE为基础
Input:          //卡链接handle
                imaxaxises 参与轴数
                piAxislist	轴列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Base(ZMC_HANDLE handle, int imaxaxises, int *piAxislist);

/*************************************************************
Description:    //定义DPOS,不建议使用，可以直接调用SETDPOS达到同样效果
Input:          //卡链接handle
                iaxis	轴号
                pfDpos 设置的坐标值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Defpos(ZMC_HANDLE handle, int iaxis, float pfDpos);

/*************************************************************
Description:    //多轴相对直线插补  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Move(ZMC_HANDLE handle, int imaxaxises, int *piAxislist,float *pfDisancelist);


/*************************************************************
Description:    //相对多轴直线插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist);



/*************************************************************
Description:    //绝对多轴直线插补  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist);


/*************************************************************
Description:    //绝对多轴直线插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float *pfDisancelist);



/*************************************************************
//Description:    //运动中修改结束位置  20130901 以后的控制器版本支持
//Input:          //卡链接handle
                    轴号 iaxis
                    绝对距离 pfDisance
//Output:         //
//Return:         //错误码
/*************************************************************/
int32  ZAux_Direct_MoveModify(ZMC_HANDLE handle, int iaxis, float pfDisance);


/*************************************************************
Description:    //相对圆心定圆弧插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心，相对与起始点。
                centre2    第二个轴运动圆心，相对与起始点。
                direction  0-逆时针，1-顺时针
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCirc(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection);


/*************************************************************
Description:    //相对圆心定圆弧插补运动 插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心，相对与起始点。
                centre2    第二个轴运动圆心，相对与起始点。
                direction  0-逆时针，1-顺时针
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCircSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection);



/*************************************************************
Description:    //绝对圆心圆弧插补运动  20130901 以后的控制器版本支持  无法画整圆
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标，绝对位置
                end2              第二个轴运动坐标，绝对位置
                centre1    第一个轴运动圆心，绝对位置
                centre2    第二个轴运动圆心，绝对位置
                direction  0-逆时针，1-顺时针
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCircAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection);


/*************************************************************
Description:    //绝对圆心圆弧插补SP运动  20130901 以后的控制器版本支持  无法画整圆
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标，绝对位置
                end2              第二个轴运动坐标，绝对位置
                centre1    第一个轴运动圆心，绝对位置
                centre2    第二个轴运动圆心，绝对位置
                direction  0-逆时针，1-顺时针
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCircAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection);




/*************************************************************
Description:    //相对3点定圆弧插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点，相对起始点距离
                mid2       第二个轴中间点，相对起始点距离
                end1              第一个轴结束点，相对起始点距离
                end2              第二个轴结束点，相对起始点距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCirc2(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2);



/*************************************************************
Description:    //绝对3点定圆弧插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点，绝对位置
                mid2       第二个轴中间点，绝对位置
                end1              第一个轴结束点，绝对位置
                end2              第二个轴结束点，绝对位置
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCirc2Abs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2);


/*************************************************************
Description:    //相对3点定圆弧插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点，相对起始点距离
                mid2       第二个轴中间点，相对起始点距离
                end1              第一个轴结束点，相对起始点距离
                end2              第二个轴结束点，相对起始点距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCirc2Sp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2);



/*************************************************************
Description:    //绝对3点定圆弧插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点，绝对位置
                mid2       第二个轴中间点，绝对位置
                end1              第一个轴结束点，绝对位置
                end2              第二个轴结束点，绝对位置
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveCirc2AbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2);


/*************************************************************
Description:    //相对3轴圆心螺旋插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心，相对与起始点
                centre2    第二个轴运动圆心，相对与起始点
                direction  0-逆时针，1-顺时针
                distance3第三个轴运动距离。
                mode      第三轴的速度计算:0(缺省)第三轴参与速度计算。1第三轴不参与速度计算。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode);


/*************************************************************
Description:    //绝对3轴圆心螺旋插补运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心坐标
                centre2    第二个轴运动圆心坐标
                direction  0-逆时针，1-顺时针
                distance3第三个轴运动距离。
                mode      第三轴的速度计算:0(缺省) 第三轴参与速度计算。1第三轴不参与速度计算。
Output:         //
 Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelicalAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode);


/*************************************************************
Description:    //相对3轴圆心螺旋插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心，相对与起始点
                centre2    第二个轴运动圆心，相对与起始点
                direction  0-逆时针，1-顺时针
                distance3第三个轴运动距离。
              mode      第三轴的速度计算: 0(缺省)第三轴参与速度计算。 1第三轴不参与速度计算。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelicalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode);


/*************************************************************
Description:    //绝对3轴圆心螺旋插补运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第一个轴运动坐标
                end2              第二个轴运动坐标
                centre1    第一个轴运动圆心坐标
                centre2    第二个轴运动圆心坐标
                direction  0-逆时针，1-顺时针
                distance3第三个轴运动距离。
                mode      第三轴的速度计算:0(缺省) 第三轴参与速度计算。1第三轴不参与速度计算。
Output:         //
 Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelicalAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fDistance3, int imode);



/*************************************************************
Description:    //相对3轴 3点画螺旋插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点
                mid2       第二个轴中间点
                end1              第一个轴结束点
                end2              第二个轴结束点
                distance3第三个轴运动距离
                mode      第三轴的速度计算:
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelical2(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode);


/*************************************************************
Description:    //绝对3轴 3点画螺旋插补运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
            imaxaxises			参与运动总轴数
            piAxislist			轴号列表
            mid1       第一个轴中间点
            mid2       第二个轴中间点
            end1              第一个轴结束点
            end2              第二个轴结束点
            distance3   第三个轴运动结束点
            mode      第三轴的速度计算:
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelical2Abs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode);


/*************************************************************
Description:    //相对3轴 3点画螺旋插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                mid1       第一个轴中间点
                mid2       第二个轴中间点
                end1              第一个轴结束点
                end2              第二个轴结束点
                distance3第三个轴运动距离
                mode      第三轴的速度计算:
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelical2Sp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode);


/*************************************************************
Description:    //绝对3轴 3点画螺旋插补SP运动  20130901 以后的控制器版本支持
Input:          //卡链接handle
            imaxaxises			参与运动总轴数
            piAxislist			轴号列表
            mid1       第一个轴中间点
            mid2       第二个轴中间点
            end1              第一个轴结束点
            end2              第二个轴结束点
            distance3   第三个轴运动结束点
            mode      第三轴的速度计算:
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MHelical2AbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fmid1, float fmid2, float fend1, float fend2, float fDistance3, int imode);




/*************************************************************
Description:    //相对椭圆插补 20130901 以后的控制器版本支持
Input:          //卡链接handle
            imaxaxises			参与运动总轴数
            piAxislist			轴号列表
            fend1              终点第一个轴运动坐标，相对于起始点。
            fend2              终点第二个轴运动坐标，相对于起始点。
            fcenter1    中心第一个轴运动坐标，相对于起始点。
            fcenter2    中心第二个轴运动坐标，相对于起始点。
            idirection  0-逆时针，1-顺时针
            fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
            fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipse(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis);


/*************************************************************
Description:    //绝对椭圆插补 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标
                fend2              终点第二个轴运动坐标
                fcenter1    中心第一个轴运动坐标。
                fcenter2    中心第二个轴运动坐标。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。

  Output:         //
  Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis);


/*************************************************************
Description:    //相对椭圆插补SP运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
            imaxaxises			参与运动总轴数
            piAxislist			轴号列表
            fend1              终点第一个轴运动坐标，相对于起始点。
            fend2              终点第二个轴运动坐标，相对于起始点。
            fcenter1    中心第一个轴运动坐标，相对于起始点。
            fcenter2    中心第二个轴运动坐标，相对于起始点。
            idirection  0-逆时针，1-顺时针
            fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
            fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis);


/*************************************************************
Description:    //绝对椭圆插补SP运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标
                fend2              终点第二个轴运动坐标
                fcenter1    中心第一个轴运动坐标。
                fcenter2    中心第二个轴运动坐标。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。

  Output:         //
  Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis);



/*************************************************************
Description:    //相对 椭圆 + 螺旋插补运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标，相对于起始点。
                fend2              终点第二个轴运动坐标，相对于起始点。
                fcenter1    中心第一个轴运动坐标，相对于起始点。
                fcenter2    中心第二个轴运动坐标，相对于起始点。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
                fDistance3	第三个轴的运动距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseHelical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis, float fDistance3);


/*************************************************************
Description:    //绝对椭圆 + 螺旋插补运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标
                fend2              终点第二个轴运动坐标
                fcenter1    中心第一个轴运动坐标。
                fcenter2    中心第二个轴运动坐标。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
                fDistance3	第三个轴的运动距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalAbs(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis,float fDistance3);


/*************************************************************
Description:    //相对 椭圆 + 螺旋插补SP运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标，相对于起始点。
                fend2              终点第二个轴运动坐标，相对于起始点。
                fcenter1    中心第一个轴运动坐标，相对于起始点。
                fcenter2    中心第二个轴运动坐标，相对于起始点。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
                fDistance3	第三个轴的运动距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis, float fDistance3);


/*************************************************************
Description:    //绝对椭圆 + 螺旋插补SP运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                fend1              终点第一个轴运动坐标
                fend2              终点第二个轴运动坐标
                fcenter1    中心第一个轴运动坐标。
                fcenter2    中心第二个轴运动坐标。
                idirection  0-逆时针，1-顺时针
                fADis         第一轴的椭圆半径，半长轴或者半短轴都可。
                fBDis        第二轴的椭圆半径，半长轴或者半短轴都可，AB相等时自动为圆弧或螺旋。
                fDistance3	第三个轴的运动距离
  Output:         //
  Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MEclipseHelicalAbsSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fcenter1, float fcenter2, int idirection, float fADis, float fBDis,float fDistance3);


/*************************************************************
Description:    //空间圆弧 + 螺旋插补运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第1个轴运动距离参数1	相对与起点
                end2              第2个轴运动距离参数1	相对与起点
                end3              第3个轴运动距离参数1	相对与起点
                centre1    第1个轴运动距离参数2	相对与起点
                centre2    第2个轴运动距离参数2	相对与起点
                centre3    第3个轴运动距离参数2 相对与起点
                mode      指定前面参数的意义
                    0 当前点，中间点，终点三点定圆弧，距离参数1为终点距离，距离参数2为中间点距离。
                    1 走最小的圆弧，距离参数1为终点距离，距离参数2为圆心的距离。
                    2 当前点，中间点，终点三点定圆，距离参数1为终点距离，距离参数2为中间点距离。
                    3 先走最小的圆弧，再继续走完整圆，距离参数1为终点距离，距离参数2为圆心的距离。
                fcenter4	第4个轴运动距离参数
                fcenter5	第5个轴运动距离参数
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MSpherical(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fend3, float fcenter1, float fcenter2, float fcenter3, int imode, float fcenter4, float fcenter5);


/*************************************************************
Description:    //空间圆弧 + 螺旋 插补SP运动 20130901 以后的控制器版本支持
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第1个轴运动距离参数1	相对与起点
                end2              第2个轴运动距离参数1	相对与起点
                end3              第3个轴运动距离参数1	相对与起点
                centre1    第1个轴运动距离参数2	相对与起点
                centre2    第2个轴运动距离参数2	相对与起点
                centre3    第3个轴运动距离参数2 相对与起点
                mode      指定前面参数的意义
                    0 当前点，中间点，终点三点定圆弧，距离参数1为终点距离，距离参数2为中间点距离。
                    1 走最小的圆弧，距离参数1为终点距离，距离参数2为圆心的距离。
                    2 当前点，中间点，终点三点定圆，距离参数1为终点距离，距离参数2为中间点距离。
                    3 先走最小的圆弧，再继续走完整圆，距离参数1为终点距离，距离参数2为圆心的距离。
                fcenter4	第4个轴运动距离参数
                fcenter5	第5个轴运动距离参数
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MSphericalSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float fend1, float fend2, float fend3, float fcenter1, float fcenter2, float fcenter3, int imode, float fcenter4, float fcenter5);



/*************************************************************
Description:    //渐开线圆弧插补运动，相对移动方式，当起始半径0直接扩散时从0角度开始
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                centre1: 第1轴圆心的相对距离
                centre2: 第2轴圆心的相对距离
                circles:  要旋转的圈数，可以为小数圈，负数表示顺时针.
                pitch:   每圈的扩散距离，可以为负。
                distance3        第3轴螺旋的功能，指定第3轴的相对距离，此轴不参与速度计算。
                distance4        第4轴螺旋的功能，指定第4轴的相对距离，此轴不参与速度计算。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSpiral(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float centre1, float centre2, float circles, float pitch, float distance3, float distance4);

/*************************************************************
Description:    //渐开线圆弧插补SP运动，相对移动方式，当起始半径0直接扩散时从0角度开始
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                centre1: 第1轴圆心的相对距离
                centre2: 第2轴圆心的相对距离
                circles:  要旋转的圈数，可以为小数圈，负数表示顺时针.
                pitch:   每圈的扩散距离，可以为负。
                distance3        第3轴螺旋的功能，指定第3轴的相对距离，此轴不参与速度计算。
                distance4        第4轴螺旋的功能，指定第4轴的相对距离，此轴不参与速度计算。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSpiralSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float centre1, float centre2, float circles, float pitch, float distance3, float distance4);

/*************************************************************
Description:    //空间直线运动，根据下一个直线运动的绝对坐标在拐角自动插入圆弧，加入圆弧后会使得运动的终点与直线的终点不一致，拐角过大时不会插入圆弧，当距离不够时会自动减小半径
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第1个轴运动绝对坐标
                end2              第2个轴运动绝对坐标
                end3              第3个轴运动绝对坐标
                next1      第1个轴下一个直线运动绝对坐标
                next2      第2个轴下一个直线运动绝对坐标
                next3      第3个轴下一个直线运动绝对坐标
                radius      插入圆弧的半径，当过大的时候自动缩小。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSmooth(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float end1, float end2, float end3, float next1, float next2, float next3, float radius);

/*************************************************************
Description:    //空间直线插补SP运动，根据下一个直线运动的绝对坐标在拐角自动插入圆弧，加入圆弧后会使得运动的终点与直线的终点不一致，拐角过大时不会插入圆弧，当距离不够时会自动减小半径
Input:          //卡链接handle
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                end1              第1个轴运动绝对坐标
                end2              第2个轴运动绝对坐标
                end3              第3个轴运动绝对坐标
                next1      第1个轴下一个直线运动绝对坐标
                next2      第2个轴下一个直线运动绝对坐标
                next3      第3个轴下一个直线运动绝对坐标
                radius      插入圆弧的半径，当过大的时候自动缩小。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSmoothSp(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, float end1, float end2, float end3, float next1, float next2, float next3, float radius);


/*************************************************************
Description:    //运动暂停		，插补运动暂停主轴。轴列表轴第一个轴
Input:          //卡链接handle
                轴号 iaxis
                模式 imode	0（缺省） 暂停当前运动。
                            1 在当前运动完成后正准备执行下一条运动指令时暂停。
                            2 在当前运动完成后正准备执行下一条运动指令时，并且两条指令的MARK标识不一样时暂停。这个模式可以用于一个动作由多个指令来实现时，可以在一整个动作完成后暂停。

Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MovePause(ZMC_HANDLE handle,int iaxis, int imode);


/*************************************************************
Description:    //取消运动暂停
Input:          //卡链接handle
                    轴号 iaxis
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveResume(ZMC_HANDLE handle,int iaxis);

/*************************************************************
Description:    //在当前的运动末尾位置增加速度限制，用于强制拐角减速
Input:          //卡链接handle
                 轴号 iaxis
Output:         //
 Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveLimit(ZMC_HANDLE handle, int iaxis,float limitspeed);

/*************************************************************
Description:    //在运动缓冲中加入输出指令
Input:          //卡链接handle
                轴号 iaxis
                输出口编号 ioutnum
                输出口状态	ivalue
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveOp(ZMC_HANDLE handle, int iaxis,int ioutnum, int ivalue);

/*************************************************************
Description:    //在运动缓冲中加入连续输出口输出指令
Input:          //卡链接handle
                轴号 iaxis
                输出口起始编号 ioutnumfirst
                输出口结束编号 ioutnumend
                对应输出口状态二进制组合值	ivalue
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveOpMulti(ZMC_HANDLE handle, int iaxis, int ioutnumfirst, int ioutnumend, int ivalue);

/*************************************************************
Description:    //在运动缓冲中加入输出指令 ,指定时间后输出状态翻转
Input:          //卡链接handle
                轴号 iaxis
                输出口编号 ioutnum
                输出口状态	ivalue
                状态反转时间 iofftimems
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveOp2(ZMC_HANDLE handle, int iaxis, int ioutnum, int ivalue, int iofftimems);

/*************************************************************
Description:    //在运动缓冲中加入AOUT输出指令
Input:          //卡链接handle
                轴号 iaxis
                DA口编号 ioutnum
                模拟量值 fvalue （4系列以下 12位0-4095）
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveAout(ZMC_HANDLE handle, int iaxis, int ioutnum, float fvalue);

/*************************************************************
Description:    //在运动缓冲中加入延时指令
Input:          //卡链接handle
                轴号 iaxis
                延时时间 itimems 毫秒
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveDelay(ZMC_HANDLE handle, int iaxis, int itimems);

/*************************************************************
Description:    //旋转台直线插补运动。  20130901 以后的控制器版本支持
Input:          //卡链接handle
                tablenum			存储旋转台参数的table编号
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveTurnabs(ZMC_HANDLE handle, int tablenum ,int imaxaxises,int *piAxislist, float *pfDisancelist);

/*************************************************************
Description:    //旋转台圆弧+螺旋插补运动。  20130901 以后的控制器版本支持
Input:          //卡链接handle
                tablenum       存储旋转参数的table编号
                refpos1    第一个轴参考点，绝对位置
                refpos2    第二个轴参考点，绝对位置
                mode      1-参考点是当前点前面，2-参考点是结束点后面，3-参考点在中间，采用三点定圆的方式。
                end1              第一个轴结束点，绝对位置
                end2              第二个轴结束点，绝对位置
                imaxaxises        参与运动轴数量
                piAxislist		  轴列表
                pfDisancelist	螺旋轴距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_McircTurnabs(ZMC_HANDLE handle, int tablenum ,float refpos1,float refpos2,int mode,float end1,float end2,int imaxaxises,int *piAxislist ,float *pfDisancelist);

/*************************************************************
Description:    //电子凸轮 同步运动
Input:          //卡链接handle
                iaxis			轴号
                istartpoint		起始点TABLE编号
                iendpoint		结束点TABLE编号
                ftablemulti		位置比例，一般设为脉冲当量值
                fDistance		参考运动的距离，用来计算总运动时间
 Output:         //
 Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Cam(ZMC_HANDLE handle,int iaxis, int istartpoint, int iendpoint, float ftablemulti, float fDistance);

/*************************************************************
Description:    //电子凸轮 同步运动
Input:          //卡链接handle
                iaxis			轴号
                istartpoint		起始点TABLE编号
                iendpoint		结束点TABLE编号
                ftablemulti		位置比例，一般设为脉冲当量值
                fDistance		参考运动的距离
                ilinkaxis		参考主轴
                ioption			参考轴的连接方式
                flinkstartpos	ioption条件中距离参数
 Output:         //
 Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Cambox(ZMC_HANDLE handle, int iaxis,int istartpoint, int iendpoint, float ftablemulti, float fDistance, int ilinkaxis, int ioption, float flinkstartpos);



/*************************************************************
Description:    //特殊凸轮 同步运动
Input:          //卡链接handle
                iaxis			参与运动的轴号(跟随轴)
                fDistance		同步过程跟随轴运动距离
                fLinkDis		同步过程参考轴(主轴)运动绝对距离
                fLinkAcc		跟随轴加速阶段，参考轴移动的绝对距离
                fLinkDec		跟随轴减速阶段，参考轴移动的绝对距离
                iLinkaxis		参考轴的轴号
                ioption			连接模式选项
                flinkstartpos	连接模式选项中运动距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Movelink(ZMC_HANDLE handle, int iaxis,float fDistance, float fLinkDis, float fLinkAcc, float fLinkDec,int iLinkaxis, int ioption, float flinkstartpos);

/*************************************************************
Description:    //特殊凸轮 同步运动
Input:          //卡链接handle
                iaxis			参与运动的轴号(跟随轴)
                fDistance		同步过程跟随轴运动距离
                fLinkDis		同步过程参考轴(主轴)运动绝对距离
                startsp			启动时跟随轴和参考轴的速度比例，units/units单位，负数表示跟随轴负向运动
                endsp			结束时跟随轴和参考轴的速度比例，units/units单位, 负数表示跟随轴负向运动。
                iLinkaxis		参考轴的轴号
                ioption			连接模式选项
                flinkstartpos	连接模式选项中运动距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Moveslink(ZMC_HANDLE handle, int iaxis,float fDistance, float fLinkDis, float startsp, float endsp,int iLinkaxis, int ioption, float flinkstartpos);

/*************************************************************
Description:    //连接 同步运动指令 电子齿轮
Input:          //卡链接handle
                ratio		比率，可正可负，注意是脉冲个数的比例。
                link_axis	连接轴的轴号，手轮时为编码器轴
                move_axis	随动轴号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Connect(ZMC_HANDLE handle, float ratio, int link_axis, int move_axis);

/*************************************************************
Description:    //连接 同步运动指令 电子齿轮 将当前轴的目标位置与link_axis轴的插补矢量长度通过电子齿轮连接
Input:          //卡链接handle
                ratio		比率，可正可负，注意是脉冲个数的比例。
                link_axis	连接轴的轴号，手轮时为编码器轴
                move_axis	随动轴号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Connpath(ZMC_HANDLE handle, float ratio, int link_axis, int move_axis);


/*************************************************************
Description:    //位置锁存指令
Input:          //卡链接handle
                iaxis	轴号
                imode	锁存模式
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Regist(ZMC_HANDLE handle,int iaxis, int imode);

/*************************************************************
Description:    //编码器输入齿轮比，缺省(1,1)
Input:          //卡链接handle
                iaxis		轴号
                mpos_count	分子，不要超过65535
                input_count	 分母，不要超过65535
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_EncoderRatio(ZMC_HANDLE handle, int iaxis,int mpos_count,int input_count);

/*************************************************************
Description:    //设置步进输出齿轮比，缺省(1,1)
Input:          //卡链接handle
                iaxis		轴号
                mpos_count	分子，1-65535
                input_count	 分母，1-65535
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_StepRatio(ZMC_HANDLE handle, int iaxis, int mpos_count,int input_count);


/*************************************************************
Description:    //所有轴立即停止
Input:          //卡链接handle
                imode 停止模式
                0（缺省）取消当前运动
                1	取消缓冲的运动
                2	取消当前运动和缓冲运动。
                3	立即中断脉冲发送。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Rapidstop(ZMC_HANDLE handle, int imode);

/*************************************************************
Description:    //多个轴运动停止
Input:          //卡链接handle  轴号， 距离
                imaxaxises		轴数
                piAxislist		轴列表
                imode	模式
                    0（缺省）取消当前运动
                    1	取消缓冲的运动
                    2	取消当前运动和缓冲运动。
                    3	立即中断脉冲发送。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_CancelAxisList(ZMC_HANDLE handle, int imaxaxises, int *piAxislist, int imode);

/*************************************************************
Description:    //CONNFRAME机械手逆解指令	2系列以上控制器支持
Input:          //卡链接handle
                Jogmaxaxises	关节轴数量
                JogAxislist		关节轴列表
                frame			机械手类型
                tablenum		机械手参数TABLE起始编号
                Virmaxaxises	关联虚拟轴个数
                VirAxislist		虚拟轴列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Connframe(ZMC_HANDLE handle,int Jogmaxaxises,int *JogAxislist, int frame, int tablenum , int Virmaxaxises , int *VirAxislist);

/*************************************************************
Description:    //CONNREFRAME机械手正解指令	2系列以上控制器支持
Input:          //卡链接handle
                Virmaxaxises	关联虚拟轴个数
                VirAxislist		虚拟轴列表
                frame			机械手类型
                tablenum		机械手参数TABLE起始编号
                Jogmaxaxises	关节轴数量
                JogAxislist		关节轴列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Connreframe(ZMC_HANDLE handle,int Virmaxaxises , int *VirAxislist, int frame, int tablenum , int Jogmaxaxises,int *JogAxislist);


#if 0
    //单轴函数
#endif

/*************************************************************
Description:    //轴叠加运动	iaddaxis运动叠加到iaxis轴 ，ADDAX指令叠加的是脉冲个数
Input:          //卡链接handle
                 iaxis		被叠加轴
                 iaddaxis	叠加轴
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_Addax(ZMC_HANDLE handle, int iaxis, int iaddaxis);

/*************************************************************
Description:    //单轴运动停止
Input:          //卡链接handle
                iaxis 轴号
                imode 模式
                    0（缺省）取消当前运动
                    1	取消缓冲的运动
                    2	取消当前运动和缓冲运动。
                    3	立即中断脉冲发送。
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_Cancel(ZMC_HANDLE handle, int iaxis, int imode);


/*************************************************************
Description:    //单轴连续运动
Input:          //卡链接handle
                iaxis 轴号
                idir 方向 1正向 -1负向
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_Vmove(ZMC_HANDLE handle, int iaxis, int idir);

/*************************************************************
Description:    //控制器方式回零
Input:          //卡链接handle
                iaxis	轴号
                imode	模式
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_Datum(ZMC_HANDLE handle, int iaxis, int imode);

/*************************************************************
Description:    //回零完成状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //homestatus 回零完成标志 0-回零异常 1回零成功
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetHomeStatus(ZMC_HANDLE handle,uint32 iaxis,uint32 * homestatus);


/*************************************************************
Description:    //单轴相对运动
Input:          //卡链接handle
                iaxis 轴号
                fdistance 距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_Move(ZMC_HANDLE handle, int iaxis, float fdistance);


/*************************************************************
Description:    //单轴绝对运动
Input:          //卡链接handle
                iaxis 轴号
                fdistance 距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Single_MoveAbs(ZMC_HANDLE handle, int iaxis, float fdistance);


#if 0
    //辅助函数
#endif

/*********************内存操作

/*********************内存操作
/*************************************************************
Description:    //写VR,
Input:          //卡链接handle
                vrstartnum		VR起始编号
                numes			写入的数量
                pfValue			写入的数据列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetVrf(ZMC_HANDLE handle,int vrstartnum, int numes, float *pfValue);

/*************************************************************
Description:    //VR读取, 可以一次读取多个
Input:          //卡链接handle
                vrstartnum	读取的VR起始地址
                numes		读取的数量
Output:         //pfValue  返回的读取值，多个时必须分配空间.
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVrf(ZMC_HANDLE handle, int vrstartnum, int numes, float *pfValue);


/*************************************************************
Description:    //VRINT读取， 必须150401以上版本才支持VRINT的DIRECTCOMMAND读取
Input:          //卡链接handle
                vrstartnum	读取的VR起始地址
                numes		读取的数量
                Output:         //piValue  返回的读取值，多个时必须分配空间.
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetVrInt(ZMC_HANDLE handle, int vrstartnum, int numes, int *piValue);

/*************************************************************
Description:    //写table
Input:          //卡链接handle
                tabstart	写入的TABLE起始编号
                numes		写入的数量
                pfValue		写入的数据值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetTable(ZMC_HANDLE handle,int tabstart, int numes, float *pfValue);

/*************************************************************
Description:    //table读取, 可以一次读取多个
Input:          //卡链接handle
                tabstart	读取TABLE起始地址
                numes		读取的数量
Output:         //pfValue  多个时必须分配空间.
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetTable(ZMC_HANDLE handle, int tabstart, int numes, float *pfValue);

/*************************************************************
Description:    //字符串转为float
Input:          //卡链接handle
                pstringin 数据的字符串
                inumes   转换数据个数
Output:         //pfvlaue 转换的数据
Return:         //错误码
*************************************************************/
int32  ZAux_TransStringtoFloat(const char* pstringin, int inumes,  float* pfvlaue);


/*************************************************************
Description:    //字符串转为int
Input:          //卡链接handle
                pstringin 数据的字符串
                inumes   转换数据个数
Output:         //pivlaue 转换的数据
Return:         //错误码
*************************************************************/
int32  ZAux_TransStringtoInt(const char* pstringin, int inumes,  int* pivlaue);

/*************************************************************
Description:    //把float格式的变量列表存储到文件， 与控制器的U盘文件格式一致.
Input:          //sFilename 文件绝对路径
                pVarlist	写入的数据列表
                inum		数据的长度
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_WriteUFile(const char *sFilename, float *pVarlist, int inum);

/*************************************************************
Description:    //读取float格式的变量列表， 与控制器的U盘文件格式一致.
Input:          //sFilename 文件绝对路径
                inum		数据的长度
Output:         //pVarlist	读取的数据列表
Return:         //错误码
*************************************************************/
int32  ZAux_ReadUFile(const char *sFilename, float *pVarlist, int* pinum);


/*************************************************************
Description:    //modbus寄存器操作 modbus_bit
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
                pdata 设置的位状态  按位存储
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Set0x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint8* pdata);

/*************************************************************
Description:    //modbus寄存器操作 modbus_bit
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
Output:         //pdata 返回的位状态  按位存储
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Get0x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint8* pdata);

/*************************************************************
Description:    //modbus寄存器操作 MODBUS_REG
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
Output:         //pdata	读取的REG寄存器值
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Set4x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint16* pdata);

/*************************************************************
Description:    //modbus寄存器操作 MODBUS_REG
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
Output:         //pdata	读取的REG寄存器值
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Get4x(ZMC_HANDLE handle, uint16 start, uint16 inum, uint16* pdata);

/*************************************************************
Description:    //modbus寄存器操作		MODBUS_IEEE
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
                pfdata	设置值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Get4x_Float(ZMC_HANDLE handle, uint16 start, uint16 inum, float* pfdata);

/*************************************************************
Description:    //modbus寄存器操作		MODBUS_IEEE
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
                pfdata	设置值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Set4x_Float(ZMC_HANDLE handle, uint16 start, uint16 inum, float* pfdata);

/*************************************************************
Description:    //modbus寄存器操作		MODBUS_LONG
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
                pidata	设置值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Get4x_Long(ZMC_HANDLE handle, uint16 start, uint16 inum, int32* pidata);

/*************************************************************
Description:    //modbus寄存器操作 MODBUS_LONG
Input:          //卡链接handle 寄存器地址
                start	起始编号
                inum	数量
Output:         //pidata	读取的REG寄存器值
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Set4x_Long(ZMC_HANDLE handle, uint16 start, uint16 inum, int32* pidata);

/*************************************************************
Description:    //读取modbus_string
Input:          //卡链接handle
                start	modbus起始地址
                inum	长度
Output:         pidata	读取返回的字符串
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Get4x_String(ZMC_HANDLE handle, uint16 start, uint16 inum, char * pidata);

/*************************************************************
Description:    //设置modbus_string
Input:          //卡链接handle
                start	modbus起始地址
                inum	长度
                pidata	写入的字符串
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Modbus_Set4x_String(ZMC_HANDLE handle, uint16 start, uint16 inum, char * pidata);

/*************************************************************
Description:    //写用户flash块, float数据
Input:          //卡链接handle
                    uiflashid 	flash块号
                    uinumes		变量个数
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_FlashWritef(ZMC_HANDLE handle, uint16 uiflashid, uint32 uinumes, float *pfvlue);

/*************************************************************
Description:    //读取用户flash块, float数据
Input:          //卡链接handle
                    uiflashid 	flash块号
                    uibuffnum	缓冲变量个数
Output:         //
                    puinumesread 读取到的变量个数
Return:         //错误码
*************************************************************/
int32  ZAux_FlashReadf(ZMC_HANDLE handle, uint16 uiflashid, uint32 uibuffnum, float *pfvlue, uint32* puinumesread);



/*****************************************************************************************************2018-08-24 V2.1函数添加**************************
/*************************************************************
Description:    //示波器触发函数 150723以后固件版本支持
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Trigger(ZMC_HANDLE handle);


//MOVE_PARA,MOVE_PWM, MOVE_SYNMOVE,MOVE_ASYNMOVE
/*************************************************************
Description:    //运动中修改参数. 20170503以上固件支持
Input:          //卡链接handle  PARANAME, INDEX, VALUE
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MovePara(ZMC_HANDLE handle,uint32 base_axis,char *  paraname,uint32 iaxis,float fvalue);

/*************************************************************
Description:    //运动中修改PWM 20170503以上固件支持
Input:          //卡链接handle  PARANAME, INDEX, VALUE
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MovePwm(ZMC_HANDLE handle,uint32 base_axis,uint32 pwm_num,float pwm_duty,float pwm_freq);

/*************************************************************
Description:    //运动中同步其他轴的运动,. 20170503以上固件支持
Input:          //卡链接handle  PARANAME, INDEX, VALUE
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveSynmove(ZMC_HANDLE handle,uint32 base_axis,uint32 iaxis,float fdist,uint32 ifsp);

/*************************************************************
Description:    //运动中触发其他轴的运动. 20170503以上固件支持
Input:          //卡链接handle  PARANAME, INDEX, VALUE
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveASynmove(ZMC_HANDLE handle,uint32 base_axis,uint32 iaxis,float fdist,uint32 ifsp);

/*************************************************************
Description:    //运动中修改TABLE
Input:          //卡链接handle
                base_axis	插补主轴编号
                table_num	TABLE编号
                fvalue		修改值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveTable(ZMC_HANDLE handle,uint32 base_axis,uint32 table_num,float fvalue);


/*************************************************************
Description:    //BASE轴运动缓冲加入一个可变的延时  固件150802以上版本， 或XPLC160405以上版本支持。
Input:          //卡链接handle
                base_axis	插补主轴编号
                paraname	参数名字符串 DPOS MPOS IN AIN VPSPEED MSPEED MODBUS_REG MODBUS_IEEE MODBUS_BIT NVRAM VECT_BUFFED  REMAIN
                inum		参数编号或轴号
                Cmp_mode	比较条件 1 >=   0=  -1<=  对IN等BIT类型参数无效。
                fvalue		比较值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveWait(ZMC_HANDLE handle,uint32 base_axis,char * paraname,int inum,int Cmp_mode,float fvalue);

/*************************************************************
Description:    //BASE轴运动缓冲加入一个TASK任务 当任务已经启动时，会报错，但不影响程序执行。
Input:          //卡链接handle
                base_axis	插补主轴编号
                tasknum   	任务编号
                labelname		BAS中全局函数名或者标号
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MoveTask(ZMC_HANDLE handle,uint32 base_axis,uint32 tasknum,char * labelname);

/*************************************************************
Description:    //位置比较PSWITCH
Input:          //卡链接handle
                //比较器编号 num 0-15
                //比较器使能 enable 0/1
                //比较的轴号 axisnum
                //输出口编号 outnum
                //输出状态	outstate 0/1
                //比较起始位置	setpos
                //输出复位位置	resetpos
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_Pswitch(ZMC_HANDLE handle, int num, int enable, int axisnum, int outnum, int  outstate, float setpos,float resetpos);

/*************************************************************
Description:    //硬件位置比较输出 4系列产品脉冲轴与编码器轴支持硬件比较输出
Input:          //卡链接handle
                //模式	 mode 1-启动比较器, 2- 停止并删除没完成的比较点.
                //方向 direction 0-坐标负向,  1- 坐标正向
                //预留 Reserve   预留
                //TABLE起始点 Tablestart  第一个比较点坐标所在TABLE编号
                //TABLE结束点 tableend 最后一个比较点坐标所在TABLE编号
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_HwPswitch(ZMC_HANDLE handle,int Axisnum,int Mode, int Direction, int Reserve, int Tablestart, int  Tableend);

/*************************************************************
Description:    //硬件位置比较输出剩余缓冲获取 4系列产品脉冲轴与编码器轴支持硬件比较输出
Input:          //卡链接handle
                //模式	 axisnum				轴号
output:			//位置比较输出剩余缓冲数		buff
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetHwPswitchBuff(ZMC_HANDLE handle,int axisnum,int * buff);

/*************************************************************
Description:    //硬件定时器用于硬件比较输出后一段时间后还原电平 4系列产品支持
Input:          //卡链接	handle
                //模式			mode		0停止,  2-启动
                //周期时间	   cyclonetime  us单位
                //有效时间	   optime		us单位
                //重复次数	   reptimes
                //输出缺省状态 opstate		输出口变为非此状态后开始计时
                // 输出口编号  opnum		必须能硬件比较输出的口
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_HwTimer(ZMC_HANDLE handle,int mode,int cyclonetime,int optime,int reptimes,int opstate,int  opnum );

/*************************************************************
Description:    //读取轴停止原因
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAxisStopReason(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //浮点型读全部轴参数状态
Input:          //卡链接handle
Input:          //
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAllAxisPara(ZMC_HANDLE handle,const char *sParam,int imaxaxis,float * pfValue);

/*************************************************************
Description:    //浮点型读全部轴参数状态
Input:          //卡链接handle
Input:          //
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetAllAxisInfo(ZMC_HANDLE handle,int imaxaxis,int * IdleStatus,float * DposStatus,float * MposStatus,int * AxisStatus);

/*************************************************************
Description:    //设置BASIC自定义全局数组
Input:          //卡链接handle
                arrayname 数组名称
                arraystart 数组起始元素
                numes		元素数量
                pfValue     设置值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetUserArray(ZMC_HANDLE handle,char * arrayname ,int arraystart, int numes, float *pfValue);

/*************************************************************
Description:    //读取设置BASIC自定义全局数组 , 可以一次读取多个
Input:          //卡链接handle
                arrayname 数组名称
                arraystart 数组起始元素
                numes		元素数量
Output:         //pfValue  多个时必须分配空间.
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetUserArray(ZMC_HANDLE handle,char * arrayname ,int arraystart, int numes, float *pfValue);

/*************************************************************
Description:    //设置自定义变量,
Input:          //卡链接handle
                varname 变量名称字符串
                pfValue	设定值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetUserVar(ZMC_HANDLE handle,char * varname , float pfValue);

/*************************************************************
Description:    //读取自定义全局变量,
Input:          //卡链接handle
                varname 变量名称字符串
Output:         //pfValue  变量值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetUserVar(ZMC_HANDLE handle,char * varname , float *pfValue);

/*************************************************************
Description:    //读取PCI的控制卡个数
Input:          //
Output:         //卡链接handle
Return:         //错误码
*************************************************************/
int32  ZAux_GetMaxPciCards();

/*************************************************************
Description:    //与控制器建立链接
Input:          //PCI卡号，
Output:         //卡链接handle
Return:         //错误码
*************************************************************/
int32  ZAux_OpenPci(uint32 cardnum, ZMC_HANDLE * phandle);

/*************************************************************
Description:    //获取控制器卡信息
Input:          //卡链接handle
Output:         SoftType 控制器型号类型
                SoftVersion 控制器软件版本（固件版本）
                ControllerId	控制器唯一ID
Return:         //错误码
uint32  ZMC_GetSoftVersion(ZMC_HANDLE handle,float *pVersion, char *pVerString, uint32 *pVerDate);
*************************************************************/
int32  ZAux_GetControllerInfo(ZMC_HANDLE handle ,char * SoftType ,char * SoftVersion ,char *  ControllerId);

/*************************************************************
Description:    //读取卡槽上节点数量
Input:          //卡链接handle
:				 //slot 槽位号缺省0
Output:         //piValue 返回扫描成功节点数量
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetNodeNum(ZMC_HANDLE handle,int slot,int *piValue);

/*************************************************************
Description:    //读取节点上的信息
Input:          //卡链接handle
                slot	槽位号
                node	节点编号
                sel		信息编号	0-厂商编号1-设备编号 2-设备版本 3-别名 10-IN个数 11-OUT个数
Output:         //piValue 返回信息
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetNodeInfo(ZMC_HANDLE handle,int slot,int node,int sel ,int *piValue);

/*************************************************************
Description:    //读取节点总线状态
Input:          //卡链接handle
                slot 槽位号缺省0
                node 节点编号
Output:         //nodestatus 按位处理 bit0-节点是否存在  bit1-通讯状态   bit2-节点状态
值为1时，bit0为1，bit1和bit2为0，设备通讯正常
值为3时，bit0和bit1为1，bit2为0，设备通讯出错
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetNodeStatus(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 * nodestatus);

/*************************************************************
Description:    //读取节点SDO参数信息
Input:          //卡链接handle
                slot	槽位号 缺省0
                node	节点编号
                index	对象字典编号（注意函数为10进制数据）
                subindex	子编号	（注意函数为10进制数据）
                type	数据类型  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
Output:         //value 读取的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SDORead(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 index,uint32 subindex ,uint32 type ,int32 * value);

/*************************************************************
Description:    //写节点SDO参数信息
Input:          //卡链接handle
                slot	槽位号 缺省0
                node	节点编号
                index	对象字典编号（注意函数为10进制数据）
                subindex	子编号	（注意函数为10进制数据）
                type	数据类型  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
                value	设定的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SDOWrite(ZMC_HANDLE handle,uint32 slot,uint32 node,uint32 index,uint32 subindex ,uint32 type ,int32  value);

/*************************************************************
Description:    //读取轴SDO参数信息
Input:          //卡链接handle
                slot	槽位号 缺省0
                node	节点编号
                index	对象字典编号（注意函数为10进制数据）
                subindex	子编号	（注意函数为10进制数据）
                type	数据类型  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
Output:         //value 读取的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SDOReadAxis(ZMC_HANDLE handle,uint32 iaxis,uint32 index,uint32 subindex ,uint32 type ,int32 * value);

/*************************************************************
Description:    //写轴SDO参数信息
Input:          //卡链接handle
                slot	槽位号 缺省0
                node	节点编号
                index	对象字典编号（注意函数为10进制数据）
                subindex	子编号	（注意函数为10进制数据）
                type	数据类型  1-bool 2-int8 3-int16 4-int32 5-uint8 6-uint16 7-uint32
                value	设定的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SDOWriteAxis(ZMC_HANDLE handle,uint32 iaxis,uint32 index,uint32 subindex ,uint32 type ,int32  value);

/*************************************************************
Description:    //Rtex读取参数信息
Input:          //卡链接handle
                iaxis	轴号
                ipara	参数分类*256 + 参数编号  Pr7.11-ipara = 7*256+11
Output:         //value 读取的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_RtexRead(ZMC_HANDLE handle,uint32 iaxis ,uint32 ipara ,float * value);

/*************************************************************
Description:    //Rtex写参数信息
Input:          //卡链接handle
                iaxis	轴号
                ipara	参数分类*256 + 参数编号  Pr7.11-ipara = 7*256+11
                value	设定的数据值
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_RtexWrite(ZMC_HANDLE handle,uint32 iaxis ,uint32 ipara,float  value);


/*************************************************************
Description:    //设置回零偏移距离
Input:          //卡链接handle
                iaxis 轴号
                fValue 偏移距离
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SetDatumOffpos(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取回零偏移距离
Input:          //卡链接handle
                iaxis 轴号
Output:         //fValue 反馈的偏移距离
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetDatumOffpos(ZMC_HANDLE handle, int iaxis, float * fValue);

/*************************************************************
Description:    //总线驱动器回零
Input:          //卡链接handle
                homemode 回零模式，查看驱动器手册
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_Datum(ZMC_HANDLE handle,uint32 iaxis,uint32  homemode);

/*************************************************************
Description:    //驱动器回零完成状态
Input:          //卡链接handle
                iaxis 轴号
Output:         //homestatus 回零完成标志 0-回零异常 1回零成功
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetHomeStatus(ZMC_HANDLE handle,uint32 iaxis,uint32 * homestatus);

/*************************************************************
Description:    //设置清除总线伺服报警
Input:          //卡链接handle
                iaxis 轴号
                mode 模式 0-清除当前告警  1-清除历史告警  2-清除外部输入告警
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_DriveClear(ZMC_HANDLE handle,uint32 iaxis,uint32  mode);

/*************************************************************
Description:    //读取当前总线驱动当前力矩	需要设置对应的DRIVE_PROFILE类型
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 当前转矩
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetDriveTorque(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置当前总线驱动最大转矩  需要设置对应的DRIVE_PROFILE类型
Input:          //卡链接handle
                iaxis 轴号
                piValue 最大转矩限制
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_SetMaxDriveTorque(ZMC_HANDLE handle, int iaxis, int piValue);

/*************************************************************
Description:    //读取当前总线驱动最大转矩  需要设置对应的DRIVE_PROFILE类型
Input:          //卡链接handle
                iaxis 轴号
Output:         //piValue 返回的最大转矩
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_GetMaxDriveTorque(ZMC_HANDLE handle, int iaxis, int *piValue);

/*************************************************************
Description:    //设置模拟量输出 力矩、速度模式下可以  总线驱动需要设置对应DRIVE_PROFILE类型 与ATYPE
Input:          //卡链接handle
                iaxis 轴号
                模拟量 输出值
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetDAC(ZMC_HANDLE handle, int iaxis, float fValue);

/*************************************************************
Description:    //读取模拟量输出 力矩、速度模式下可以  总线驱动需要设置对应DRIVE_PROFILE类型 与ATYPE
Input:          //卡链接handle
                iaxis 轴号
Output:         //fValue 模拟量返回值
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetDAC(ZMC_HANDLE handle, int iaxis, float *fValue);

/*************************************************************
Description:    //总线初始化  （针对Zmotion tools 工具软件配置过总线参数控制器使用有效）
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_BusCmd_InitBus(ZMC_HANDLE handle);

/*************************************************************
Description:    //获取总线初始化完成状态  （针对Zmotion tools 工具软件配置过总线参数控制器使用有效）
Input:          //卡链接handle
Output:         //
Return:         //错误码piValue 0-初始化失败 1成功
*************************************************************/
int32  ZAux_BusCmd_GetInitStatus(ZMC_HANDLE handle,int *piValue);

/*************************************************************
Description:    //读取多个输入信号
Input:          //卡链接handle
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetInMulti(ZMC_HANDLE handle, int startio , int endio, int32 *piValue);

/*************************************************************
Description:    //命令的延时等待时间
Input:          //卡链接handle 毫秒时间
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_SetTimeOut(ZMC_HANDLE  handle, uint32 timems);

/*************************************************************2017-12-14添加
Description:    //硬件位置比较输出2 4系列产品, 20170513以上版本支持.  ZMC306E/306N支持
Input:          //卡链接					handle
                //模式						mode
                //输出口编号				Opnum		4系列 out 0-3为硬件位置比较输出
                //第一个比较点的输出状态	Opstate		0-关闭 1打开
                //多功能参数				ModePara1
                //多功能参数				ModePara2
                //多功能参数				ModePara3
                //多功能参数				ModePara4

mode 1-启动比较器,
        ModePara1 =  第一个比较点坐标所在TABLE编号
        ModePara2 =	 最后一个比较点坐标所在TABLE编号
        ModePara3 =  第一个点判断方向,  0-坐标负向,  1- 坐标正向,  -1-不使用方向
        ModePara4 =	 预留

mode 2- 停止并删除没完成的比较点.
        ModePara1 =  预留
        ModePara2 =	 预留
        ModePara3 =  预留
        ModePara4 =	 预留

mode 3- 矢量比较方式
        ModePara1 =  第一个比较点坐标所在TABLE编号
        ModePara2 =	 最后一个比较点坐标所在TABLE编号
        ModePara3 =  预留
        ModePara4 =	 预留

Mode=4 :矢量比较方式, 单个比较点
        ModePara1 =  比较点坐标
        ModePara2 =	 预留
        ModePara3 =  预留
        ModePara4 =	 预留

Mode=5 :矢量比较方式, 周期脉冲模式
        ModePara1 =  比较点坐标
        ModePara2 =	 重复周期, 一个周期内比较两次, 先输出有效状态,再输出无效状态.
        ModePara3 =  周期距离, 每隔这个距离输出Opstate, 输出有效状态的距离（ModePara4）后还原为无效状态.
        ModePara4 =	 输出有效状态的距离,  (ModePara3- ModePara4) 为无效状态距离

Mode=6 :矢量比较方式, 周期模式, 这种模式一般与HW_TIMER一起使用.
        ModePara1 =  比较点坐标
        ModePara2 =	 重复周期, 一个周期只比较一次
        ModePara3 =  周期距离, 每隔这个距离输出一次
        ModePara4 =	 预留
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_HwPswitch2(ZMC_HANDLE handle,int Axisnum,int Mode, int Opnum , int Opstate, float ModePara1, float ModePara2,float ModePara3,float ModePara4);

/*************************************************************
Description:    //获取控制器最大规格数
Input:          //卡链接handle
Output:         //Max_VirtuAxises	 最大虚拟轴数
Output:         //Max_motor			 最大电机数量
Output:         //Max_io			 最大IN,OUT,AD,DA数量
Return:         //错误码
*************************************************************/
int32  ZAux_GetSysSpecification(ZMC_HANDLE  handle, uint16 * Max_VirtuAxises,uint8 * Max_motor,uint8 *Max_io);

/*************************************************************
Description:    ////主动上报的回调函数格式
Input:          //卡链接handle
                itypecode: 上传类型码
                idatalength: 数据长度
                pdata: 数据指针
Output:         //
Return:         //错误码
*************************************************************/
typedef void (*PZMCAutoUpCallBack)(ZMC_HANDLE handle, int32 itypecode, int32 idatalength, uint8 *pdata);

/*************************************************************
Description:    //控制器自动上报
Input:          //卡链接	handle
Input:         //回调函数   pcallback
Return:         //错误码
*************************************************************/
int32  ZAux_SetAutoUpCallBack(ZMC_HANDLE handle, PZMCAutoUpCallBack pcallback);

/*************************************************************
Description:    //IO接口 设置输出
Input:          //卡链接handle
Input:          //IO口起始编号  iofirst
Input:          //IO口结束编号  ioend
Input:          //输出口状态    istate按位存储，一个UINT存储32个输出口状态
Output:         //状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_SetOutMulti(ZMC_HANDLE handle, uint16 iofirst,uint16 ioend, uint32 * istate);

/*************************************************************
Description:    //IO接口 设置多路输出
Input:          //卡链接handle
Input:          //IO口起始编号  iofirst
Input:          //IO口结束编号  ioend
Input:          //输出口状态    istate按位存储，一个UINT存储32个输出口状态
Output:         //状态
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_GetOutMulti(ZMC_HANDLE handle, uint16 iofirst,uint16 ioend, uint32 * istate);

/*************************************************************
Description:    //多条相对多轴直线插补
Input:          //卡链接handle
                iMoveLen			填写的运动长度
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MultiMove(ZMC_HANDLE handle,int iMoveLen, int imaxaxises, int *piAxislist, float *pfDisancelist);

/*************************************************************
Description:    //多条相对多轴直线插补
Input:          //卡链接handle
                iMoveLen			填写的运动长度
                imaxaxises			参与运动总轴数
                piAxislist			轴号列表
                pfDisancelist		距离列表
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_MultiMoveAbs(ZMC_HANDLE handle,int iMoveLen, int imaxaxises, int *piAxislist, float *pfDisancelist);

/*************************************************************
Description:    //机械手坐标系旋转
Input:          //卡链接handle
                iaxis					轴号 关节轴/虚拟轴
                pfRotatePara			平移旋转参数
Output:         //
Return:         //错误码
*************************************************************/
int32  ZAux_Direct_FrameRotate(ZMC_HANDLE handle,int iaxis, float *pfRotatePara);

/*************************************************************
Description:    //获取CAN扩展资源规格
Input:          //卡链接handle
Output:         //CanNum			 当前连接的CAN从站数量
Output:         //CanId_List		 当前连接的CAN从站ID列表
Output:         //CanIn_List		 节点输入点数量
Output:         //CanOut_List		 节点输出点数量
Output:         //CanAin_List		 节点AD数量
Output:         //CanAOut_List		 节点DA数量
Output:         //CanAxis_List		 节点轴数量
Return:         //错误码
*************************************************************/
int32  ZAux_GetCanInfo(ZMC_HANDLE  handle, uint8 * CanNum,uint16 * CanId_List,uint8 *CanIn_List,uint8 *CanOut_List,uint8 *CanAin_List,uint8 *CanAOut_List,uint8 *CanAxis_List);

#ifdef __cplusplus
}
#endif


#endif











