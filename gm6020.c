#include "gm6020.h"
#include "pid.h"
#include "can.h"
#include "rc_translation.h"

float GM6020_Angle_Offset[4] = {-75.0f,-60.0f,120.0f,105.0f};//指定零位
/*
0x1FF	Gimbal	1234	0x205-0x208		---can1------steering
0x2FF	Gimbal	567		0x209-0x20A		---can1other
*/
//控制编号为1-4的GM6020电机
float gm6020_angle_expection[4] = {45.0f,-45.0f,45.0f,-45.0f};//期望角度//绝对值//通过遥控器+底盘解算得知
int32_t circle[4] = {0,0,0,0};//转动圈数
float GM6020_Angle[4] = {0,0,0,0};//角度，范围为float
int16_t GM6020_Velocity[4] = {0,0,0,0};//编码器速度
int16_t GM6020_Voltage[4] = {0,0,0,0};//给定的电压

//gm6020回发数据处理//放置在RX0接收中断回调函数中
void GM6020_Data_Handler(uint8_t ID,uint8_t *gm6020_data)
{
	static int8_t old_area[4]={0,0,0,0};
	static int8_t new_area[4]={0,0,0,0};
	ID -= 0x205;//将电机ID转化为0123
	GM6020_Velocity[ID]= (int16_t)( (gm6020_data[2]<<8)|gm6020_data[3] );
	//过零处理//模型：三块区域划分[0   (-1)   2730    (0)   5461  (1)  8191]
	old_area[ID] = new_area[ID];
	uint16_t current_angle = ( (gm6020_data[0]<<8)|gm6020_data[1] );
	//检测相对空间区域
	if(current_angle<2730)
	{
		new_area[ID]=-1;
	}
	else if(current_angle>5461)
	{
		new_area[ID]=1;
	}
	else
	{
		new_area[ID]=0;
	}
	//检测是否过零//如果过零则对转动圈数作出调整
	if((new_area[ID]-old_area[ID])==-2)
	{
		circle[ID]+=1;
	}
	else if((new_area[ID]-old_area[ID])==2)
	{
		circle[ID]-=1;
	}
	//结合转动圈数计算出绝对位置//理论上转个一天一夜也远远不会溢出
	GM6020_Angle[ID] = (float)current_angle*360.0f/8192.0f+(float)circle[ID]*360.0f+GM6020_Angle_Offset[ID];
}

//gm6020发送数据处理
//需要用到pid.c中的pid控制函数
void GM6020_Tx_Resolver(void)
{
	static int16_t ex_velocity[4];
	for(uint8_t ID=0; ID<4; ID++)
	{
		//4个gm6020的串级PID 角度->速度->电压
		//墓前用的是同一套pid参数
		ex_velocity[ID] = GM6020_Angle_PID(ID, gm6020_angle_expection[ID] - GM6020_Angle[ID] );//角度->速度
		GM6020_Voltage[ID] = GM6020_Velocity_PID(ID, ex_velocity[ID]-GM6020_Velocity[ID] );//速度->电压
	}
}

//外接函数
void Start_GM6020(void)
{
	Start_CAN1();//开启can1，开启中断接收
	GM6020_PID_Para_Set();//设置PID参数
}
void GM6020_CMD(void)
{
	rc_update_gm6020_angle(circle,GM6020_Angle,gm6020_angle_expection);//Chassis//改变四个gm6020角度数值
	GM6020_Tx_Resolver();//进行PID计算
	CAN1_Tx(GM6020_Voltage);//发送数据给电机
}


