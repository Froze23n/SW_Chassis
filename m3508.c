#include "m3508.h"
#include "pid.h"
#include "can.h"
#include "rc_translation.h"

/*
0x200	chassis	1234	0x201-204			---can2------speeding
0x1FF	chassis	5678	0x205-0x208		
*/

int16_t m3508_velocity_expection[4];
int16_t M3508_Velocity[4];
int16_t M3508_Current[4];

void M3508_Data_Handler(uint8_t ID,uint8_t *m3508_data)
{
	ID-=0x201;
	M3508_Velocity[ID] = (int16_t)( (m3508_data[2]<<8)|m3508_data[3] );
}

void M3508_Tx_Resolver(void)
{
	for(uint8_t ID=0; ID<4; ID++)
	{
		//4个m3508的PID 速度->电流
		M3508_Current[ID] = M3508_Velocity_PID(ID, m3508_velocity_expection[ID]-M3508_Velocity[ID] );
	}
}

//外接函数
void Start_M3508(void)
{
	Start_CAN2();//开启can2，开启中断接收
	M3508_PID_Para_Set();//设置PID参数
}
void M3508_CMD(void)
{
	rc_update_m3508_velocity(m3508_velocity_expection);
	M3508_Tx_Resolver();//进行PID计算
	CAN2_Tx(M3508_Current);//发送数据
}


