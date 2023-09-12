#include "pid.h"


float p_error_window;
float	i_error_window;
float	d_error_window;



struct PID_Para_t
{
	float p ;
	float i ;
	float d ;
};

struct PID_Para_t gv;//gm6020速度环
struct PID_Para_t ga;//gm6020角度环
struct PID_Para_t mv;//m3508速度环

void GM6020_PID_Para_Set(void)
{
	gv.p = 40.0f;
	gv.i = 40.0f;
	gv.d = 8.0f;
	
	ga.p = 5.0f;
	ga.i = 1.0f;
	ga.d = 4.0f;
}
void M3508_PID_Para_Set(void)
{
	mv.p = 5;
	mv.i = 20;
	mv.d = 0;
}

int16_t GM6020_Velocity_PID(uint8_t ID,int16_t error)
{
	static float p_error[4] = {0,0,0,0};
	static float i_error[4] = {0,0,0,0};
	static float d_error[4] = {0,0,0,0};
	static float old_error[4] = {0,0,0,0};
	
	p_error[ID] = (float)error;
	i_error[ID] += old_error[ID];
	d_error[ID] = old_error[ID]-p_error[ID];//attention!
	
	old_error[ID] = p_error[ID];
	
	if(i_error[ID]>=3000){i_error[ID]=3000;}
	if(i_error[ID]<=-3000){i_error[ID]=-3000;}
	float temp_voltage = (gv.p*p_error[ID]+gv.i*i_error[ID]+gv.d*d_error[ID]);
	if(temp_voltage>=30000){temp_voltage = 30000;}
	if(temp_voltage<=-30000){temp_voltage = -30000;}
	
	return (int16_t)temp_voltage;
}

int16_t GM6020_Angle_PID(uint8_t ID, float error)
{
	static float p_error[4] = {0,0,0,0};
	static float i_error[4] = {0,0,0,0};
	static float d_error[4] = {0,0,0,0};
	static float old_error[4] = {0,0,0,0};
	
	p_error[ID] = error;
	i_error[ID] += old_error[ID]/100.0f;
	d_error[ID] = p_error[ID]-old_error[ID];//attention!
	old_error[ID] = p_error[ID];
	
	if(i_error[ID]>=1){i_error[ID]=1;}
	if(i_error[ID]<=-1){i_error[ID]=-1;}
	int16_t temp_velocity = (int16_t)(ga.p*p_error[ID]+ga.i*i_error[ID]+ga.d*d_error[ID]);
	if(temp_velocity>=360){temp_velocity = 360;}
	if(temp_velocity<=-360){temp_velocity = -360;}
	
	p_error_window = p_error[0];
	i_error_window = i_error[0];
	d_error_window = d_error[0];
	
	return temp_velocity;
}

int16_t M3508_Velocity_PID(uint8_t ID,int16_t error)
{
	static float p_error[4] = {0,0,0,0};
	static float i_error[4] = {0,0,0,0};
	static float d_error[4] = {0,0,0,0};
	static float old_error[4] = {0,0,0,0};
	
	p_error[ID] = (float)error;
	i_error[ID] += old_error[ID]/100.0f;
	d_error[ID] = (old_error[ID]-p_error[ID])/10.0f;//attention!
	
	old_error[ID] = p_error[ID];
	
	if(i_error[ID]>=500){i_error[ID]=500;}
	if(i_error[ID]<=-500){i_error[ID]=-500;}
	float temp_current = (mv.p*p_error[ID]+mv.i*i_error[ID]+mv.d*d_error[ID]);
	if(temp_current>=15000){temp_current = 15000;}
	if(temp_current<=-15000){temp_current = -15000;}
	
	p_error_window = p_error[3];
	i_error_window = i_error[3];
	d_error_window = d_error[3];
	
	
	return (int16_t)temp_current;
}
