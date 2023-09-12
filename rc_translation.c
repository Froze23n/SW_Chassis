#include "rc_translation.h"
#include "usart.h"
#include "math.h"
#define	pi	3.14159274f

uint8_t safe_count = 0;
uint8_t rc_data_buffer[18] = {0};
extern DMA_HandleTypeDef hdma_usart3_rx;
struct rc_info
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
  /* mouse movement and button information */
  struct
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  union {
    uint16_t key_code;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
};

struct rc_info rc;

/*
舵轮运动解算，云台固定模式
x左右
y前后
z旋转
1-4为电机ID

							^(y)
							|							
		2					|					1		
							|							
							|							
							|							
							|							
(x)<---------(z)------------
							|							
							|							
							|							
		3					|					4		
							|							
							|						
							|					

*/
float RC_Velocity[4]={0,0,0,0};
float RC_Angle[4]={45.0f,-45.0f,45.0f,-45.0f};
int8_t m3508_direction[4] = {1,1,1,1};

float error[4];
float rc_expection[4];

void RC_Chassis_Resolver(void)
{
	static float vx[4];
	static float vy[4];
	
	float x = (float)rc.ch3;
	float y = (float)rc.ch4;
	float z = (float)rc.ch1;
//	if(y!=0){x=0;z=0;}
//	else if(x!=0){z=0;}
	
	x = -x*4;
	y = y*4;
	z = -z*4;
	
	if(x==0 && y==0 && z==0)
	{
		RC_Velocity[0]=0; RC_Velocity[1]=0; RC_Velocity[2]=0; RC_Velocity[3]=0;
		RC_Angle[0]=45;	RC_Angle[1]=-45;		RC_Angle[2]=45;	RC_Angle[3]=-45;
	}
	else
	{
		vx[0]=x+z/sqrt(2); vy[0]=y+z/sqrt(2);//ID=1
		vx[1]=x+z/sqrt(2); vy[1]=y-z/sqrt(2);//ID=2
		vx[2]=x-z/sqrt(2); vy[2]=y-z/sqrt(2);//ID=3
		vx[3]=x-z/sqrt(2); vy[3]=y+z/sqrt(2);//ID=4
		for(uint8_t ID=0; ID<4; ID++)
		{
			RC_Velocity[ID] = sqrt(    pow(vx[ID],2) + pow(vy[ID],2)    )  ;
			RC_Angle[ID] = atan2f( vy[ID],vx[ID] );//(-pi,pi]
			RC_Angle[ID] = RC_Angle[ID]*180.0f/pi;//(-180,+180]
		}
	}
}

//遥控器数据处理//这个函数会放在gm6020.c里面，要输入期望角度指针
void rc_update_gm6020_angle(int32_t * Circle, float * Current_Angle,float * Expection)
{
	for(uint8_t ID=0; ID<4; ID++)//四个电机的角度解算
	{
		rc_expection[ID] = RC_Angle[ID]+Circle[ID]*360.0f;
		error[ID] = rc_expection[ID]-Current_Angle[ID];
		//警惕Circle[ID]（圈数）是个可能会跳变的参考值，用来减少运算量//真角度还得看Current_Angle[ID]
		//以防万一，要分别进行双向角度逼近//
		m3508_direction[ID]=1;
		while(error[ID]>180.0f)
		{
				rc_expection[ID]-=360.0f;
				error[ID]-=360.0f;
		}
		while(error[ID]<-180.0f)
		{
				rc_expection[ID]+=360.0f;
				error[ID]+=360.0f;
		}
		if(error[ID]>90.0f)
		{
			rc_expection[ID]-=180.0f;
			m3508_direction[ID]=-1;
		}
		else if(error[ID]<-90.0f)
		{
			rc_expection[ID]+=180.0f;
			m3508_direction[ID]=-1;
		}
		Expection[ID] = rc_expection[ID];
	}
}

void rc_update_m3508_velocity(int16_t * Expection)
{
	static int8_t original_direction[4] = {-1,-1,1,1};
	for(uint8_t ID=0; ID<4; ID++)
	{
		Expection[ID] = RC_Velocity[ID]*original_direction[ID]*m3508_direction[ID];
	}
}



void Begin_Receive_DJIRC_Data(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rc_data_buffer,18);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart3)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,rc_data_buffer,18);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
		safe_count++;
		HAL_GPIO_TogglePin(Blue_GPIO_Port,Blue_Pin);
	}
}

void Process_RC_Buffer_Data(void)
{
	if(safe_count>0)
	{
		safe_count=0;
		rc.ch1 = (rc_data_buffer[0] | rc_data_buffer[1] << 8) & 0x07FF;
		rc.ch1 -= 1024;
		rc.ch2 = (rc_data_buffer[1] >> 3 | rc_data_buffer[2] << 5) & 0x07FF;
		rc.ch2 -= 1024;
		rc.ch3 = (rc_data_buffer[2] >> 6 | rc_data_buffer[3] << 2 | rc_data_buffer[4] << 10) & 0x07FF;
		rc.ch3 -= 1024;
		rc.ch4 = (rc_data_buffer[4] >> 1 | rc_data_buffer[5] << 7) & 0x07FF;
		rc.ch4 -= 1024;
		if(rc.ch1 <= 50 && rc.ch1 >= -50){rc.ch1 = 0;}
		if(rc.ch2 <= 50 && rc.ch2 >= -50){rc.ch2 = 0;}
		if(rc.ch3 <= 50 && rc.ch3 >= -50){rc.ch3 = 0;}
		if(rc.ch4 <= 50 && rc.ch4 >= -50){rc.ch4 = 0;}
		rc.sw1 = ((rc_data_buffer[5] >> 4) & 0x000C) >> 2;
		rc.sw2 = (rc_data_buffer[5] >> 4) & 0x0003;
	}
	else
	{
		rc.ch1=0;	rc.ch2=0;	rc.ch3=0;	rc.ch4=0;
	}
}


