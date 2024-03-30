#include "chassis.h"
#include "main.h"
#include "rc.h"
#include "motor.h"
#include "pid.h"
#include <math.h>
#define PI (3.1415927F)

/*
CAN ID(正面)
/1   2\
   ^   
\4   3/
CAN ID(反面)
/2   1\
   x   
\3   4/
*/

/*--------------------------------------常量--------------------------------------*/
const float STEERoffset[4]={57,78,-100,60}; //这个值取决于舵6020的实际零点位置
const int32_t SW_Speed=300; //野蛮程度

/*--------------------------------------变量--------------------------------------*/
float RC_Angle[4] = {-PI/4 , PI/4 , -PI/4 , PI/4}; //遥控器解算出来的GM6020指向
float RC_Velocity[4] = { 0,0,0,0 }; //遥控器解算出来的每个轮子的速度

float GM6020_Angle_Error[4];//optimized  //优化并计算出来的，gm6020该往哪儿转
float M3508_Velocity_Error[4];//calculated  //计算出来的，M3508该转多块，往哪个方向转

/*--------------------------------------函数--------------------------------------*/
void RC_Translation(void)
{
    float x = -rc.LX;
    float y = rc.LY;
    float z = rc.RX;
    float vx[4];
    float vy[4];
    if( (0==x) && (0==y) && (0==z) )
    {
        RC_Angle[0] =-PI/4; RC_Angle[1] = PI/4; RC_Angle[2] =-PI/4; RC_Angle[3] = PI/4;
        RC_Velocity[0] = 0; RC_Velocity[1] = 0; RC_Velocity[2] = 0; RC_Velocity[3] = 0;
        //HAL_GPIO_WritePin(Green_GPIO_Port,Green_Pin,GPIO_PIN_SET);
    }
    else
    {
        vx[0]=x-z/sqrt(2); vy[0]=y+z/sqrt(2);//ID=1 -+
		vx[1]=x-z/sqrt(2); vy[1]=y-z/sqrt(2);//ID=2 --
		vx[2]=x+z/sqrt(2); vy[2]=y-z/sqrt(2);//ID=3 +-
		vx[3]=x+z/sqrt(2); vy[3]=y+z/sqrt(2);//ID=4 ++
        for(int8_t i=0; i<4; i++)
        {
            //得到角度
            RC_Angle[i] = atan2f( vy[i],vx[i] );//(-pi,pi]
            //得到速度
            RC_Velocity[i] = sqrtf( powf(vx[i],2) + powf(vy[i],2) );
        }
    }
}

void RC_Optimization(void)
{
    static int32_t SW_Direction[4] = { 1,-1,1,-1 };
    float expA,truA;
    for(int8_t i=0; i<4; i++)
    {
        expA = RC_Angle[i] + STEERoffset[i]*PI/180.0f;
        truA = GM6020_DATA[i].RealaAngle;
        //角度逼近
        while(expA-truA> PI){ expA-=PI*2;}
        while(expA-truA<-PI){ expA+=PI*2;}
        //反转处理
        if(expA-truA > PI/2){
            expA-=PI;
            SW_Direction[i] *= -1;
        }else if(expA-truA < -PI/2){
            expA+=PI;
            SW_Direction[i] *= -1;
        }
        GM6020_Angle_Error[i] = expA-truA;
        M3508_Velocity_Error[i] = RC_Velocity[i]*SW_Direction[i]*SW_Speed - M3508_Velocity[i];
    }
}

void CHAS_GM6020_CMD(void)
{
    int16_t TxVoltage[4];
    TxVoltage[0] = CHAS_GM6020_PID(0, GM6020_Angle_Error[0], GM6020_DATA[0].Velocity);
    TxVoltage[1] = CHAS_GM6020_PID(1, GM6020_Angle_Error[1], GM6020_DATA[1].Velocity);
    TxVoltage[2] = CHAS_GM6020_PID(2, GM6020_Angle_Error[2], GM6020_DATA[2].Velocity);
    TxVoltage[3] = CHAS_GM6020_PID(3, GM6020_Angle_Error[3], GM6020_DATA[3].Velocity);
    //发送
    GM6020_Tx(TxVoltage);
}

void CHAS_M3508_CMD(void)
{
    int16_t TxCurrent[4];
    TxCurrent[0] = CHAS_M3508_PID(0, M3508_Velocity_Error[0]);
    TxCurrent[1] = CHAS_M3508_PID(1, M3508_Velocity_Error[1]);
    TxCurrent[2] = CHAS_M3508_PID(2, M3508_Velocity_Error[2]);
    TxCurrent[3] = CHAS_M3508_PID(3, M3508_Velocity_Error[3]);
    //发送
    M3508_Tx(TxCurrent);
}
