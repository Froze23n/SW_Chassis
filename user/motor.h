#ifndef _MOTOR_H_
#define _MOROR_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

struct gm6020_data_t {
  //int16_t raw_angle_buff; //0-8191,用于过零处理
  //float old_velocity; //rad/s,用于均值滤波
  
  //int32_t Circle; //转动圈数
  float RealaAngle; //相对角度
  //float TotalAngle; //累计角，弧度
  float Velocity; //角速度，rad/s
} ;

/*-----------------------externs-----------------------*/
//can1数据：
extern struct gm6020_data_t GM6020_DATA[4];
//can2数据：
extern float M3508_Velocity[4];

/*函数声名*/
void Enable_Motors(void);
void GM6020_Tx(int16_t voltage[4]);
void M3508_Tx(int16_t current[4]);



#ifdef __cplusplus
}
#endif

#endif