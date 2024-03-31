/*
所有pid计算都发生在这里
*/
#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*结构体声名*/
typedef struct
{
   float kp;
   float ki;
   float kd;
} PID_T;

/*函数声名*/
int16_t CHAS_M3508_PID(uint8_t index, float pError); //单环

int16_t CHAS_GM6020_PID(uint8_t index, float angleError, float truV); //双环
float gm_velocity_to_voltage(uint8_t index, float vError);

#ifdef __cplusplus
}
#endif

#endif
