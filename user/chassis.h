/*
封装了定时器中断触发后要干的活
包含对遥控器数据的解读 对GM6020电机数据的解算 对总共8个电机的控制
封装出来的函数放到 "stm32f4xx_it.c" 的对应位置
*/
#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/*函数声名*/
void RC_Translation(void);
void RC_Optimization(void);

void CHAS_GM6020_CMD(void);
void CHAS_M3508_CMD(void);


#ifdef __cplusplus
}
#endif

#endif
