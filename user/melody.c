#include "melody.h"
#include "tim.h"

void MCU_BeepUp(void)
{
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3); //使能pwm，htim4和Channel_3是cubemx里配好的
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,500-1); //调整占空比，这里的效果是占空比变为 50%
    HAL_Delay(100); //响上0.1秒足够了
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0); //占空比为0 也就是没有PWM波 蜂鸣器关闭
}

void Play_See_You_Again(void)
{
    //Todo :)
}