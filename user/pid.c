#include "pid.h"

PID_T M3508V = {45.0f , 0.2f , '\0'}; //轮 速度
PID_T GM6020A = {80.0f , '\0' , 3.0f}; //舵 角度
PID_T GM6020V = {600.0f , 100.0f , '\0'}; //舵 速度


#define CHAS_I_ERROR_LIMIT (35000.0f)
/*
输入：ID(0-3)&期望速度
输出：PI控制得到的电流
*/
int16_t CHAS_M3508_PID(uint8_t index, float pError)
{
    static float iError[4] = {0,0,0,0};
    if(index<0 || 3<index){return 0;}  //防呆保护
    //计算iError
    iError[index] += pError;
    //积分限幅
    if(iError[index]>= CHAS_I_ERROR_LIMIT){iError[index] = CHAS_I_ERROR_LIMIT;}
    if(iError[index]<=-CHAS_I_ERROR_LIMIT){iError[index] =-CHAS_I_ERROR_LIMIT;}
    //计算电流
    float output = pError * M3508V.kp + iError[index] * M3508V.ki ;
    //限制理论输出+-16384对应+-20A
    if(output>+16300.0f){output=+16300.0f;}
    if(output<-16300.0f){output=-16300.0f;}
    //返回结果
    return (int16_t)output;   
}

/*
舵向电机双环pid
pError <=> angleError
*/
int16_t CHAS_GM6020_PID(uint8_t index, float angleError, float truV)
{
    //角度到速度：
    static float oldError[4] = {0,0,0,0};
    float dError;
    float expV;
    float Voltage;
    if(index<0 || 3<index){return 0;}  //防呆保护
    //pid计算与迭代
    dError = angleError - oldError[index];
    oldError[index] = angleError;
    //结合前馈，得到预期速度
    expV = angleError * GM6020A.kp + dError * GM6020A.kd;
    if(expV > 30){expV = 30;}
    if(expV <-30){expV =-30;}//300 rpm ~ 10pi rad/s
    //速度到电压
    Voltage = gm_velocity_to_voltage(index, expV-truV );
    return (int16_t)Voltage;
}

#define GM_I_ERROR_LIMIT (80.0f) //为保证速度环灵敏度，不宜过大
/*内部函数helper function: vError = expV - truV*/
float gm_velocity_to_voltage(uint8_t index, float vError)
{
    static float iError[4] = {0,0,0,0};
    float output;
    iError[index] += vError;
    if(iError[index] > GM_I_ERROR_LIMIT){iError[index] = GM_I_ERROR_LIMIT;}
    if(iError[index] <-GM_I_ERROR_LIMIT){iError[index] =-GM_I_ERROR_LIMIT;}
    //计算加和
    output = vError * GM6020V.kp + iError[index] * GM6020V.ki;
    //限制理论电压上限+-2600mV
    if(output > 26000.0f){output = 26000.0f;}
    if(output <-26000.0f){output =-26000.0f;}
    return output;
}
