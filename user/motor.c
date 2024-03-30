#include "motor.h"
#include "can.h"
#include "tim.h"

#define PI (3.1415927F)

/*------------------------------ 手册 ------------------------------//
命令     电机    编号   反馈码
0x200   M3508   1234  0x201-204
0x200   m2006   1234  0x201-204

0x1FF   M3508   5678  0x205-208
0x1FF   GM6020  1234  0x205-208
0x1FF   m2006   5678  0x205-208

0x2FF   GM6020  567   0x209-20A
*/

/* ---------------------------- 函数声明 ---------------------------- */
inline void GM6020_Data_Handler(const uint32_t CAN_ID , const uint8_t RxData[8]);
inline void M3508_Data_Handler(const uint32_t CAN_ID , const uint8_t RxData[8]);
/* ------------------------------ 常量 ------------------------------ */
const uint32_t GM6020_CMD_ID = 0x1ff;
const uint32_t GM6020_RXID_BASE = 0x205;

const uint32_t M3508_CMD_ID = 0x200;
const uint32_t M3508_RXID_BASE = 0x201;

const float _pi_over_4096_ = PI/4096.0f;
const float _rads_per_rpm_ = 2*PI/60.0f;

/*变量*/
struct gm6020_data_t GM6020_DATA[4] = {
    {.RealaAngle=-PI/4, .Velocity=0},
    {.RealaAngle=+PI/4, .Velocity=0},
    {.RealaAngle=-PI/4, .Velocity=0},
    {.RealaAngle=+PI/4, .Velocity=0},
};
float M3508_Velocity[4]={0.0f , 0.0f , 0.0f , 0.0f};

/* -------------------- 初始化:配置过滤器+打开中断-------------------- */
void Enable_Motors(void)
{
    /*配置报文过滤器*/
    CAN_FilterTypeDef CAN_Filter;
    CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK; //掩码模式
	CAN_Filter.SlaveStartFilterBank = 0;
	CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
	CAN_Filter.FilterIdHigh = 0x0000;
	CAN_Filter.FilterIdLow = 0x0000;
	CAN_Filter.FilterMaskIdHigh= 0x0000;
	CAN_Filter.FilterMaskIdLow = 0x0000; //
	if (HAL_CAN_ConfigFilter(&hcan1,&CAN_Filter)!= HAL_OK){
        /*
        倘若配置成功，它的效果是：不对数据做任何过滤
        数据会被存放到 FIFO0 中
        FIFO: 可以理解成一个队列，first in first out 先进先出 正常情况下新数据会顶替掉旧数据
        开发板上有两个FIFO缓冲区
        */
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//失败则亮灯警告 以便debug
        Error_Handler();
    }
	CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	CAN_Filter.FilterBank = 14;
	CAN_Filter.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan2,&CAN_Filter)!= HAL_OK){
        /*
        上面是对can1配置，这里对can2进行配置
        这里我们把can2接收到的数据放到FIFO1里
        当然放FIFO0里也可以，只不过这样会导致所有处理代码都塞到同一个回调函数: HAL_CAN_RxFifo0MsgPendingCallback 导致它看起来很臃肿
        */
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//失败则亮灯警告
        Error_Handler();
    }
    /*使能can，打开中断*/
    HAL_CAN_Start(&hcan1); //使能can1
    HAL_CAN_Start(&hcan2); //使能can2
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  //can1 的 FIFO0 收到报文会产生中断(别忘了先在cubemx里把你想要开启的中断提前勾上)
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  //不多赘述
    HAL_TIM_Base_Start_IT(&htim3);//1ms一次中断用于电机数据处理 can1~tim3~舵 4个GM6020
    HAL_TIM_Base_Start_IT(&htim5);//1ms一次中断用于电机数据处理 can2~tim5~轮 4个M3508
}

/* ----------------------- 发送函数：舵----------------------- */
void GM6020_Tx(int16_t voltage[4])
{
    //数据
    uint8_t TxData[8]; //一个普通的数据帧8个字节
    TxData[0] = (uint8_t)(voltage[0]>>8);
    TxData[1] = (uint8_t)voltage[0];
    TxData[2] = (uint8_t)(voltage[1]>>8);
    TxData[3] = (uint8_t)voltage[1];
    TxData[4] = (uint8_t)(voltage[2]>>8);
    TxData[5] = (uint8_t)voltage[2];
    TxData[6] = (uint8_t)(voltage[3]>>8);
    TxData[7] = (uint8_t)voltage[3];
    //header
    CAN_TxHeaderTypeDef TxHeader = {
        .DLC = 8,  //4个电机*2 见手册
        .IDE = CAN_ID_STD,    // 标准帧
        .RTR = CAN_RTR_DATA,  // 数据帧
        .StdId = GM6020_CMD_ID//0x1ff 见手册
    };
    //mailbox
    uint32_t TxBox = CAN_TX_MAILBOX0;
    /*
    发送邮箱 似乎是个与接收缓冲区相呼应的东西，应该是每个can上3个(或者是总共3个)
    算了不重要，我们总共就两组报文要发送，肯定用不完的
    */
    //发送
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxBox) != HAL_OK)
    {
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
        //没必要添加error handler 别把自己写死了
    }
}

/* ----------------------- 发送函数：轮----------------------- */
//不多赘述
void M3508_Tx(int16_t current[4])
{
    //数据
    uint8_t TxData[8];
    TxData[0] = (uint8_t)(current[0]>>8);
    TxData[1] = (uint8_t)current[0];
    TxData[2] = (uint8_t)(current[1]>>8);
    TxData[3] = (uint8_t)current[1];
    TxData[4] = (uint8_t)(current[2]>>8);
    TxData[5] = (uint8_t)current[2];
    TxData[6] = (uint8_t)(current[3]>>8);
    TxData[7] = (uint8_t)current[3];
    //header
    CAN_TxHeaderTypeDef TxHeader = {
        .DLC = 8,  //4个电机*2
        .IDE = CAN_ID_STD,    // 标准帧
        .RTR = CAN_RTR_DATA,  // 数据帧
        .StdId = M3508_CMD_ID//0x200
    };
    //mailbox
    uint32_t TxBox = CAN_TX_MAILBOX1;
    //发送
    if(HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxBox) != HAL_OK)
    {
        HAL_GPIO_WritePin(Red_GPIO_Port,Red_Pin,GPIO_PIN_SET);//警告
    }
}

/* ------------------------- 接收函数 ------------------------- */
//helper function
inline void GM6020_Data_Handler(const uint32_t CAN_ID , const uint8_t RxData[8])
{
    //index
    uint32_t i = CAN_ID-GM6020_RXID_BASE; //见手册
    //数据
    int16_t rawAngle = ( (RxData[0]<<8) | RxData[1] ); //0-8191
    int16_t rawVelocity = ( (RxData[2]<<8) | RxData[3] ); //rpm
    //安全措施
    if(i>=4){return;}
    //角度处理
    GM6020_DATA[i].RealaAngle = (float)rawAngle * _pi_over_4096_ ; //弧度制相对角
    //速度处理
    GM6020_DATA[i].Velocity = (float)rawVelocity * _rads_per_rpm_ ; //全部转成弧度
}
//helper function
inline void M3508_Data_Handler(const uint32_t CAN_ID , const uint8_t RxData[8])
{
    //变量
    uint32_t i = CAN_ID - M3508_RXID_BASE;
    int16_t origVelocity = ( (RxData[2]<<8) | RxData[3] );
    //安全措施
    if(i>=4){return;}
    //赋值
    M3508_Velocity[i] = (float)origVelocity * _rads_per_rpm_ ;
}


//接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    //can1--GM6020
    if (hcan == &hcan1) 
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)  // 获得接收到的数据头和数据
        {
            GM6020_Data_Handler(RxHeader.StdId, RxData);
        }
        HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    //can2--m3508
    if (hcan == &hcan2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
        {
            M3508_Data_Handler(RxHeader.StdId, RxData);
        }
    }
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);  // 再次使能FIFO0接收中断
}

/*废弃版本
废弃原因：舵轮底盘不需要任何过零处理
struct gm6020_data_t GM6020_DATA[4] = {
    {.raw_angle_buff=4096, .Circle=0, .RealaAngle=0, .TotalAngle=0, .old_velocity=0, .Velocity=0},
    {.raw_angle_buff=4096, .Circle=0, .RealaAngle=0, .TotalAngle=0, .old_velocity=0, .Velocity=0},
    {.raw_angle_buff=4096, .Circle=0, .RealaAngle=0, .TotalAngle=0, .old_velocity=0, .Velocity=0},
    {.raw_angle_buff=4096, .Circle=0, .RealaAngle=0, .TotalAngle=0, .old_velocity=0, .Velocity=0},
};
inline void GM6020_Data_Handler(const uint32_t CAN_ID , const uint8_t RxData[8])
{
    //指针变量
    uint32_t i = CAN_ID-GM6020_RXID_BASE;
    struct gm6020_data_t *dataptr = GM6020_DATA + i ;
    //数据
    int16_t rawAngle = ( (RxData[0]<<8) | RxData[1] ); //0-8191
    int16_t rawVelocity = ( (RxData[2]<<8) | RxData[3] ); //rpm
    int16_t angleDiff = rawAngle - (dataptr->raw_angle_buff);
    float newVelocity = (float)rawVelocity * _rads_per_rpm_ ;
    //安全措施
    if(i>=4){return;}
    //角度处理
    if(angleDiff< -4100){ dataptr->Circle += 1;}
    if(angleDiff>  4100){ dataptr->Circle -= 1;}
    dataptr->RealaAngle = (float)rawAngle*_pi_over_4096_ ; //弧度制相对角
    dataptr->TotalAngle = (dataptr->RealaAngle) + (dataptr->Circle)*2*PI;
    //速度处理
    dataptr->Velocity = (newVelocity + dataptr->old_velocity)/2 ; //弧度制速度
    //迭代更新
    dataptr->old_velocity = newVelocity;
    dataptr->raw_angle_buff = rawAngle;
}
*/
