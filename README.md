环境配置请查看群文件hello word环境配置
SW == stearing wheel == 舵轮

motor.c : 
电机使能Enable_Motors
舵电机控制GM6020_Tx
轮电机M3508_Tx
GM6020不需要过零处理

pid.c:
CHAS_M3508_PID 单环速度M3508
CHAS_GM6020_PID+gm_velocity_to_voltage 双环GM6020

rc.c:
rc.LX 左右平移
rc.LY 前后平移
rc.RX 底盘旋转

chassis.c:
会调用rc pid motor
RC_Translation 
解读遥控器得到每个轮子的<方向向量>(速度+角度)

RC_Optimization 
如果方向向量转动度数超过90°，那么我们可以从另一个方向更快转过去，同时令轮子反转
可以节省功率，提高底盘稳定性(物理)

核心代码：
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
	
	//得到实际控制数据，然后发配给pid&motor
	GM6020_Angle_Error[i] = expA-truA;
        M3508_Velocity_Error[i] = RC_Velocity[i]*SW_Direction[i]*SW_Speed - M3508_Velocity[i];

PS: 本工程还有一些地方可以改进
摇杆(期望速度)归零时四只轮子会立刻45°抱死。好处是更快刹车，坏处是刹车太快。
建议加上看门狗，这样就可以使用遥控器远程复位