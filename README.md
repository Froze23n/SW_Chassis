SW == stearing wheel == 舵轮

核心代码：
```
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
	
//得到实际控制数据
GM6020_Angle_Error[i] = expA-truA;
M3508_Velocity_Error[i] = RC_Velocity[i]*SW_Direction[i]*SW_Speed - M3508_Velocity[i];
```

PS:
摇杆(期望速度)归零时四只轮子会立刻45°抱死。好处是更快刹车，坏处是刹车太快。
