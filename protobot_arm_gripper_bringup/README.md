# Protobot Arm With Gripper Real Bring up   
## 电机模式设置为脉冲+方向模式   
错误解决：  
***[ERROR] Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino***   
在启动drive.launch程序时如果一直出现以上错误，一般是因为串口通信波特率过高导致，正常情况下请设置波特率57600，同时修改单片机串口通信波特率和电脑端rosserial通信波特率   
