# Protobot Arm With Gripper Real Bring up 
## 下载程序前请确认电机的脉冲与方向引脚是否与程序内定义的相同   
## 在上位机软件中将电机模式设置为脉冲+方向模式  
## 在上位机软件中将电机的目标速度速度设置为1500  
## 在上位机软件中将电机的电子齿轮分子改为8，分母改为1  

错误解决：  
***[ERROR] Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino***   
在启动drive.launch程序时如果一直出现以上错误，一般是因为串口通信波特率过高导致，正常情况下请设置波特率57600，同时修改单片机串口通信波特率和电脑端rosserial通信波特率   
