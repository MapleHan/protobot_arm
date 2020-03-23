//arm_ros_driver.ino V1.1;
//Made By Maple:190258039@qq.com
//2019年10月17日
//所有角度变量统一使用弧度制
#define USE_USBCON//DUE控制版需要启用USE_USBON,定义在第一行,UNO不需要
#include "JointGroup.h"                             //机械臂控制库文件
#include "DueTimer.h"                               //DUE定时器库文件
#include "Trajectory.h"                             //轨迹数据类型库文件
#include <Servo.h>                                  //舵机库文件
#include <ros.h>                                    //ros库文件
#include <ros/time.h>                               //ros时间库文件
#include <sensor_msgs/JointState.h>                 //ros关节状态消息库文件
#include <std_msgs/Float32MultiArray.h>             //ros标准消息类型/多维float数组消息库文件
#include <std_msgs/Bool.h>                          //ros标准消息类型/布尔消息库文件
const int kArmJointNumber = 4;                      //宏定义机械臂关节数量即自由度，不包括机械手
const char kArmJointName[kArmJointNumber][20] = { "rot_joint","uarm_joint","larm_joint","flange_joint"};
const int kGripperJointNumber = 1;                  //机械手关节数量即自由度
const char kGripperJointName[kGripperJointNumber][20] = {"finger1_joint" };
const int KGripperPin = 3;                          //机械手舵机引脚定义
#define MOVE  true
#define STOP  false
JointGroup arm = JointGroup(kArmJointNumber);				//机械臂运动组对象
Trajectory current_tra;                             //机械臂运动轨迹数据变量
Servo gripper;                                      //机械手舵机对象
const float KGripperInitPWM = 90;                   //gripper init position PWM
const float KGripperInitRad = 0;                    //gripper init position radian
const float kServoDistance = 270;
const int kServoDir = -1;
int gripper_cpwm = KGripperInitPWM;                 //gripper current position
int gripper_tpwm = gripper_cpwm;                    //gripper target position
bool gripper_state = STOP;                          //gripper state,STOP or MOVE
unsigned long jspdt = 30;                           //ms,joint state publish delta time
unsigned long ljspt = millis();                     //ms,last joint state publish time
unsigned long heartbeat = millis();                 //ms

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
	SerialUSB.begin(57600);
  armInit();
  gripper.attach(KGripperPin);
  gripper.write(gripper_tpwm);
  topicInit();
}


void loop()
{
  if(millis()-heartbeat>5000)
  {
    SerialUSB.println("peng peng ... "+String(millis()));
    heartbeat = millis();
  }
  //test();
  if(!current_tra.isNull())
  {
    if(!arm.isRunning())
    {
      Waypoint* next_point = current_tra.getNextWaypoint();
      for(uint8_t i=0;i<arm.kJointNumber;++i)									//角度控制信息
      {
        arm.getJoint(i).rotateTo(next_point->pos.getValue(i),(next_point)->time);
//        char info[60];
//        sprintf(info,"arm joint %i rotate to %010.6f duration %010.6f s",i,next_point->pos.getValue(i),(next_point)->time);
//        SerialUSB.println(info);
      }
    }
  }

  if(gripper_state == MOVE)
  {
    if(gripper_tpwm == gripper_cpwm)
    {
      gripper_state = STOP;
    }
    else
    {
      gripper_cpwm += (gripper_tpwm-gripper_cpwm)/abs(gripper_tpwm-gripper_cpwm);
      gripper.write(gripper_cpwm);
    }
  }

	if(arm.isCalibrationFinish()&&(millis()>jspdt))
	{
		pubJointState();
		ljspt+= jspdt;
		if( (arm.isRunning()==true)||(gripper_state==MOVE) )
			jspdt = 10;
		else
			jspdt = 100;
	}
  topicSpin();
}




void test()
{
	static double run_to = 1.56;
	Serial.println("******************************************");
	Serial.println("************* Joint rotateTo *************");
	arm.warning(13,false);
	bool key = true;
	char pos[60];
	for(int i=0;i<kArmJointNumber;++i)
	{
		key = arm.getJoint(i).rotateTo(run_to);
		arm.warning(LED_BUILTIN,!key);
	}
	while(arm.isRunning())
	{
		delay(200);
		for(int j=0;j<kArmJointNumber;++j)
		{
			sprintf(pos,"%s: current_position: %010.6f",arm.getJoint(j).name,arm.getJoint(j).getPosition());
			Serial.println(pos);
		}
	}
	Serial.println("******************************************");
	run_to *= -1;
	delay(2000);
}
