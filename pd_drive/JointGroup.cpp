//JointGroup.h v1.1
//Made By Maple:190258039@qq.com
//2019年1月9日
#include"JointGroup.h"

//机械臂初始化函数
void JointGroup::initialize()
{
}

//校准完成函数，此时机械臂应为"复位"状态
void JointGroup::calibrationFinish()
{
	for(uint8_t i=0;i<kJointNumber;++i)
	{
		double val = 0-getJoint(i).getPosition();
		getJoint(i).setPositionOffset(val);
	}
	while(isRunning())
		delay(10);
	_calibration = true;
}

//校准完成函数，此时机械臂应为"复位"状态
bool JointGroup::isCalibrationFinish()
{
	return(_calibration);
}

StepperJoint& JointGroup::getJoint(uint8_t n)
{
  if(n<kJointNumber)
	  return(*joints[n]);
  else
    SerialUSB.println("ERROR:n > kJointNumber!!!");
}

//判断机械臂是否在运行
bool JointGroup::isRunning()
{
	for(uint8_t i=0;i<kJointNumber;++i)
	{
		if(getJoint(i).isRunning())
			return(true);
	}
	return(false);
}

//机械臂恢复到复位状态函数
void JointGroup::reset()
{
	for(uint8_t i=0;i<kJointNumber;++i)
	{
		getJoint(i).rotateTo(0);
	}
}

//warning
void JointGroup::warning(int pin,bool key)
{
}

