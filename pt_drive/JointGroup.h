//JointGroup.h v1.1
//Made By Maple:190258039@qq.com
//2019年1月9日
#ifndef _BLARM_H_
#define _BLRAM_H_
#include <Arduino.h>
#include "StepperJoint.h"				//引用驱动程序头文件
#include <vector>
//机械臂类定义：
class JointGroup
{
//公开函数：
public:
	//机械臂初始化函数
  JointGroup(int joint_number):kJointNumber(joint_number)
  {
    for(int i=0;i<joint_number;++i)
    {
      joints.push_back(new StepperJoint());
    }
  }

	void initialize();
    //校准完成函数，此时机械臂应为"复位"状态
	void calibrationFinish();
	bool isCalibrationFinish();
	//
	StepperJoint& getJoint(uint8_t n);
	//判断机械臂是否在运行
	bool isRunning();
	//机械臂恢复到复位状态函数
	void reset();
	//warning
	void warning(int pin,bool key);
	const uint8_t kJointNumber;

private:


//私有变量：
private:
	bool _calibration = false;
  std::vector<StepperJoint*> joints;
};
#endif
