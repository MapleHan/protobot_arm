void joint0Run()
{
  arm.getJoint(0).run();
}
void joint1Run()
{
  arm.getJoint(1).run();
}
void joint2Run()
{
  arm.getJoint(2).run();
}
void joint3Run()
{
  arm.getJoint(3).run();
}
void jointG0Run()
{
  gri.getJoint(0).run();
}

#define NULL_PIN 53
void armInit()
{
  arm.getJoint(0).setName(kArmJointName[0]);
  arm.getJoint(0).setMachineReduction(56.8);
  arm.getJoint(0).setStepperOnestepAngleInDegree(0.087890625);//电子齿轮分子8
  arm.getJoint(0).setStepperSubdivision(1);
  arm.getJoint(0).setMotorDirection(HIGH);
  arm.getJoint(0).setLimitAngleSpeed(1.571);
  arm.getJoint(0).setStepperDEPPin(7,NULL_PIN,6);
  arm.getJoint(0).setTimer(&Timer1,joint0Run);
  arm.getJoint(0).initialize();

  arm.getJoint(1).setName(kArmJointName[1]);
  arm.getJoint(1).setLimitPosition(-M_PI/4,M_PI/4,true);
  arm.getJoint(1).setMachineReduction(113.6);
  arm.getJoint(1).setStepperOnestepAngleInDegree(0.087890625);
  arm.getJoint(1).setStepperSubdivision(1);
  arm.getJoint(1).setMotorDirection(HIGH);
  arm.getJoint(1).setLimitAngleSpeed(1.571);
  arm.getJoint(1).setStepperDEPPin(31,NULL_PIN,30);
  arm.getJoint(1).setTimer(&Timer6,joint1Run);
  arm.getJoint(1).initialize();

  arm.getJoint(2).setName(kArmJointName[2]);
  //arm.getJoint(2).setLimitPosition(-M_PI/4.0*3.0,M_PI/4.0*3.0,true);
  arm.getJoint(2).setMachineReduction(56.8);
  arm.getJoint(2).setStepperOnestepAngleInDegree(0.087890625);
  arm.getJoint(2).setStepperSubdivision(1);
  arm.getJoint(2).setMotorDirection(HIGH);
  arm.getJoint(2).setLimitAngleSpeed(1.571);
  arm.getJoint(2).setStepperDEPPin(33,NULL_PIN,32);
  arm.getJoint(2).setTimer(&Timer7,joint2Run);
  arm.getJoint(2).initialize();

  arm.getJoint(3).setName(kArmJointName[3]);
  arm.getJoint(3).setMachineReduction(56.8);
  arm.getJoint(3).setStepperOnestepAngleInDegree(0.087890625);
  arm.getJoint(3).setStepperSubdivision(1);
  arm.getJoint(3).setMotorDirection(HIGH);
  arm.getJoint(3).setLimitAngleSpeed(1.571);
  arm.getJoint(3).setStepperDEPPin(35,NULL_PIN,34);
  arm.getJoint(3).setTimer(&Timer8,joint3Run);
  arm.getJoint(3).initialize();     

  gri.getJoint(0).setName(kGripperJointName[0]);
  gri.getJoint(0).setMachineReduction(56.8);
  gri.getJoint(0).setStepperOnestepAngleInDegree(0.087890625);
  gri.getJoint(0).setStepperSubdivision(1);
  gri.getJoint(0).setMotorDirection(HIGH);
  gri.getJoint(0).setLimitAngleSpeed(1.571);
  gri.getJoint(0).setStepperDEPPin(35,NULL_PIN,34);
  gri.getJoint(0).setTimer(&Timer2,jointG0Run);
  gri.getJoint(0).initialize(); 
  gri.initialize();
}
