//Drive.h v1.0
//Made By Maple:190258039@qq.com
//2019年1月7日
#ifndef __DRIVE_H__
#define __DRIVE_H__

class Drive
{
	public:
    virtual void setName(const char* name_str)=0;
		virtual void initialize()=0;

		virtual bool hasLimitPosition()=0;
		virtual void setLimitPosition(double start,double end,bool dir)=0;

		virtual void setPositionOffset(double posi_offset)=0;						//radian
		virtual void setMachineReduction(double redu)=0;							//+
		virtual void setMotorDirection(bool level)=0;

		virtual bool rotate(double delta_ang, double time)=0;
		virtual bool rotateTo(double ang, double time)=0;
		virtual double getPosition()=0;												//radian

		//最大最小速度均为绝对值，不含方向
		virtual void setLimitAngleSpeed(double max_s,double min_s)=0;				//radian/s
//    virtual void setAcceleration(double acce)=0;                        //radian/s2
//    virtual void refreshSpeed()=0;

		virtual void pause()=0;
		virtual void continue_()=0;
		virtual bool isRunning()=0;
};
#endif
