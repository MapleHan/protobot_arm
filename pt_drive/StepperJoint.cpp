#include "StepperJoint.h"
//#define DEBUG_SERIAL SerialUSB/

//构造函数:
StepperJoint::StepperJoint()
{
  _has_limit = false;
  _position_offset = 0;
  _machine_redu = 1.0;
  _target_position_in_pulse = 0;
  _current_position = 0;
  _current_position_in_pulse = 0;
  _min_angle_speed = 0;
  _onestep_angle = 1.8/180.0*M_PI;
  _subdivision = 16;
  _pauseing = true;
  _dir_level = HIGH;
  _en_level = LOW;
  _interrupt_pulse_level = LOW;
}

StepperJoint::~StepperJoint()
{
  if(_timer!=NULL)
  {
    timerStop();
    digitalWriteDirect(_en_pin,!_en_level);
  }
}

void StepperJoint::setName(const char* name_str)
{
  strcpy(name,name_str);
}
//驱动初始化函数：
void StepperJoint::initialize()
{
  pinMode(_dir_pin,OUTPUT);
  digitalWrite(_dir_pin, _dir_level);
  pinMode(_en_pin,OUTPUT);
  digitalWrite(_en_pin, _en_level);
  pinMode(_pulse_pin,OUTPUT);
  digitalWrite(_pulse_pin, LOW);
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
    DEBUG_SERIAL.print(name);
    DEBUG_SERIAL.println(":debug_mode");
  #endif
}

//读是否有极限位置函数
bool StepperJoint::hasLimitPosition()
{
  return(_has_limit);
}

//设置最大最小位置函数：radian//-2PI--2PI
void StepperJoint::setLimitPosition(double start,double end,bool dir)
{
  _area = Area(start,end,dir);
  _has_limit = true;
}

//设置位置偏移值：radian
void StepperJoint::setPositionOffset(double posi_offset)
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("StepperJoint::setPositionOffset FUNCTION");
  #endif
  double pos = getPosition();
  _current_position_in_pulse += (posi_offset/_onestep_angle*_subdivision*_machine_redu);
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(name);
    DEBUG_SERIAL.print(":offset");
    DEBUG_SERIAL.print(pos);
    DEBUG_SERIAL.print(" to ");
    DEBUG_SERIAL.println(getPosition());
  #endif
  
}

//设置Joint-motor减速比函数
void StepperJoint::setMachineReduction(double redu)
{
  _machine_redu = fabs(redu);
}

//设置关节往角度值大一侧转动时电机方向信号电平
void StepperJoint::setMotorDirection(bool level)
{
  _dir_level = level;
}

//关节转动至某角度函数：radian//-2PI->2PI
bool StepperJoint::rotateTo(double ang, double time)
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("StepperJoint::rotateTo FUNCTION");
  #endif
  while(ang>=2*M_PI)
    ang-=(2*M_PI);
  while(ang<=-2*M_PI)
    ang+=(2*M_PI);
  pause();
  double cur = getPosition();
  struct Route route_p(cur,ang,true);
  struct Route route_n(cur,ang,false);
  struct Route r;
  if(!_has_limit)
  {
    if( route_p.len() <= route_n.len() )
    {
      r = route_p;
    }
    else
    {
      r = route_n;
    }
  }
  else
  {
    bool bp = _area.routeInArea(route_p);
    bool bn = _area.routeInArea(route_n);
    if(bp)
    {
      r = route_p;
    }
    else if(bn)
    {
      r = route_n;    
    }
    else
    {
      continue_();
      return(false);
    }
  }

  double delta_ang = r.len();
  long delta_pulse = abs(fabs(delta_ang)/_onestep_angle*_subdivision*_machine_redu);
  if(r.dir==true)
  {
    _target_position = _current_position+delta_ang;
    _target_position_in_pulse = _current_position_in_pulse + delta_pulse;
    digitalWriteDirect(_dir_pin,_dir_level);
  }
  else
  {
    _target_position = _current_position-delta_ang;
    _target_position_in_pulse = _current_position_in_pulse - delta_pulse;
    digitalWriteDirect(_dir_pin,!_dir_level);
  }
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(name);
    DEBUG_SERIAL.print(":rotateTo:");
    DEBUG_SERIAL.print(_target_position,6);
    DEBUG_SERIAL.print('\t');
    DEBUG_SERIAL.print(_target_position_in_pulse);
    DEBUG_SERIAL.print("   current:");
    DEBUG_SERIAL.print(_current_position,6);
    DEBUG_SERIAL.print('\t');
    DEBUG_SERIAL.println(_current_position_in_pulse);
    DEBUG_SERIAL.print("_max_angle_speed: ");
    DEBUG_SERIAL.println(_max_angle_speed,6);
  #endif
  if(delta_pulse>1)
  {
    if(time==0)
      time = delta_ang/(_max_angle_speed*0.5);
    timerStart(2*delta_pulse/time);
  }
  return(true);
}
double StepperJoint::getPosition()
{
  _current_position = _current_position_in_pulse*_onestep_angle/_subdivision/_machine_redu;
}

//设置关节极限转动速度函数//radian/s
void StepperJoint::setLimitAngleSpeed(double max_s,double min_s)
{
  _max_angle_speed = fabs(max_s);
  _min_angle_speed = fabs(min_s);
}

//中断停止，电机停转；
void StepperJoint::pause()
{
    if(_pauseing == false)
      timerStop();
}
void StepperJoint::continue_()
{
    if(_pauseing == true)
      timerStart(_frequency);
}
bool StepperJoint::isRunning()
{
  return(!_pauseing);
}

//设置步进电机控制引脚函数
void StepperJoint::setStepperDEPPin(uint8_t d_pin, uint8_t e_pin, uint8_t p_pin)
{
  _dir_pin = d_pin;
  _en_pin = e_pin;
  _pulse_pin = p_pin;
}
//设置电机使能信号电平
void StepperJoint::setEnMotorLevel(bool level)
{
  _en_level = level;
}
//设置步进电机步进角//degree
void StepperJoint::setStepperOnestepAngleInDegree(double deg)
{
  _onestep_angle = deg/180.0*M_PI;
}
//设置步进电机步进角//degree
void StepperJoint::setStepperOnestepAngleInRadian(double rad)
{
  _onestep_angle = rad;
}
//设置步进电机细分系数
void StepperJoint::setStepperSubdivision(uint8_t sub)
{
  _subdivision = sub;
}

//设置控制信号所用定时器
void StepperJoint::setTimer(DueTimer* timer,void (*isr)())
{
  _timer = timer;
  _timer->attachInterrupt(isr);
}
void StepperJoint::timerStart(double fre)
{
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("StepperJoint::timerStart FUNCTION");
  #endif
  _frequency  = fabs(fre);
  if(_frequency>(2*_max_angle_speed/_onestep_angle*_subdivision*_machine_redu))
    _frequency = (2*_max_angle_speed/_onestep_angle*_subdivision*_machine_redu);
  else if(_frequency<(2*_min_angle_speed/_onestep_angle*_subdivision*_machine_redu))
    _frequency = (2*_min_angle_speed/_onestep_angle*_subdivision*_machine_redu);
  _timer->stop();
  _timer->setFrequency(_frequency);
  _timer->start();
  _pauseing = false;
  #ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(name);
    DEBUG_SERIAL.print(":desired frequency:");
    DEBUG_SERIAL.print(fre,6);
    DEBUG_SERIAL.print("   real frequency:");
    DEBUG_SERIAL.println(_frequency,6); 
  #endif
}
void StepperJoint::timerStop()
{
  _timer->stop();
  digitalWriteDirect(_pulse_pin,LOW);
  _pauseing = true;
}
void StepperJoint::run()
{
  _interrupt_pulse_level = !_interrupt_pulse_level;
  digitalWriteDirect(_pulse_pin,_interrupt_pulse_level);
  if(_current_position_in_pulse>_target_position_in_pulse)
     _current_position_in_pulse-=_interrupt_pulse_level;
  else if(_current_position_in_pulse<_target_position_in_pulse)
     _current_position_in_pulse+=_interrupt_pulse_level;
  else
    timerStop();
}
