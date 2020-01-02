//StepperJointe.h v1.0
//Made By Maple:190258039@qq.com
//2019年1月7日
#pragma once
#include <math.h>
#include <Arduino.h>
#include "Drive.h"
#include "DueTimer.h"
inline float f_mod(float a, float n) 
{          
    return a - n * floor(a / n);
}

//步进电机驱动类
class StepperJoint:public Drive
{
  struct Route
  {
    bool dir;
    double s_ang;
    double e_ang;
    Route(double start, double end,bool direction) 
    {
      dir = direction;
      s_ang = f_mod(start + M_PI, 2 * M_PI) - M_PI;//s_ang -> [-PI, PI)
      if (dir == true)
      {
        e_ang = f_mod(end-start, 2 * M_PI) + s_ang;
        if(e_ang > 2 * M_PI)
        {
          e_ang -= 2 * M_PI;
          s_ang -= 2 * M_PI;
        }
      } else if(dir == false)
      {
        e_ang = s_ang - f_mod(start - end, 2 * M_PI);
        if(e_ang < -2 * M_PI)
        {
          e_ang += 2 * M_PI;
          s_ang += 2 * M_PI;
        }
      }
    }
    Route()
    {
      dir = true;
      s_ang = e_ang = 0;
    }
    double len()
    {
      return dir ? e_ang - s_ang : s_ang - e_ang;
    }
  };

  struct Area
  {
    struct  Route route_p;
    struct  Route route_n;
    Area(double start, double end, bool direction=true)
    {
      if(direction == true)
      {
        route_p = Route(start, end, true);
        route_n = Route(end, start, false);
      } else {
        route_p = Route(end, start, true);
        route_n = Route(start, end, false);
      }
      
    }

    Area(){;}

    bool routeInArea(struct Route rou)
    {
      if(rou.dir==true)
      {
        double l = Route(rou.s_ang, route_p.e_ang, true).len();
        return route_p.len() >= l && l >= rou.len();
        // return (rou.s_ang>=route_p.s_ang&&rou.e_ang<=route_p.e_ang);
      }
      else if(rou.dir==false)
      {
        double l = Route(rou.s_ang, route_n.e_ang, false).len();
        return route_n.len() >= l && l >= rou.len();
        // return (rou.s_ang<=route_n.s_ang&&rou.e_ang>=route_n.e_ang);
      }
      else
        return (false);
    }
  };

  //公开函数：
  public:
    //构造函数、析构函数
    StepperJoint();
    ~StepperJoint();
    //设置关节名称
    virtual void setName(const char* name_str);
    //初始化函数
    virtual void initialize();

    //读关节是否有极限位置函数
    virtual bool hasLimitPosition();
    //设置关节最大最小位置函数：radian
    virtual void setLimitPosition(double start,double end,bool dir=true);       //-2PI--2PI

    //设置关节位置偏移值：radian，默认0
    virtual void setPositionOffset(double posi_offset);
    //设置Joint-motor减速比函数，默认1.0//+
    virtual void setMachineReduction(double redu=1);
    //设置关节往角度值大一侧转动时电机方向信号电平，默认高电平
    virtual void setMotorDirection(bool level);
    virtual bool rotate(double delta_ang, double time){;};
    //关节转动至某角度函数：radian
    virtual bool rotateTo(double ang, double time=0);
    //读关节目前的角度函数：radian
    virtual double getPosition();

    //设置关节极限转动速度函数，radian/s，电机转动前必须设置
    virtual void setLimitAngleSpeed(double max_s,double min_s=0);

    //关节暂停转动
    virtual void pause();
    //取消暂停，继续转动
    virtual void continue_();
    virtual bool isRunning();

    //设置步进电机控制引脚函数
    void setStepperDEPPin(uint8_t d_pin, uint8_t e_pin, uint8_t p_pin);
    //设置电机使能信号电平，默认高电平
    void setEnMotorLevel(bool level);
    //设置步进电机步进角，degree，默认1.8°
    void setStepperOnestepAngleInDegree(double deg=1.8);
    //设置步进电机步进角,radian,默认1.8/180.0*M_PI
    void setStepperOnestepAngleInRadian(double rad=1.8/180.0*M_PI);
    //设置步进电机细分系数，默认16
    void setStepperSubdivision(uint8_t sub=16);
    //设置控制信号所用定时器，电机转动前必须设置
    void setTimer(DueTimer* timer,void (*isr)());
    void run();
    char name[20];

  //私有函数
  private:
    //faster digital write//only support due
    static void digitalWriteDirect(int pin, boolean val)
    {
      if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
      else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
    }
    //定时器中断函数：
    void timerStart(double fre);
    void timerStop();
  //私有变量：
  private:
    //是否有极限
    bool _has_limit;
    //最大、最小位置
    struct Area _area;
    //位置偏移量
    double _position_offset;
    //joint-motor减速比
    double _machine_redu;
    //目标位置、当前位置：
    double _target_position;
    long _target_position_in_pulse;
    double _current_position;
    volatile long _current_position_in_pulse;
    double _frequency;
    //最大、最小速度
    double _max_angle_speed;
    double _min_angle_speed;
    //电机步距角、驱动器细分数
    double _onestep_angle;
    double _subdivision;
    //运行状态相关变量：
    bool _pauseing;
    uint8_t _dir_pin;
    uint8_t _en_pin;
    uint8_t _pulse_pin;
    bool _dir_level;
    bool _en_level;
    volatile bool _interrupt_pulse_level;
    DueTimer* _timer;
};
