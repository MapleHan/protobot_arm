#include<Arduino.h>
struct JointPos
{
public:
	JointPos(int size=4):_size(size)
	{
    _pos = new double[_size]();
		for(uint8_t i=0;i<_size;++i)
			_pos[i] = 0.0;
	}

	void addValue(uint8_t n,double p)
	{
		if(n<_size)
			_pos[n] = p;
	}

	double getValue(uint8_t n)
	{
	if(n<_size)
	  return(_pos[n]);
	else
	  return(0);
	}

private:
  uint8_t _size;
	double* _pos;
};

struct Waypoint 
{
  JointPos pos; // 终点位置
  double time = 0;
  Waypoint *next = NULL;
};

class Trajectory
{
public:
	Trajectory();
	~Trajectory();
	Waypoint* getNextWaypoint();
	void addWaypoint(Waypoint p);
  bool isNull();
  uint16_t getSize();
  void clear();
private:
	Waypoint* _begin;
	Waypoint* _last;
  Waypoint _next;
  uint16_t _size;
};
