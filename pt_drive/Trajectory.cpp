#include "Trajectory.h"

Trajectory::Trajectory()
{
	_begin = NULL;
	_last = NULL;
}

Trajectory::~Trajectory()
{
	Waypoint *p = _begin;
	while(p)
	{
		Waypoint *del = p;
		p = p->next;
		delete del;
		del = NULL;
	}
 _begin = NULL;
 _last = NULL;
}

void Trajectory::clear()
{
  Waypoint *p = _begin;
  while(p)
  {
    Waypoint *del = p;
    p = p->next;
    delete del;
    del = NULL;
  }
  _begin = NULL;
  _last = NULL;
  _size = 0;
}

void Trajectory::addWaypoint(Waypoint p)
{
//  SerialUSB.println("Trajectory::addWaypoint FUNCTION");
	Waypoint* new_point = new Waypoint();
	new_point->pos = p.pos;
  new_point->time = p.time;
	new_point->next = NULL;
	if(_begin == NULL)
	{
		_begin = new_point;
		_last = _begin;
//    char out[20];
//    sprintf(out,"%u",_begin);
//    SerialUSB.println("_begin："+String(out));
	}
	else
	{
		_last->next = new_point;
		_last = new_point;
	}
  _size++;
}

Waypoint* Trajectory::getNextWaypoint()
{
//  SerialUSB.println("Trajectory::getNextWaypoint FUNCTION");
  if(_begin!=NULL)
  {
//    char out[20];
//    sprintf(out,"%u",_begin);
//    SerialUSB.println("_begin："+String(out));
    _next = *_begin;
    Waypoint* del_point = _begin;
    _begin = _begin->next;
    delete del_point;
    _size--;
    return(&_next);
  }
  return(NULL);
}

uint16_t Trajectory::getSize()
{
  return(_size);
}

bool Trajectory::isNull()
{
  if(_size<=0)
    return(true);
  return(false);
}
