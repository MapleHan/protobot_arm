//BLArm Joint Trajectory Action Service
//made by Maple 03/27/2019
//Email 190258039@qq.com

#include <ros/ros.h>
#include <signal.h>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
inline double f_mod(double a, double n)
{
    return a - n * floor(a / n);
}

class FollowJointTrajectoryAction
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
private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
  typedef actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> GoalHandle;
  ros::NodeHandle _node;
  FJTAS _action_server;
  GoalHandle _active_goal;
  bool _has_active_goal;
  std::string _action_name;
  ros::Publisher _pub_trajectory_command;
  ros::Subscriber _sub_joint_state;
  ros::Timer _watchdog_timer;
  trajectory_msgs::JointTrajectory _current_traj;
  sensor_msgs::JointStateConstPtr _current_joint_state;

  std::vector<std::string> _joint_names;
  double _goal_constraint;
  double _goal_time_constraint;
  double _allowed_execution_duration_scaling;

public:
  FollowJointTrajectoryAction(ros::NodeHandle &n,std::string name):
    _node(n),
    _action_name(name),
    _action_server(n,name,
                   boost::bind(&FollowJointTrajectoryAction::goalCB,this,_1),
                   boost::bind(&FollowJointTrajectoryAction::cancelCB,this,_1),
                   false),
    _has_active_goal(false)
  {
    using namespace XmlRpc;
    XmlRpc::XmlRpcValue controller_list;
    ros::Time t0 = ros::Time::now();
    ros::Time t1 = ros::Time::now() + ros::Duration(3.0);
    while (!_node.getParam("controller_list",controller_list))
    {
      if(ros::Time::now() > t1 )
      {
        ROS_INFO("Waiting Param controller_list!");
        t1 += ros::Duration(3.0);
      }
      if(ros::Time::now() > t0 + ros::Duration(15.0))
      {
        ROS_ERROR("No Find Param controller_list!");
        exit(1);
      }
      ros::Duration(0.1).sleep();
    }
    ROS_ASSERT(controller_list.getType()==XmlRpc::XmlRpcValue::TypeArray);
    int controller_index = -1;
    for(int i=0;i<controller_list.size();++i)
    {
      ROS_ASSERT(controller_list[i].getType()==XmlRpc::XmlRpcValue::TypeStruct);
      for (XmlRpc::XmlRpcValue::iterator j=controller_list[i].begin(); j!=controller_list[i].end(); ++j) 
      {
        if(j->first == "name")
        {
          ROS_ASSERT(j->second.getType()==XmlRpc::XmlRpcValue::TypeString);
          if(std::string(j->second)=="arm_group_controller")
          {
            controller_index = i;
            break;
          }
        }
      }
    }
    if(controller_index != -1)
    {
      for (XmlRpc::XmlRpcValue::iterator j=controller_list[controller_index].begin(); j!=controller_list[controller_index].end(); ++j) 
      {
        if(j->first == "joints")
        {
          if(j->second.getType()==XmlRpc::XmlRpcValue::TypeArray)
          {
            for(int k=0; k<j->second.size(); ++k) 
            {
              ROS_ASSERT(j->second[k].getType()==XmlRpc::XmlRpcValue::TypeString);
              ROS_INFO("joints[%d]: %s",k,std::string(j->second[k]).c_str());
              _joint_names.push_back(std::string(j->second[k]));
            }
          }
        }
      }
    }
    _node.param("trajectory_execution/allowed_start_tolerance",_goal_constraint,0.005);
    ROS_INFO("_goal_constraint = %f",_goal_constraint);
    _node.param("trajectory_execution/allowed_execution_duration_scaling",_allowed_execution_duration_scaling,1.2);
    _node.param("trajectory_execution/allowed_goal_duration_margin",_goal_time_constraint,1.0);
    ROS_INFO("_allowed_execution_duration_scaling= %f",_allowed_execution_duration_scaling);
    ROS_INFO("_goal_time_constraint= %f",_goal_time_constraint);
    //_pub_trajectory_command = _node.advertise<trajectory_msgs::JointTrajectory>("/command", 1);
    _pub_trajectory_command = _node.advertise<std_msgs::Float32MultiArray>("drive_command", 300);
    _sub_joint_state = _node.subscribe("/joint_states",1,&FollowJointTrajectoryAction::jointStateCB,this);
    _watchdog_timer = _node.createTimer(ros::Duration(1.0), &FollowJointTrajectoryAction::watchdog, this);
    _action_server.start();
  }

  ~FollowJointTrajectoryAction()
  {
    _pub_trajectory_command.shutdown();
    _sub_joint_state.shutdown();
    _watchdog_timer.stop();   
  }

  void cancelCB(GoalHandle gh)
  {
    ROS_INFO("Cancle Goal");
    if (_active_goal == gh)
    {
      // Stops the controller.
      // trajectory_msgs::JointTrajectory empty;
      // empty.joint_names = _joint_names;
      // _pub_trajectory_command.publish(empty);
      publishFlag(-1);
      // Marks the current goal as canceled.
      _active_goal.setCanceled();
      ROS_INFO("setCanceled!");
      _has_active_goal = false;
    }
  }

  void publishFlag(double n)
  {
    std_msgs::Float32MultiArray flag;
    flag.data.clear();
    for(int j=0;j<=_joint_names.size();++j)
    {
      flag.data.push_back(n);
    }
    _pub_trajectory_command.publish(flag);
    ros::Duration(0.01).sleep();
    _pub_trajectory_command.publish(flag);
  }
private:

  static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
  {
    if (a.size() != b.size())
      return false;

    for (size_t i = 0; i < a.size(); ++i)
    {
      if (count(b.begin(), b.end(), a[i]) != 1)
        return false;
    }
    for (size_t i = 0; i < b.size(); ++i)
    {
      if (count(a.begin(), a.end(), b[i]) != 1)
        return false;
    }

    return true;
  }

  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (_has_active_goal)
    {
      bool should_abort = false;
      if (!_current_joint_state)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a joint state message.");
      }
      else if ((now - _current_joint_state->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the joint in %.3lf seconds",
                 (now - _current_joint_state->header.stamp).toSec());
      }

      if (should_abort)
      {
        // Stops the controller.
       publishFlag(-1);
        // Marks the current goal as aborted.
        _active_goal.setAborted();
        _has_active_goal = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
    // Ensures that the joints in the goal match the joints we are commanding.
    if (!setsEqual(_joint_names, gh.getGoal()->trajectory.joint_names))
    {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }
    ROS_INFO("Received a Goal");
    // Cancels the currently active goal.
    if (_has_active_goal)
    {
      // Stops the controller.
      // trajectory_msgs::JointTrajectory empty;
      // empty.joint_names = _joint_names;
      // _pub_trajectory_command.publish(empty);
      publishFlag(-1);
      // Marks the current goal as canceled.
      ROS_INFO("has a Goal");
      _active_goal.setCanceled();
      ROS_INFO("setCanceled a Goal");
      _has_active_goal = false;
    }

    gh.setAccepted();
    _active_goal = gh;
    FJTAS::Goal  goal = *_active_goal.getGoal();
    _current_traj = goal.trajectory;
    _has_active_goal = true;

    ros::Duration time_start = _current_traj.points[0].time_from_start;
    ROS_INFO("New Goal Trajectory Waypoint Count: %ld",_current_traj.points.size());
    ROS_WARN("start publish trajectory");
    publishFlag(-1);
    std_msgs::Float32MultiArray positions;
    for(int i=0;i<_current_traj.points.size();++i)
    {
      positions.data.clear();
      for(int j=0;j<_joint_names.size();++j)
      {
        for(int k=0;k<_current_traj.joint_names.size();++k)
        {
          if(_current_traj.joint_names[k] == _joint_names[j])
          {
            positions.data.push_back(_current_traj.points[i].positions[k]);
            break;
          }
        }
      }
      double dt = (_current_traj.points[i].time_from_start - time_start).toSec();
      positions.data.push_back(dt);
      time_start = _current_traj.points[i].time_from_start;
      _pub_trajectory_command.publish(positions);
      ros::Duration(max(dt/2,0.03)).sleep();
    }
    publishFlag(1);
    ROS_WARN("end publish trojectory");
    ROS_WARN("trajectory duration: %f",_current_traj.points[_current_traj.points.size() - 1].time_from_start.toSec());
  }

  void jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
  {
    // ROS_INFO("Received a msg");
    _current_joint_state = msg;
    ros::Time now = ros::Time::now();

    if (!_has_active_goal)
      return;
    if (_current_traj.points.empty())
      return;

    // if (!setsEqual(_joint_names, msg->name))
    // {
    //   ROS_ERROR("Joint names from the controller don't match our joint names.");
    //   return;
    // }

    int last = _current_traj.points.size() - 1;
    ros::Time end_time = _active_goal.getGoalID().stamp + (_current_traj.points[last].time_from_start);
    ros::Time fail_time = _active_goal.getGoalID().stamp + ros::Duration(_current_traj.points[last].time_from_start.toSec()*_allowed_execution_duration_scaling+_goal_time_constraint);
    if (now >= end_time)
    {
      // Checks that we have ended inside the goal constraints
      bool inside_goal_constraints = true;
      for (size_t i = 0; i < msg->name.size() && inside_goal_constraints; ++i)
      {
        for(int j = 0;j<_current_traj.joint_names.size();++j)
        {
          if(msg->name[i] == _current_traj.joint_names[j])
          {
            double l1 = fabs(Route(_current_traj.points[last].positions[j], msg->position[i],true).len());
            double l2 = fabs(Route(_current_traj.points[last].positions[j], msg->position[i],false).len());
            if(min(l1,l2) > _goal_constraint)
              inside_goal_constraints = false;
          }
        }
      }

      if (inside_goal_constraints)
      {
        if (now <= fail_time)
        {
          _active_goal.setSucceeded();
          _has_active_goal = false;
          ROS_INFO("action trajectory execution setSucceeded");
        }
      }
      else if (now < fail_time)
      {
        // Still have some time left to make it.
      }
      else
      {
        ROS_WARN("Aborting because we wound up outside the goal constraints");
        ROS_WARN("NOW: %f",now.toSec());
        ROS_WARN("END: %f",end_time.toSec());
        for (size_t i = 0; i < msg->name.size() && inside_goal_constraints; ++i)
        {
          for(int j = 0;j<_current_traj.joint_names.size();++j)
          {
            if(msg->name[i] == _current_traj.joint_names[j])
            {
              double l1 = fabs(Route(_current_traj.points[last].positions[j], msg->position[i],true).len());
              double l2 = fabs(Route(_current_traj.points[last].positions[j], msg->position[i],false).len());
              if(min(l1,l2) > _goal_constraint)
                ROS_WARN("joint: %s outside the goal constraints %f! goal: %f, state:%f",
                msg->name[i].c_str(), min(l1,l2),_current_traj.points[last].positions[j], msg->position[i]);
            }
          }
        }
        _active_goal.setAborted();
        _has_active_goal = false;
      }
    }
  }
};

FollowJointTrajectoryAction* action = NULL;
void shutdown(int sig)
{
  if(action != NULL)
  {
    action->publishFlag(-1);
    delete action;
  }
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"arm_trajectory_service_node");
  ros::NodeHandle node;
  action = new FollowJointTrajectoryAction(node,"/arm_group_controller/follow_joint_trajectory");
  signal(SIGINT,shutdown);
  ros::spin();
  return(0);
}
