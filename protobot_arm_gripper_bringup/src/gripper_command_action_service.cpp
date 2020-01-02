//BLArm Joint Trajectory Action Service
//made by Maple 03/27/2019
//Email 190258039@qq.com

#include <ros/ros.h>
#include <signal.h>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalID.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
inline double f_mod(double a, double n) 
{
    return a - n * floor(a / n);
}

class GripperCommandAction
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
  typedef actionlib::ActionServer<control_msgs::GripperCommandAction> GCAS;
  typedef actionlib::ServerGoalHandle<control_msgs::GripperCommandAction> GoalHandle;
  ros::NodeHandle _node;
  GCAS _action_server;
  GoalHandle _active_goal;
  bool _has_active_goal;
  std::string _action_name;
  ros::Publisher _pub_gripper_command;
  ros::Subscriber _sub_joint_state;
  ros::Timer _watchdog_timer;
  ros::Time _goal_received;
  sensor_msgs::JointStateConstPtr _current_joint_state;
  control_msgs::GripperCommand _current_command;
  
  std::vector<std::string> _joint_names;
  double _goal_constraint;
  double _goal_time_constraint;
  double _allowed_execution_duration_scaling;

public:
  GripperCommandAction(ros::NodeHandle &n,std::string name):
    _node(n),
    _action_name(name),
    _action_server(n,name,
                   boost::bind(&GripperCommandAction::goalCB,this,_1),
                   boost::bind(&GripperCommandAction::cancelCB,this,_1),
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
          if(std::string(j->second)=="hand_group_controller")
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
    _pub_gripper_command = _node.advertise<std_msgs::Float32MultiArray>("drive_command", 10);
    _sub_joint_state = _node.subscribe("/joint_states",1,&GripperCommandAction::jointStateCB,this);
    _watchdog_timer = _node.createTimer(ros::Duration(1.0), &GripperCommandAction::watchdog, this);
    _action_server.start();
  }

  ~GripperCommandAction()
  {
    _pub_gripper_command.shutdown();
    _sub_joint_state.shutdown();
    _watchdog_timer.stop();   
  }

  void cancelCB(GoalHandle gh)
  {
    ROS_INFO("Cancle Goal");
    if (_active_goal == gh)
    {
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
    flag.data.push_back(n);
    flag.data.push_back(n);
    _pub_gripper_command.publish(flag);
    ros::Duration(0.01).sleep();
    _pub_gripper_command.publish(flag);
  }
private:

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
        ROS_WARN("Aborting goal because we have never heard a gripper joint state message.");
      }
      else if ((now - _current_joint_state->header.stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the gripper joint in %.3lf seconds",
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
    ROS_INFO("Received a Goal");
    if (_has_active_goal)
    {
      publishFlag(-1);
      // Marks the current goal as canceled.
      ROS_INFO("has a GripperCommand Goal");
      _active_goal.setCanceled();
      ROS_INFO("setCanceled a GripperCommand");
      _has_active_goal = false;
    }

    gh.setAccepted();
    _active_goal = gh;
    GCAS::Goal  goal = *_active_goal.getGoal();
    _current_command = goal.command;
    _goal_received = ros::Time::now();
    _has_active_goal = true;
    publishFlag(-1);
    std_msgs::Float32MultiArray position;
    position.data.clear();
    position.data.push_back(_current_command.position);
    position.data.push_back(_current_command.max_effort);
    _pub_gripper_command.publish(position);
    publishFlag(1);
  }

  void jointStateCB(const sensor_msgs::JointStateConstPtr &msg)
  {
    // ROS_INFO("Received a msg");
    _current_joint_state = msg;
    ros::Time now = ros::Time::now();

    if (!_has_active_goal)
      return;
    bool inside_goal_constraints = true;
    double p = 0;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if(msg->name[i] == _joint_names[0])
      {
        p = msg->position[i];
        double l1 = fabs(Route(_current_command.position, msg->position[i],true).len());
        double l2 = fabs(Route(_current_command.position, msg->position[i],false).len());
        if(min(l1,l2) > _goal_constraint)
          inside_goal_constraints = false;
      }
    }
    control_msgs::GripperCommandFeedback feedback;
    feedback.position = p;
    feedback.effort = 0;
    feedback.reached_goal = false;
    feedback.stalled = false;

    control_msgs::GripperCommandResult result;
    result.position = p;
    result.effort =0;
    result.reached_goal = false;
    result.stalled = false;

    if(!inside_goal_constraints)
    {
      if (now - _goal_received < ros::Duration(5.0))
      {
        return;
      }
      else
      {
        ROS_ERROR("controller fail, Cancelling goal!");
        _active_goal.setAborted(result);
        _has_active_goal = false;
      }
    }
    else
    {
      feedback.reached_goal = true;
      result.reached_goal = true;
      _active_goal.setSucceeded(result);
      ROS_INFO("gripper action execution setSucceeded");
      _has_active_goal = false;
    }
    _active_goal.publishFeedback(feedback);
  }
};

GripperCommandAction* action = NULL;
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
  ros::init(argc,argv,"gripper_command_service_node");
  ros::NodeHandle node;
  action = new GripperCommandAction(node,"/hand_group_controller/gripper_cmd");
  signal(SIGINT,shutdown);
  ros::spin();
  return(0);
}


