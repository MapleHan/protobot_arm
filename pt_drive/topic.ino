#define DATA_FLAG   0
#define START_FLAG  -1
#define END_FLAG    1
void addWaypoint(const std_msgs::Float32MultiArray& point_msg);
ros::NodeHandle  nh;
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("/joint_states", &joint_state_msg);
ros::Subscriber<std_msgs::Float32MultiArray> waypoint_sub("drive_command",addWaypoint);
ros::Subscriber<std_msgs::Bool> cali_sub("calibrated",calibration );

void topicInit()
{
  const int joint_number = kArmJointNumber+kGripperJointNumber;
  static char *joint_names[joint_number];
  static float joint_positions[joint_number] = {};
  nh.initNode();
  nh.advertise(joint_state_pub);
  nh.subscribe(waypoint_sub);
  nh.subscribe(cali_sub);

  joint_state_msg.position_length = joint_number;
  joint_state_msg.position = joint_positions;
  joint_state_msg.name_length = joint_number;
  for(uint8_t i=0;i<kArmJointNumber;++i)
    joint_names[i] = arm.getJoint(i).name;
  joint_names[joint_number-1] = (char*)kGripperJointName[0];
  joint_state_msg.name = joint_names;
}


void pubJointState()
{
   joint_state_msg.header.stamp = nh.now();
   for(uint8_t i=0;i<arm.kJointNumber;++i)
   {
      joint_state_msg.position[i] = arm.getJoint(i).getPosition();
   }
   joint_state_msg.position[arm.kJointNumber] = gri.getJoint(0).getPosition();
   joint_state_pub.publish( &joint_state_msg );
}

void topicSpin()
{
  nh.spinOnce();
}


void addWaypoint(const std_msgs::Float32MultiArray& point_msg)
{
//  SerialUSB.println("addWaypoint FUNCTION");
//  SerialUSB.println("point_msg.data_length = "+String(point_msg.data_length));
  static int book_n = 0;
  int msg_flag = flag(point_msg);
  if(msg_flag==START_FLAG)
  {
    SerialUSB.println("restart receive trajectory");
    book_n = 0;
    if(point_msg.data_length==kGripperJointNumber+1)
    {
      gri.getJoint(0).pause();
    }
    else if(point_msg.data_length==kArmJointNumber+1)
    {
      if(!current_tra.isNull())
      {
        current_tra.clear();
      }
    }
  }
  else if(msg_flag==END_FLAG)
  {
    SerialUSB.println("received a trajectory, waypoint count: "+String(book_n));
    book_n = 0;
  }
  else
  {
    if(point_msg.data_length==kArmJointNumber+1)
    {
      Waypoint waypoint;
      uint8_t i=0;
      for(i=0;i<point_msg.data_length-1;++i)                  //角度控制信息
        waypoint.pos.addValue(i,point_msg.data[i]);
      waypoint.time = point_msg.data[i];
//      char output[100];
//      sprintf(output,"addWaypoint: %.6f %.6f %.6f %.6f",point_msg.data[0],point_msg.data[1],point_msg.data[2],point_msg.data[3]);
//      SerialUSB.println(output);
      current_tra.addWaypoint(waypoint);
//      sprintf(output,"size:%d",current_tra.getSize());
//      SerialUSB.println(output);
      book_n++;
    }
    else if(point_msg.data_length==kGripperJointNumber+1)
    {
      gri.getJoint(0).rotateTo(point_msg.data[0]);
    }
  }
  nh.spinOnce();
}



void calibration(const std_msgs::Bool& key)
{
  if(key.data==true)
  {
    arm.calibrationFinish();
    digitalWrite(LED_BUILTIN,HIGH);
    SerialUSB.println("calibrated");
  }
}


int flag(const std_msgs::Float32MultiArray& point_msg)
{
  int f = START_FLAG;
  for(uint8_t i=0;i<point_msg.data_length;++i)
  {
    if(point_msg.data[i]!=START_FLAG)
    {
      f = DATA_FLAG;
      break;
    }
  }
  if(f == START_FLAG)
    return(START_FLAG);
  else
  {
    f = END_FLAG;
    for(uint8_t i=0;i<point_msg.data_length;++i)
    {
      if(point_msg.data[i]!=END_FLAG)
      {
        f = DATA_FLAG;
        break;
      }
    }
    if(f == END_FLAG)
      return(END_FLAG);
  }
  return(DATA_FLAG);
}
