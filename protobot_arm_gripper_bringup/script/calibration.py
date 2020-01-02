#!/usr/bin/env python
# -- coding: utf-8 --
#Maple 20190114  

import rospy,time,math
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class Calibration:
  def __init__(self, name):
    self.name = name
    rospy.init_node(self.name)
    #初始化化ros note，设置节点名称
    self.joint_names = ["rot_joint","uarm_joint","larm_joint","flange_joint","finger1_joint"]
    self.joint_position = [0,0,0,0,0]
    self.sub = rospy.Subscriber("/joint_states", JointState, self.callback,queue_size=1)
    self.calibration_serial = rospy.Publisher('drive_command',Float32MultiArray,queue_size=100)

  def callback(self, state):
    #rospy.loginfo(self.name+" received joint_states")
    #rospy.loginfo(state)
    
    change = False
    for index in range(len(state.name)):
      for jn in range(len(self.joint_names)):
        if state.name[index] == self.joint_names[jn]:
          if state.position[index] != self.joint_position[jn]:
            change = True
            self.joint_position[jn] = state.position[index]
    if change==False:
      return
    out = Float32MultiArray()
    out.data = [self.joint_position[0],self.joint_position[1],self.joint_position[2],self.joint_position[3],0]
    start_flag = -1
    start = Float32MultiArray()
    start.data = [start_flag,start_flag,start_flag,start_flag,start_flag]
    self.calibration_serial.publish(start)
    self.calibration_serial.publish(out)
    finish_flag = 1
    finish = Float32MultiArray()
    finish.data = [finish_flag,finish_flag,finish_flag,finish_flag,finish_flag]
    self.calibration_serial.publish(finish)
    out = Float32MultiArray()
    out.data = [self.joint_position[4],0]
    self.calibration_serial.publish(out)

if __name__=='__main__':
    try:
        print("start calibration...")
        cali = Calibration('calibration')
        rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(self.name+'Failed to start calibration...')

