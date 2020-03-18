#!/usr/bin/env python
# -- coding: utf-8 --
#Maple 20191018

import sys
import copy
import rospy
import tf
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand
from find_object_2d.msg import ObjectsStamped
distance_flan_gri = 0.21045+0.005
#长度方向：左、右
#宽度方向：前、后
#高度方向：上、下
objects_id=[[99,99,99,99,1,99],\
           [99,99,99,99,2,99],\
           [32,33,34,35,36,37]\
          ]
#长、宽、高
objects_size = [(0.05,0.05,0.1),\
                (0.05,0.05,0.1),\
                (0.05,0.05,0.1)\
                ]
objects_name = ["beer","beer","beer"]
tf_listener =None
interface = None




def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    hand_group = moveit_commander.MoveGroupCommander("hand_group")
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    planning_frame = arm_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame
    eef_link = arm_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.robot = robot
    self.scene = scene
    self.arm_group = arm_group
    self.hand_group = hand_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def reset(self):
    self.arm_group.set_named_target('home')
    self.arm_group.go(wait=True)
    self.hand_group.go([0,0,0],wait=True)

  def go_to_joint_state(self,joint_goal):
    move_group = self.arm_group
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def hand_go_to_joint_state(self,joint_goal):
    move_group = self.hand_group
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.1)
 
  def go_to_position_goal(self,x,y,z):
    move_group = self.arm_group
    xyz= [x,y,z]
    move_group.set_position_target(xyz)
    plan = move_group.go(wait=True)
    print "plan:",plan
    move_group.stop()
    current_pos = move_group.get_current_pose().pose
    current_position = pose_to_list(current_pos)
    return all_close(xyz, current_position, 0.1)

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in self.scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False

#box_pose.pose.orientation.w = 1.0
#box_pose.pose.position.y = -0.2 # slightly above the end effector
  def add_box(self, box_name, box_pose, box_size, timeout=5):
    if(type(box_pose)!=geometry_msgs.msg.PoseStamped):
      return False
    if(type(box_size)!=tuple):
      return False
    self.scene.add_box(box_name, box_pose, size=box_size)
    return self.wait_for_state_update(box_name,box_is_known=True, timeout=timeout)


  def attach_box(self, box_name, timeout=5):
    grasping_group = 'hand_group'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.eef_link, box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_name,box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, box_name,timeout=5):
    self.scene.remove_attached_object(self.eef_link, name=box_name)
    return self.wait_for_state_update(box_name,box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, box_name, timeout=5):
    self.scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_name,box_is_attached=False, box_is_known=False, timeout=timeout)


def object_callback(data):
  if tf_listener == None:
    return
  b = False
  for k in range(len(data.objects.data)/12):
    for i in range(len(objects_id)):
      for j in range(len(objects_id[0])):
        if data.objects.data[k*12] == objects_id[i][j]:
          b =True
          try:
            base_frame = rospy.get_namespace().strip('/')+'object_'+str(int(data.objects.data[k*12]))  # frameid
            (trans, rot) = tf_listener.lookupTransform('world', base_frame, rospy.Time(0)) # TODO: frame id !!!
            x, y, z = trans
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
          except tf.LookupException:
            print tf.LookupException
            break
          except tf.ConnectivityException:
            print tf.ConnectivityException
            break
          except tf.ExtrapolationException:
            print tf.ExtrapolationException
            break
          print base_frame,x, y, z,roll, pitch, yaw
          if interface !=None: 
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z-objects_size[k][j/2]/2
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            pose.header.frame_id = "world"
            interface.add_box(box_name=objects_name[k], box_pose=pose, box_size=objects_size[k])
            time.sleep(3)
            dh = objects_size[k][j/2]/2
            print "hand open:",interface.hand_go_to_joint_state([-0.26,-0.26,-0.26])
            print x,y,z+distance_flan_gri-dh
            if interface.go_to_position_goal(x,y,z+distance_flan_gri+0.05):
              time.sleep(2)
              if interface.go_to_position_goal(x,y,z+distance_flan_gri-dh):
                interface.attach_box(box_name=objects_name[k])
                for i in range(3):
                  time.sleep(1)
                  print i
                print "hand colose:"
                time.sleep(2)
                print ":",interface.hand_go_to_joint_state([-0.1,-0.1,-0.1])
                time.sleep(2)
                print "up:",interface.go_to_position_goal(x,y,z+distance_flan_gri-dh+0.05)
                print "left:",interface.go_to_position_goal(x,y+0.1,z+distance_flan_gri-dh+0.05)
                print "hand release:",interface.hand_go_to_joint_state([-0.26,-0.26,-0.26])
                interface.detach_box(box_name=objects_name[k])
                time.sleep(3)
            interface.detach_box(box_name=objects_name[k])
            interface.remove_box(box_name=objects_name[k])
            print "reset\n"
            interface.reset()
          break
      if b == True:
        b = False
        break


def main():
  try:
    rospy.init_node('move_group_python_interface', anonymous=True)
    global interface,tf_listener
    interface = MoveGroupPythonInteface()
    interface.reset()
    tf_listener = tf.TransformListener()
    object_sub = rospy.Subscriber("objectsStamped",ObjectsStamped,object_callback,queue_size=1)
    #interface.go_to_joint_state([0,0.78,-0.78,0,0,0.78])
    #interface.go_to_position_goal(0.4,0.2,0.6)
    #print interface.add_box()
    #interface.attach_box()
    #interface.detach_box()
    #interface.remove_box()
  except:
    return
  rate= rospy.Rate(10)
  while not rospy.is_shutdown():
    try:
      #interface.hand_go_to_joint_state([-0.26,-0.26,-0.26])
      #interface.go_to_position_goal(0.4,0,0.3)
      #print "hand colose:",interface.hand_go_to_joint_state([0.1,0.1,0.1])
      #time.sleep(2)
      #print "up:",interface.go_to_position_goal(0.4,0,0.35)
      #print "left:",interface.go_to_position_goal(0.4,0.1,0.35)
      #print "hand detach:",interface.hand_go_to_joint_state([-0.26,-0.26,-0.26])
      #interface.reset()
      rate.sleep()
    except:
      return

if __name__ == '__main__':
  main()



