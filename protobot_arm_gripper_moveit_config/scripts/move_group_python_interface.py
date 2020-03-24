#!/usr/bin/env python
# -- coding: utf-8 --
#Maple 20191018

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from control_msgs.msg import GripperCommand

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


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm_group")
    hand_group = moveit_commander.MoveGroupCommander("hand_group")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    planning_frame = arm_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = arm_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
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
    self.hand_group.go([0,0],wait=True)
    #数组长度为hand关节个数

  def go_to_joint_state(self,joint_goal):
    move_group = self.arm_group
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_position_goal(self,x,y,z):
    move_group = self.arm_group
    xyz= [x,y,z]
    move_group.set_position_target(xyz)
    plan = move_group.go(wait=True)
    print plan
    move_group.stop()
    current_pos = move_group.get_current_pose().pose
    current_position = pose_to_list(current_pos)
    return all_close(xyz, current_position, 0.005)


  def plan_cartesian_path(self, scale=1):
    waypoints = []
    move_group = self.arm_group
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    print plan, fraction
    return plan, fraction


  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    move_group = self.arm_group
    move_group.execute(plan, wait=True)

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    box_name = self.box_name
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False


  def add_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.eef_link
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = -0.1 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.03, 0.03, 0.03))
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names
    grasping_group = 'hand_group'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link
    scene.remove_attached_object(eef_link, name=box_name)
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    box_name = self.box_name
    scene = self.scene
    scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute reset ..."
    raw_input()
    tutorial.reset()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    tutorial.go_to_joint_state([0,0.2,0.2,0,0,0.2])

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.go_to_position_goal(0.2,0.2,0.6)

    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    raw_input()
    tutorial.display_trajectory(cartesian_plan)

    print "============ Press `Enter` to execute a saved path ..."
    raw_input()
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to add a box to the planning scene ..."
    raw_input()
    print tutorial.add_box()

    print "============ Press `Enter` to attach a Box to the Panda robot ..."
    raw_input()
    tutorial.attach_box()
    tutorial.hand_group.go([-0.3,-0.3],wait=True)
    #数组长度为hand关节个数

    print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    raw_input()
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    print "============ Press `Enter` to detach the box from the Panda robot ..."
    raw_input()
    tutorial.detach_box()
    tutorial.hand_group.go([0,0],wait=True)
    #数组长度为hand关节个数

    print "============ Press `Enter` to remove the box from the planning scene ..."
    raw_input()
    tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()


