#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Joy
import roslib
from urdf_parser_py.urdf import URDF
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState
import numpy as np
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('jogger',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

scene = moveit_commander.PlanningSceneInterface()
box_name = 'box'
vf_active = input('Enter 1 to enable Vf or 0 to disable VF: ')


def add_box():
  box_pose = geometry_msgs.msg.PoseStamped()
  box_pose.header.frame_id = "panda_link0"
  box_pose.pose.orientation.w = 1.0
  box_pose.pose.orientation.x = 0.0
  box_pose.pose.orientation.z = 0.0
  box_pose.pose.orientation.y = 0.0
  box_pose.pose.position.x = 0.65#1.25
  box_pose.pose.position.y = 0
  box_pose.pose.position.z = 0#-0.08
  box_name = "box"
  print "============ Press `Enter` to add a box to the planning scene ..."
  raw_input()
  scene.add_box(box_name, box_pose, size=(0.4, 0.4, 0.4))

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


def go_to_joint_state(joint_pose, group):
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = joint_pose[0]
  joint_goal[1] = joint_pose[1]
  joint_goal[2] = joint_pose[2]
  joint_goal[3] = joint_pose[3]
  joint_goal[4] = joint_pose[4]
  joint_goal[5] = joint_pose[5]
  joint_goal[6] = joint_pose[6]
  group.go(joint_goal, wait=True)
  group.stop()
  current_joints = group.get_current_joint_values()
  return all_close(joint_goal, current_joints, 0.01)


def go_to_pose_goal(pose_in, group):
  group.set_pose_target(pose_in)
  plan = group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  current_pose=group.get_current_pose().pose
  return all_close(pose_in, current_pose, 0.01)


def callback(data):
  curr_cart_pose = group.get_current_pose().pose
  pose_goal = group.get_current_pose().pose
  curr_joint_state = group.get_current_joint_values()
  joint_goal = group.get_current_joint_values()
  pose_increment = 0.075
  joint_increment = 0.1
  F = np.array([0,0])

  #print('vf_active',vf_active)
  #Jog robot End Effector with VF enabled
  if data.buttons[6]==0 and data.buttons[7]==0 and vf_active==1:
    #jog in X axis
    if data.axes[6]==1:
      F[0]=-1   	
    if data.axes[6]==-1:
      F[0]=1 
    #jog in Y axis
    if data.axes[7]==-1:
      F[1]=-1 
    if data.axes[7]==1:
      F[1]=1     
    #jog in Z axis
    if data.buttons[0]==1:
      pass 
    if data.buttons[4]==1:
      pass 

    p_e_pos = group.get_current_pose().pose.position
    p_e = np.c_[p_e_pos.x, p_e_pos.y]
    F_desired = vf_force_filter(F, p_e)
    pose_increment_vec = np.multiply(pose_increment, F_desired)
    #print(F_desired) 
    if np.linalg.norm(F_desired)==0:
    	print("MOTION PROHIBITED BY VF")
    else:
    	print("MOTION PERMITTED BY VF")
    pose_goal.position.x = curr_cart_pose.position.x + pose_increment_vec[0,0]
    pose_goal.position.y = curr_cart_pose.position.y + pose_increment_vec[0,1]
    pose_goal.position.z = curr_cart_pose.position.z
    go_to_pose_goal(pose_goal, group)


  #Jog robot End Effector with VF disabled
  if data.buttons[6]==0 and data.buttons[7]==0 and vf_active==0:
    #jog in X axis
    if data.axes[6]==1:       
      pose_goal.position.x = curr_cart_pose.position.x - pose_increment
    if data.axes[6]==-1:
      pose_goal.position.x = curr_cart_pose.position.x + pose_increment 
    #jog in Y axis
    if data.axes[7]==-1:
      pose_goal.position.y = curr_cart_pose.position.y - pose_increment 
    if data.axes[7]==1:
      pose_goal.position.y = curr_cart_pose.position.y + pose_increment       
    #jog in Z axis
    if data.buttons[0]==1:
      pose_goal.position.z = curr_cart_pose.position.z - pose_increment 
    if data.buttons[4]==1:
      pose_goal.position.z = curr_cart_pose.position.z + pose_increment 
    go_to_pose_goal(pose_goal, group)


  #Jog robot joints counterclockwise
  if data.buttons[6]==1:
    if data.buttons[0]==1:
      joint_goal[0]=curr_joint_state[0]-joint_increment
    if data.buttons[1]==1:
      joint_goal[1]=curr_joint_state[1]-joint_increment
    if data.buttons[3]==1:
      joint_goal[2]=curr_joint_state[2]-joint_increment
    if data.buttons[4]==1:
      joint_goal[3]=curr_joint_state[3]-joint_increment
    if data.axes[6]==1:
      joint_goal[4]=curr_joint_state[4]-joint_increment
    if data.axes[6]==-1:
      joint_goal[5]=curr_joint_state[5]-joint_increment
    if data.axes[7]==1 or data.axes[7]==-1:
      joint_goal[6]=curr_joint_state[6]-joint_increment
    go_to_joint_state(joint_goal, group)
  
  #Jog robot joints clockwise
  if data.buttons[7]==1:
    if data.buttons[0]==1:
      joint_goal[0]=curr_joint_state[0]+joint_increment
    if data.buttons[1]==1:
      joint_goal[1]=curr_joint_state[1]+joint_increment
    if data.buttons[3]==1:
      joint_goal[2]=curr_joint_state[2]+joint_increment
    if data.buttons[4]==1:
      joint_goal[3]=curr_joint_state[3]+joint_increment
    if data.axes[6]==1:
      joint_goal[4]=curr_joint_state[4]+joint_increment
    if data.axes[6]==-1:
      joint_goal[5]=curr_joint_state[5]+joint_increment
    if data.axes[7]==1 or data.axes[7]==-1:
      joint_goal[6]=curr_joint_state[6]+joint_increment
    go_to_joint_state(joint_goal, group)
  #print(group.get_current_joint_values())

#Parametric curve (straight line in x-dir)
def r(s):
    thetad = 0
    theta = thetad * (np.pi/180)
    x1 = 0
    y1 = 0
    x = x1 + (s*np.cos(theta))
    y = y1 + (s*np.sin(theta))
    return np.c_[x, y]


def closest_point(p_e):
    line_vec = r(1)
    d_line = np.dot(p_e, np.transpose(line_vec))
    pt_closest = np.multiply(d_line, line_vec)
    tangent = line_vec
    return [pt_closest, tangent]


def vf_force_filter(F, p_e):
	k_d = 0 #stiffness factor
	r_s,t = closest_point(p_e)
	d = np.subtract(r_s, p_e)
	U = np.add(t, np.multiply(k_d,d))
	u = np.divide(U,np.linalg.norm(U))
	F_D = np.dot(u, F)
	F_D_vec = np.multiply(F_D, u)
	return F_D_vec
    
def main():
  #print(group.get_current_joint_values())
  #go_to_joint_state([0, 0, 0, 0, 0, 0, 0], group)
  print ("============ Press ENTER to move robot to start position ==============")
  raw_input()
  #go_to_joint_state([0.5413,   -1.1310,    0.3037,   -2.7722,    0.1115,    1.8626,    0.3403], group)
  go_to_joint_state([-0.0073795209295667045, -0.2501263558747043, 0.007659234098432531, -2.7643094205475003, 0.0016373376404264525, 2.515489379362175, 0.7839116051429639], group)
  #start_pose = group.get_current_pose().pose
  #start_pose.position.x = 0.45
  #start_pose.position.y = 0
  #start_pose.position.z = 0.2
  #go_to_pose_goal(start_pose, group)
  #go_to_joint_state([6.28, 0., -6.28, -1.57, 6.28, 1.57, 0], group)

  #go_to_joint_state([0.0000,    1.4404,   -0.0000,    0.8355,   -0.0000,    0.6041,   -0.0000], group)
  #go_to_joint_state([0.7293,    1.6846,   -0.1203,    0.4525,   -0.0595,    1.4252,    0.3700], group)
  #print('press ENTER to remove box')
  #raw_input()
  #scene.remove_world_object(box_name)
  #add_box()
  print("You can now jog the robot")
  rospy.Subscriber("joy", Joy, callback)
  rospy.spin()
if __name__ == '__main__':
  main()

