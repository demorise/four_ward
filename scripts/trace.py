#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import moveit_commander
import geometry_msgs.msg
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import roslib
#import




def talker():
    print('Publishing marker')
    rospy.init_node('talker',anonymous=True)
    marker = Marker()
    #marker.header.frame_id = "panda_link0"
    marker.type = marker.SPHERE
    #marker.type = marker.MESH_RESOURCE;#
    #path = roslib.packages.get_pkg_dir("moveit_tutorials") + "/table.STL"
    #print(path)
    #marker.mesh_resource = 'package://' + 'moveit_tutorials' + '/' + 'table.stl'
    marker.action = marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.a = 1.0
    marker.color.r = 0.750
    marker.color.g = 0.20
    marker.color.b = 0.20
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    #marker.id = 1
    i=1

    markerArray = MarkerArray()
    #markerArray.markers.append(marker)

    topic = 'visualization_marker_array'
    pub = rospy.Publisher(topic, MarkerArray, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        marker.id = i
        i=i+1
        #print(i)
        marker.header.frame_id = "panda_leftfinger"
        markerArray.markers.append(marker)
        pub.publish(markerArray)
        #scene_pub.publish(PlanningScene)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
