#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import forklift_server.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
from visualization_msgs.msg import Marker
import tf2_ros
import heapq
import math
from threading import Thread
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy
import time


class Viewer():
    def __init__(self):
        rospy.loginfo("Start Mark Visualization.")
        self.pub = rospy.Publisher("/GoalMarker", Marker, queue_size=100)

    def publish(self, goal, pose):
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration()
        marker.ns = goal
        marker.id = 0
        marker.type = Marker.ARROW

        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = pose[2]
        marker.pose.orientation.w = pose[3]

        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.1

        marker.color.r = 255 / 255
        marker.color.g = 255 / 255
        marker.color.b = 255 / 255
        marker.color.a = 1.0
        rospy.sleep(0.4)    #
        self.pub.publish(marker)


def markpoint(waypoints):
    viewer = Viewer()
    for i in waypoints:
        viewer.publish(i, waypoints[i])


if __name__ == '__main__':
    rospy.init_node('TopologyMap_server')
    
    # Get All Parameters
    graph = rospy.get_param(rospy.get_name() + "/graph")
    waypoints = rospy.get_param(rospy.get_name() + "/waypoints")

    # Mark Visualization
    t = Thread(target=markpoint, args=(waypoints,))
    t.start()

    # Server
    rospy.spin()