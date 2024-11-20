#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from math import inf, sqrt
from heapq import heappush, heappop
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import forklift_server.msg
import tf
from visualization_msgs.msg import Marker

# Create a topology map for navigation planning
class TopologyMap():
    def __init__(self):
        self.graph = self.get_ajacency_list()
        self.start = ""
        self.goal = ""

    def get_ajacency_list(self):    # Convert directed to undirected graph
        adjacency = {}
        for vertex in GRAPH.keys():
            # 確保每個節點在無向圖中都有一個紀錄
            if vertex not in adjacency:
                adjacency[vertex] = dict()

            for neighbor, weight in GRAPH[vertex].items():
                adjacency[vertex].update({neighbor: weight})

                # 確保每個鄰邊在無向圖中都有一個紀錄
                if neighbor not in adjacency:
                    adjacency[neighbor] = dict()
                
                adjacency[neighbor].update({vertex: weight})

        return adjacency

    # Find the shortest path using dijkstra.
    # Return a tuple - (a int that means cost, a LIST that is shortest(minium cost) path).
    def dijkstra(self):     
        # 檢查起始點和終點是否在圖中
        if self.start not in self.graph or self.goal not in self.graph:
            raise ValueError(f"起始點 '{self.start}' 或終點 '{self.goal}' 不在圖中")
        
        # 儲存最短路徑的字典
        shortest_paths = {vertex: inf for vertex in self.graph}
        shortest_paths[self.start] = 0

        # 儲存優先隊列
        priority_queue = [(0, self.start)]  # (距離, 頂點)
        
        # 儲存每個頂點的父節點
        previous_vertices = {vertex: None for vertex in self.graph}

        while priority_queue:
            current_distance, current_vertex = heappop(priority_queue)

            # 如果當前距離大於已知最短距離，則跳過
            if current_distance > shortest_paths[current_vertex]:
                continue

            # 遍歷相鄰的頂點
            for neighbor, weight in self.graph[current_vertex].items():
                distance = current_distance + weight

                # 只有當新的距離更短時，才更新最短路徑
                if distance < shortest_paths[neighbor]:
                    shortest_paths[neighbor] = distance
                    previous_vertices[neighbor] = current_vertex  # 記錄父節點
                    heappush(priority_queue, (distance, neighbor))

        # 構建從起始點到終點的路徑
        path = []
        current_vertex = self.goal
        while current_vertex is not None:
            path.append(current_vertex)
            current_vertex = previous_vertices[current_vertex]
        
        path.reverse()  # 反轉路徑，因為我們是從終點回溯到起點
        return shortest_paths[self.goal], path  # 返回最短距離和路徑

# Implementation of single-point navigation
class Navigation():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def get_robot_pose(self):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
            # return trans[0], trans[1], rot[2], rot[3]
            return float(trans[0]), float(trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(e)
            return 0.0, 0.0, 0.0, 0.0

    def move(self, goal):
        # Get location from goal name
        loc = WAYPOINT[goal]
        x = loc[0]; y = loc[1]; zr = loc[2]; w = loc[3];

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position.x = x
        move_base_goal.target_pose.pose.position.y = y
        move_base_goal.target_pose.pose.orientation.z = zr
        move_base_goal.target_pose.pose.orientation.w = w

        self.client.send_goal(move_base_goal)
        self.position = move_base_goal.target_pose.pose.position
        self.orientation = move_base_goal.target_pose.pose.orientation
        rospy.loginfo('Navigate to %s' % goal)
        rospy.loginfo('- Position: x: %f\ty: %f\tz: %f' % (self.position.x, self.position.y, self.position.z))
        rospy.loginfo('- Orientation: x: %f\ty: %f\tz: %f' % (self.orientation.x, self.orientation.y, self.orientation.z))

        # Pass Point
        while True:
            pose_x, pose_y = self.get_robot_pose()
            dx = abs(pose_x - float(x)); dy = abs(pose_y - float(y))
            current_distance = sqrt(pow(dx, 2) + pow(dy, 2))
            # rospy.loginfo('Current Dis: %f' % current_distance)
            if current_distance < PASS_DISTANCE:
                rospy.loginfo('Navigation: done!')
                return self.client.get_result
        
        # Not Pass Point
        # wait = self.client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return self.client.get_result()

# Implementation of multiple-point navigation
class TopologyMapAction():
    # Create messages that are used to publish feedback/result
    _result = forklift_server.msg.TopologyMapResult()
    _feedback = forklift_server.msg.TopologyMapFeedback()

    def __init__(self, name):
        # Initial simple action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, forklift_server.msg.TopologyMapAction, execute_cb=self.execute_callback, auto_start = False)
        self._as.start()
        
        # For path planning
        self.topology = TopologyMap()
        self.topology.start = START
        self.navigation = Navigation()

    def execute_callback(self, msg):
        rospy.loginfo('TopologyMap receive command: %s' % (msg.goal))

        # Path planning
        self.topology.goal = msg.goal
        cost, paths = self.topology.dijkstra()
        for i in paths:
            if self.navigation.move(i):
                rospy.loginfo('TopologyMapAction: %s execution done!' % i)
        self.topology.start = msg.goal    # Last node is start node for next navigation request.

        # TopologyMapAction Done
        self._as.publish_feedback(self._feedback)
        rospy.logwarn('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

# Visualization of waypoints
class Viewer():
    def __init__(self):
        rospy.loginfo("Start Mark Visualization.")
        self.pub = rospy.Publisher("/GoalMarker", Marker, queue_size=100)

    def publish(self, goal):
        marker = Marker()
        marker.action = Marker.ADD
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration()
        marker.ns = goal
        marker.id = 0
        marker.type = Marker.ARROW

        loc = WAYPOINT[goal]
        marker.pose.position.x = loc[0]
        marker.pose.position.y = loc[1]
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = loc[2]
        marker.pose.orientation.w = loc[3]

        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.1

        marker.color.r = 255 / 255
        marker.color.g = 255 / 255
        marker.color.b = 255 / 255
        marker.color.a = 1.0
        rospy.sleep(DELAY)    # Don't delete!
        self.pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('TopologyMapServer')
    rate = rospy.Rate(10)

    # Initial parameters
    ODOMTOPIC = rospy.get_param(rospy.get_name() + "/odom", "/odom")
    DELAY = rospy.get_param(rospy.get_name() + "/delay")
    PASS_DISTANCE = rospy.get_param(rospy.get_name() + "/pass_distance", "1.1")
    SPIN = rospy.get_param(rospy.get_name() + "/spin") # True/False
    START = rospy.get_param(rospy.get_name() + "/start_node", "START")
    GRAPH = rospy.get_param(rospy.get_name() + "/graph")
    WAYPOINT = rospy.get_param(rospy.get_name() + "/waypoints")

    # Visualization
    viewer = Viewer()
    for name in WAYPOINT.keys():
        viewer.publish(name)
        rospy.loginfo("PublishMarker: %s", name)

    # Create a server to receive navigation requests
    rospy.logwarn('TopologyMapServer start')
    server = TopologyMapAction(rospy.get_name())
    
    rospy.spin()