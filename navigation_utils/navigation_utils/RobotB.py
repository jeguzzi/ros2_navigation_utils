#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import action_msgs.msg
from nav_msgs.msg import Path
import navigation_msgs.action

from .gridmap import OccupancyGridMap
import matplotlib.pyplot as plt
from .a_star import a_star, plot_path
from .GMAP import GMAP
import cv2
import matplotlib.pyplot as plt
import numpy as np
import copy
from typing import Any, Optional
import time
import subprocess

class RobotB(Node):

    def __init__(self):
        super().__init__('RobotB')

        self.declare_parameter('mapPath', 'meow')
        mapPath = self.get_parameter('mapPath').value
        self.get_logger().info("mapPath: %s" % (str(mapPath),))

        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value
        self.get_logger().info("frame_id: %s" % (str(self.frame_id),))

        self.declare_parameter('robotA', 'RM0001')
        self.robotA = self.get_parameter('robotA').value
        self.get_logger().info("robotA: %s" % (str(self.robotA ),))

        self.declare_parameter('robotB', 'RM0002')
        self.robotB = self.get_parameter('robotB').value
        self.get_logger().info("robotB: %s" % (str(self.robotB),))

        self.end_pub = self.create_publisher(String, 'end', 10)
        self.scenario_subB = self.create_subscription(PoseArray, self.robotB + '/scenario', self.do_astar, 1)
        self.poseA_sub = self.create_subscription(PoseStamped, self.robotA + '/GT', self.do_call, 1)

        self.way_pointsB = []
        self.midpoint = None


        self.gmap = GMAP(mapPath)
        bin_map = copy.deepcopy(self.gmap.map) #cv2.cvtColor(self.gmap.map, cv2.COLOR_BGR2GRAY)
        bin_map[bin_map == (255-205)] = 255
        kernel = np.ones((21,21),np.uint8)
        bin_map = cv2.dilate(bin_map,kernel,iterations = 1)
        cv2.imwrite("planning_map.png", bin_map)
        bin_map = bin_map.astype(np.float32) / 255
        bin_map[bin_map < 1.0] = 0.0
        self.occ_map = OccupancyGridMap(bin_map, 1.0)

        self.gmap.map[self.gmap.map == (255-205)] = 255
        kernel = np.ones((21,21),np.uint8)
        self.gmap.map = cv2.dilate(self.gmap.map, kernel, iterations = 1)
        cv2.imwrite("scenario_map.png", self.gmap.map)

        # Action client*   
        self.follow_clientB = ActionClient(self, navigation_msgs.action.FollowPath, self.robotB + "/follow_path")
        
        self.goal_msgB = navigation_msgs.action.FollowPath.Goal()
        self.goal_msgB.speed = 0.5
        self.goal_msgB.angular_speed = 1.0
        self.goal_msgB.angular_goal_tolerance = 0.1
        self.goal_msgB.spatial_goal_tolerance = 0.1
        self.goal_msgB.path.header.frame_id = self.frame_id
        self.goal_msgB.turn_ahead = True

        self.publish = False

        #self.start_sub = self.create_subscription(String, 'start', self.do_call, 1)



    #async def do_call(self, start_msg):
    def do_astar(self, scenario_msg):

        time.sleep(1)

        yawS = np.arctan2(scenario_msg.poses[0].orientation.z, scenario_msg.poses[0].orientation.w)
        startB = [scenario_msg.poses[0].position.x, scenario_msg.poses[0].position.y, yawS]
        yawE = np.arctan2(scenario_msg.poses[1].orientation.z, scenario_msg.poses[1].orientation.w)
        endB = [scenario_msg.poses[1].position.x, scenario_msg.poses[1].position.y, yawE]


        self.midpoint = startB

        self.get_logger().info("Robot B: start point ({} {} {})".format(startB[0], startB[1], startB[2]))
        self.get_logger().info("Robot B: end point ({} {} {})".format(endB[0], endB[1], endB[2]))

        self.get_logger().info("Starting A* for robot B")
        start_nodeB = self.gmap.world2map([startB[0], startB[1]])
        goal_nodeB = self.gmap.world2map([endB[0], endB[1]])
        occ_map = copy.deepcopy(self.occ_map) 
        pathB, path_pxB = a_star(start_nodeB, goal_nodeB, occ_map, movement='8N')
        self.way_pointsB = self.create_way_points(pathB)
        self.get_logger().info("Finished A* for robot B")
        self.publish = True



    async def do_call(self, pose_msg):

        if self.publish:

            p = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y])
            norm = np.linalg.norm(p - self.midpoint[:2])
            #self.get_logger().info("norm AB {}".format(norm))
            if norm < 3.0:
                self.publish = False
                self.get_logger().info("Start path B publishing")

                await self.do_pathB(self.way_pointsB)

                self.get_logger().info("Finish path B publishing")
                new_msg = String()
                self.end_pub.publish(new_msg)
                self.destroy_node()


    def create_way_points(self, path):

        idx = 0
        node_num = len(path)
        start_node = path[idx]
        way_points = []
        curr_move = np.array([0.0 , 0.0])

        while (idx < node_num - 1):
            move = np.array([path[idx + 1][0] - path[idx][0], path[idx + 1][1] - path[idx][1]])
            if (np.abs(np.sum(curr_move - move)) > 0):

                wp = self.gmap.map2world(np.array([path[idx][0], path[idx][1]]))
                way_points.append(wp)
                curr_move = move
                #self.get_logger().info(f'move {move}') 
            idx += 1

        wp = self.gmap.map2world(np.array([path[node_num - 1][0], path[node_num - 1][1]]))
        way_points.append(wp)
        self.get_logger().info(f'way_points {way_points}', once=False) 

        #way_points = way_points[1:]

        return way_points

  
    def feedback_callback2(self, msg) -> None:
        ...

    async def do_pathB(self, waypoints):
        self.follow_clientB.wait_for_server()
        direction = 1
        while True:
            self.goal_msgB.path.poses.clear()
            for x, y in waypoints:
                p = PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.position.z = 0.06
                #p.pose.orientation.z = 0.0
                #p.pose.orientation.w = 1.0
                #p.pose.orientation.z = math.sin(theta / 2)                
                #p.pose.orientation.w = math.cos(theta / 2)
                #self.get_logger().error("waypoint {} {}".format(x, y))
                self.goal_msgB.path.poses.append(p)

            goal_handle = await self.follow_clientB.send_goal_async(
                self.goal_msgB, feedback_callback=self.feedback_callback2)
            if not goal_handle.accepted:
                self.get_logger().error('Request rejected')
                return
            self.get_logger().info('Start following')
            response = await goal_handle.get_result_async()
            if response.status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done following')
                break                
            else:
                self.get_logger().warning('Failed following')
            direction *= -1






def main(args=None):
 
    rclpy.init(args=args)
    node = RobotB()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    #executor.create_task(node.do_call)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()