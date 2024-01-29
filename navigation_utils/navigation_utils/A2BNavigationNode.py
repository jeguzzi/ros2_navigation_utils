#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from robomaster_msgs.action import Move

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


class A2BNavigationNode(Node):

    def __init__(self):
        super().__init__('A2BNavigationNode')

        self.declare_parameter('mapPath', 'meow')
        mapPath = self.get_parameter('mapPath').value
        self.get_logger().info("mapPath: %s" % (str(mapPath),))

        self.declare_parameter('robotName', 'meow')
        robotName = self.get_parameter('robotName').value
        self.get_logger().info("robotName: %s" % (str(robotName),))

        self.declare_parameter('startPoint', [0.0, 0.0, 0.0])
        startPoint = self.get_parameter('startPoint').value
        self.get_logger().info("startPoint: %s" % (str(startPoint),))

        self.declare_parameter('endPoint', [0.0, 0.0, 0.0])
        endPoint = self.get_parameter('endPoint').value
        self.get_logger().info("endPoint: %s" % (str(endPoint),))

        self.gmap = GMAP(mapPath)
        bin_map = copy.deepcopy(self.gmap.map) #cv2.cvtColor(self.gmap.map, cv2.COLOR_BGR2GRAY)
        bin_map[bin_map == (255-205)] = 255
        bin_map = bin_map.astype(np.float32) / 255
        bin_map[bin_map < 1.0] = 0.0
        self.occ_map = OccupancyGridMap(bin_map, 1.0)


        start_node = self.gmap.world2map([startPoint[0], startPoint[1]])
        goal_node = self.gmap.world2map([endPoint[0], endPoint[1]])

        # run A*
        path, path_px = a_star(start_node, goal_node, self.occ_map, movement='8N')
        self.waypoints = self.create_way_points(path)

        self.follow_client = ActionClient(self, navigation_msgs.action.FollowPath, "{}/follow_path".format(robotName))
        #self.follow_client = ActionClient(self, navigation_msgs.action.FollowPath, "follow_path")
        self.goal_msg = navigation_msgs.action.FollowPath.Goal()
        self.goal_msg.speed = 0.5
        self.goal_msg.angular_speed = 1.0
        self.goal_msg.angular_goal_tolerance = 0.1
        self.goal_msg.spatial_goal_tolerance = 0.1
        self.goal_msg.path.header.frame_id = 'odom'
        self.goal_msg.turn_ahead = True

        


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
        way_points = way_points[1:]


        self.get_logger().info(f'way_points {way_points}', once=True) 

        return way_points


        
    


    def feedback_callback(self, msg) -> None:
        ...

    async def run(self):
        self.follow_client.wait_for_server()
        direction = 1
        while True:
            self.goal_msg.path.poses.clear()
            for x, y in self.waypoints:
                p = PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                #p.pose.orientation.z = math.sin(theta / 2)                
                #p.pose.orientation.w = math.cos(theta / 2)
                #self.get_logger().error("waypoint {} {}".format(x, y))
                self.goal_msg.path.poses.append(p)
            goal_handle = await self.follow_client.send_goal_async(
                self.goal_msg, feedback_callback=self.feedback_callback)
            if not goal_handle.accepted:
                self.get_logger().error('Request rejected')
                return
            self.get_logger().info('Start following')
            response = await goal_handle.get_result_async()
            if response.status == action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Done following')
            else:
                self.get_logger().warning('Failed following')
            direction *= -1




def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = A2BNavigationNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.create_task(node.run)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

