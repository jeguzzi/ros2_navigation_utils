#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

from .GMAP import GMAP
import cv2
import numpy as np
import time


class GenerateScenario(Node):

    def __init__(self):
        super().__init__('GenerateScenario')

        self.declare_parameter('mapPath', 'meow')
        mapPath = self.get_parameter('mapPath').value
        self.get_logger().info("mapPath: %s" % (str(mapPath),))

        self.declare_parameter('frame_id', 'odom')
        self.frame_id = self.get_parameter('frame_id').value
        self.get_logger().info("frame_id: %s" % (str(self.frame_id),))

        self.scenario_pub = self.create_publisher(PoseArray, 'scenario', 10)
        self.kill_sub = self.create_subscription(String, 'death', self.do_die, 1)


        self.gmap = GMAP(mapPath)
        self.gmap.map[self.gmap.map == (255-205)] = 255
        kernel = np.ones((21,21),np.uint8)
        self.gmap.map = cv2.dilate(self.gmap.map, kernel, iterations = 1)
        cv2.imwrite("scenario_map.png", self.gmap.map)



    def do_die(self, msg):

        self.get_logger().info("suicide!")
        exit()

    def createPoseMsg(self, pnt):

        p = Pose()
        p.position.x = pnt[0]
        p.position.y = pnt[1]
        p.position.z = 0.06
        q = quaternion_from_euler(0, 0, pnt[2])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        return p



    def generateValidPose(self):

        br = self.gmap.BottomRight()
        tl = self.gmap.TopLeft()
        brw = self.gmap.map2world(br)
        tlw = self.gmap.map2world(tl)

        x = np.random.random(1)[0]  * (brw[0] - tlw[0]) + tlw[0]
        y = np.random.random(1)[0]  * (brw[1] - tlw[1]) + tlw[1];

        while(self.gmap.isValid([x, y]) == False):
            x = np.random.random(1)[0]  * (brw[0] - tlw[0]) + tlw[0]
            y = np.random.random(1)[0]  * (brw[1] - tlw[1]) + tlw[1];

        theta = 0.0 #np.random.random(1)[0] * 2 * np.pi - np.pi;
        return np.array([x, y, theta])


    async def do_call(self):

        time.sleep(1)

        start = [0.0, 0.0, 0.0] #self.generateValidPose()
        #start = self.generateValidPose()
        end = self.generateValidPose()


        if (np.linalg.norm(start[:2] - end[:2]) < 7.0):
            end = self.generateValidPose()


        self.get_logger().info("start point ({} {} {})".format(start[0], start[1], start[2]))
        self.get_logger().info("end point ({} {} {})".format(end[0], end[1], end[2]))

        poseArray = PoseArray()
        poseArray.header.frame_id = self.frame_id
        poseArray.header.stamp = self.get_clock().now().to_msg()
        poseArray.poses.append(self.createPoseMsg(start))
        poseArray.poses.append(self.createPoseMsg(end))

        self.scenario_pub.publish(poseArray)






def main(args=None):
 
    rclpy.init(args=args)
    node = GenerateScenario()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.create_task(node.do_call)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()