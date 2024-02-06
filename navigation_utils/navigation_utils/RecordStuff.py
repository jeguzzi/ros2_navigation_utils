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

class RecordStuff(Node):

    def __init__(self):
        super().__init__('RecordStuff')


        self.start_pub = self.create_publisher(String, 'start', 10)
        self.end_sub = self.create_subscription(String, 'end', self.do_end, 1)
        self.record_sub = self.create_subscription(String, 'record', self.do_record, 1)



    def do_end(self, msg):

        res = subprocess.Popen("pgrep -f record", shell=True, stdout=subprocess.PIPE).stdout.read()
        pid = res.split(b'\n')[0].decode("utf-8")
        self.get_logger().info("kill {}".format(pid))
        return_code = subprocess.call("kill {}".format(pid), shell=True)
        #self.destroy_node()
    
        # res = subprocess.Popen("pgrep -f follow", shell=True, stdout=subprocess.PIPE).stdout.read()
        # pid = res.split(b'\n')[0].decode("utf-8")
        # self.get_logger().info("kill {}".format(pid))
        # return_code = subprocess.call("kill {}".format(pid), shell=True)
        # pid = res.split(b'\n')[1].decode("utf-8")
        # self.get_logger().info("kill {}".format(pid))
        # return_code = subprocess.call("kill {}".format(pid), shell=True)


        #kill $(pgrep -f joint_state)

        res = subprocess.Popen("pgrep -f scenario.launch", shell=True, stdout=subprocess.PIPE).stdout.read()
        pid = res.split(b'\n')[0].decode("utf-8")
        self.get_logger().info("kill {}".format(pid))
        return_code = subprocess.call("kill {}".format(pid), shell=True)





    def do_record(self, msg):

        subprocess.Popen(['/bin/bash', '-c', "ros2 bag record --all"],  stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        self.get_logger().info("Recording")




def main(args=None):
 
    rclpy.init(args=args)
    node = RecordStuff()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()