import math
from typing import Any
import numpy as np

import action_msgs.msg
import geometry_msgs.msg
import navigation_msgs.action
import rclpy
import rclpy.action
import rclpy.node


class Client(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("client")
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.follow_client = rclpy.action.ActionClient(
            self, navigation_msgs.action.FollowPath, 'follow_path')
        self.waypoints = [[0.0, 0.0, 0.0],
                          [2.0, 0.0, 1.0],
                          [2.0, 2.0, 2.0]]

        self.waypoints = [[4 * np.sin(1.0 * t), 3 * np.cos(1.5 * t), 0.0] for t in np.linspace(0, 2 * np.pi, 100)]

        self.goal_msg = navigation_msgs.action.FollowPath.Goal()
        self.goal_msg.speed = 0.5
        self.goal_msg.angular_speed = 1.0
        self.goal_msg.angular_goal_tolerance = 0.1
        self.goal_msg.spatial_goal_tolerance = 0.1
        self.goal_msg.path.header.frame_id = self.frame_id
        self.goal_msg.turn_ahead = True

    def feedback_callback(self, msg) -> None:
        ...

    async def run(self):
        self.follow_client.wait_for_server()
        direction = 1
        while True:
            self.goal_msg.path.poses.clear()
            for x, y, theta in self.waypoints[::direction]:
                p = geometry_msgs.msg.PoseStamped()
                p.pose.position.x = x
                p.pose.position.y = y
                p.pose.orientation.z = math.sin(theta / 2)
                p.pose.orientation.w = math.cos(theta / 2)
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
    node = Client()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.create_task(node.run)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()