import signal
from typing import Any, Optional

import geometry_msgs.msg
import nav_msgs.msg
import navigation_msgs.action
import rclpy
import rclpy.action
import rclpy.node
import rclpy.task

from .tf import TF, transform_twist
from .utils import Path, follow_path


class Controller(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("follow_path_server")

        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.tau = self.declare_parameter('tau', 0.5).value
        self.horizon = self.declare_parameter('horizon', 0.1).value
        self.tf = TF(self)
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.msg.Twist,
                                                 'cmd_vel', 10)
        self.create_subscription(nav_msgs.msg.Odometry, 'odom',
                                 self.has_received_odom, 1)
        self.path: Optional[Path] = None
        self.future: Optional[rclpy.task.Future] = None
        self.goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        self.follow_action_server = rclpy.action.ActionServer(
            self,
            navigation_msgs.action.FollowPath,
            'follow_path',
            execute_callback=self.follow_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_cb)

    def shutdown(self) -> None:
        if rclpy.ok():
            if self.goal_handle and self.goal_handle.is_active:
                self.goal_handle.abort()
            self.stop()

    def stop(self) -> None:
        if rclpy.ok():
            self.cmd_vel_pub.publish(geometry_msgs.msg.Twist())

    def goal_callback(self,
                      goal_request: Any) -> rclpy.action.server.GoalResponse:
        if self.goal_handle:
            return rclpy.action.server.GoalResponse.REJECT
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_cb(
        self, goal_handle: rclpy.action.server.ServerGoalHandle
    ) -> rclpy.action.CancelResponse:
        if self.future:
            self.future.set_result(False)
        return rclpy.action.CancelResponse.ACCEPT

    def update_control(self, msg: nav_msgs.msg.Odometry) -> None:
        if self.path and self.goal_handle:
            r = self.goal_handle.request
            position, orientation = self.tf.get_pose_in_frame(
                msg, self.frame_id)
            twist, distance, angular_distance = follow_path(
                path=self.path,
                position=position,
                orientation=orientation,
                horizon=self.horizon,
                speed=r.speed,
                angular_speed=r.angular_speed,
                tau=self.tau,
                turn_ahead=r.turn_ahead)
            if distance < r.spatial_goal_tolerance and (
                    r.angular_speed <= 0
                    or angular_distance < r.angular_goal_tolerance):
                self.stop()
                self.future.set_result(True)
                self.path = None
                return
            arrival_time = 0.0
            if r.speed > 0:
                arrival_time += abs(distance -
                                    r.spatial_goal_tolerance) / r.speed
            if r.angular_speed > 0:
                arrival_time += abs(angular_distance -
                                    r.angular_goal_tolerance) / r.angular_speed
            feedback_msg = navigation_msgs.action.FollowPath.Feedback(
                time_to_arrive=arrival_time)
            self.goal_handle.publish_feedback(feedback_msg)
            self.cmd_vel_pub.publish(transform_twist(twist, msg))

    def has_received_odom(self, msg: nav_msgs.msg.Odometry) -> None:
        # self.get_logger().info(f"has_received_odom {msg.pose.pose}")
        self.update_control(msg)

    async def follow_cb(
            self, goal_handle: rclpy.action.server.ServerGoalHandle) -> bool:
        self.path = self.tf.get_path_in_frame(goal_handle.request.path,
                                              self.frame_id,
                                              goal_handle.request.orientation_interpolation)
        if not self.path:
            self.get_logger().warning('Invalid path')
            goal_handle.succeed()
            return navigation_msgs.action.FollowPath.Result(success=False)
        else:
            self.get_logger().info('Start following path')
        self.goal_handle = goal_handle
        self.future = rclpy.task.Future()
        success = await self.future
        if goal_handle.is_cancel_requested:
            goal_handle.cancel()
        elif goal_handle.is_active:
            goal_handle.succeed()
        self.path = None
        self.goal_handle = None
        self.future = None
        return navigation_msgs.action.FollowPath.Result(success=success)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Controller()

    def shutdown(sig, _):
        node.get_logger().info("Stop before exiting")
        node.shutdown()
        node.get_logger().info("Shutdown")
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, shutdown)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.shutdown()
    rclpy.try_shutdown()
    node.destroy_node()
