import signal
from typing import Any, Optional

import geometry_msgs.msg
import nav_msgs.msg
import navigation_msgs.action
import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.executors
import rclpy.node
import rclpy.task

from .tf import TF, transform_twist
from .utils import Path, follow_path


class Controller(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("follow_path_server")

        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.use_pose = self.declare_parameter('use_pose', True).value
        self.tau = self.declare_parameter('tau', 0.5).value
        self.horizon = self.declare_parameter('horizon', 0.1).value
        self.multithreaded = self.declare_parameter('multithreaded',
                                                    False).value
        self.get_logger().info(
            "Started FollowPath server with "
            f"frame_id {self.frame_id}, tau {self.tau}, "
            f"horizon {self.horizon}, multithreaded {self.multithreaded}")

        self.tf = TF(self)
        self.cmd_vel_pub = self.create_publisher(geometry_msgs.msg.Twist,
                                                 'cmd_vel', 10)
        self.path: Optional[Path] = None
        self.future: Optional[rclpy.task.Future] = None
        self.goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        if self.use_pose:
            self.create_subscription(
                geometry_msgs.msg.PoseStamped,
                'pose',
                self.has_received_pose,
                1,
                callback_group=rclpy.callback_groups.
                MutuallyExclusiveCallbackGroup() if self.multithreaded else None)
        else:
            self.create_subscription(
                nav_msgs.msg.Odometry,
                'odom',
                self.has_received_odom,
                1,
                callback_group=rclpy.callback_groups.
                MutuallyExclusiveCallbackGroup() if self.multithreaded else None)
        self.follow_action_server = rclpy.action.ActionServer(
            self,
            navigation_msgs.action.FollowPath,
            'follow_path',
            execute_callback=self.follow_cb,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_cb,
            callback_group=rclpy.callback_groups.
            MutuallyExclusiveCallbackGroup() if self.multithreaded else None)

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

    def update_control(self, msg: geometry_msgs.msg.Pose, frame_id: str) -> None:
        if self.path and self.goal_handle and self.future:
            r = self.goal_handle.request
            pose = self.tf.get_pose_in_frame(msg, frame_id, self.frame_id)
            if not pose:
                self.get_logger().error(
                    f"Could not transform pose in {frame_id} "
                    f"to frame {self.frame_id}")
                return
            position, orientation = pose
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
        self.update_control(msg.pose.pose, msg.header.frame_id)

    def has_received_pose(self, msg: geometry_msgs.msg.PoseStamped) -> None:
        self.update_control(msg.pose, msg.header.frame_id)

    async def follow_cb(
            self, goal_handle: rclpy.action.server.ServerGoalHandle) -> bool:
        self.path = self.tf.get_path_in_frame(
            goal_handle.request.path, self.frame_id,
            goal_handle.request.orientation_interpolation)
        if not self.path:
            self.get_logger().warning(
                f'Cannot compute valid path in frame {self.frame_id} '
                f'from goal path {goal_handle.request.path}')
            goal_handle.succeed()
            return navigation_msgs.action.FollowPath.Result(success=False)
        self.get_logger().info(f'Start following path in frame {self.frame_id}')
        self.future = rclpy.task.Future()
        self.goal_handle = goal_handle
        success = await self.future
        if goal_handle.is_cancel_requested:
            self.get_logger().warning('Action has been cancelled')
            goal_handle.canceled()
        elif goal_handle.is_active:
            self.get_logger().info(f'Action has succeed: {success}')
            goal_handle.succeed()
        else:
            self.get_logger().error(f'Action no more active')
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

    if node.multithreaded:
        executor: rclpy.executors.Executor = rclpy.executors.MultiThreadedExecutor()
    else:
        executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()
    node.destroy_node()
