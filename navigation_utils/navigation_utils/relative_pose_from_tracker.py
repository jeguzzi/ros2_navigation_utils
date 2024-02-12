from typing import Any

import PyKDL
import rclpy
import rclpy.action
import rclpy.node
from geometry_msgs.msg import Pose, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber

BODIES = ('RM0001', 'RM0002')


def frame_from_msg(msg: Pose) -> PyKDL.Frame:
    pos = PyKDL.Vector(msg.position.x, msg.position.y, msg.position.z)
    rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w)
    return PyKDL.Frame(rot, pos)


class Node(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("test")
        self._subs = [
            Subscriber(self, PoseStamped, f"/optitrack/{name}")
            for name in BODIES
        ]
        self._ts = ApproximateTimeSynchronizer(self._subs, 1, 0.01)
        self._ts.registerCallback(self.has_received_poses)

    def has_received_poses(self, pose_a: PoseStamped, pose_b: PoseStamped):
        if pose_a.header.frame_id != pose_b.header.frame_id:
            self.get_logger().warning(
                f"Poses have different frames {pose_a.header.frame_id} "
                f"!= {pose_b.header.frame_id}")
        frame_a = frame_from_msg(pose_a.pose)
        frame_b = frame_from_msg(pose_b.pose)
        # relative pose of A with respect to B
        frame = frame_b.Inverse() * frame_a
        x = frame.p.x()
        y = frame.p.y()
        theta, *_ = frame.M.GetEulerZYX()
        self.get_logger().info(f"Relative pose: {x} {y} {theta}")


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
