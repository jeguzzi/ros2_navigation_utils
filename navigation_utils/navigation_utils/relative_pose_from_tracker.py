from typing import Any

import PyKDL
import rclpy
import rclpy.action
import rclpy.node
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber


def frame_from_msg(msg: Pose) -> PyKDL.Frame:
    pos = PyKDL.Vector(msg.position.x, msg.position.y, msg.position.z)
    rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w)
    return PyKDL.Frame(rot, pos)


class Node(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("test")
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.names = self.declare_parameter('names',
                                            ['RM0001', 'RM0002']).value
        assert (len(self.names) == 2)
        self.pubs = {
            name: self.create_publisher(PointStamped, f'{name}/robot_gt', 1)
            for name in self.names
        }
        self._subs = [
            Subscriber(self, PoseStamped, f"/optitrack/{name}")
            for name in self.names
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
        frame_ab = frame_b.Inverse() * frame_a
        # relative pose of B with respect to A
        frame_ba = frame_ab.Inverse()
        for (name, frame) in zip(self.names, (frame_ba, frame_ab)):
            theta, *_ = frame.M.GetEulerZYX()
            out = PointStamped()
            out.header.stamp = pose_a.header.stamp
            out.header.frame_id = f'{name}/{self.frame_id}'
            out.point.x = frame.p.x()
            out.point.y = frame.p.y()
            out.point.z = frame.p.z()
            self.pubs[name].publish(out)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
