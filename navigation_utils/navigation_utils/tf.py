from typing import Optional

import geometry_msgs.msg
import nav_msgs.msg
import numpy as np
import PyKDL
import rclpy.duration
import rclpy.node
import tf2_ros

from .utils import Path, Pose, Twist


def transform_twist(twist: Twist, msg: nav_msgs.msg.Odometry):
    v = PyKDL.Vector(*twist[0], 0.0)
    w = PyKDL.Vector(0.0, 0.0, twist[1])
    t = transform_from_odometry_msg(msg).M.Inverse()
    v = t * v
    w = t * w
    twist_msg = geometry_msgs.msg.Twist()
    twist_msg.linear.x = v.x()
    twist_msg.linear.y = v.y()
    twist_msg.linear.z = v.z()
    twist_msg.angular.x = w.x()
    twist_msg.angular.y = w.y()
    twist_msg.angular.z = w.z()
    return twist_msg


def transform_from_odometry_msg(msg: nav_msgs.msg.Odometry) -> PyKDL.Frame:
    position_msg = msg.pose.pose.position
    pos = PyKDL.Vector(position_msg.x, position_msg.y, position_msg.z)
    quaterion_msg = msg.pose.pose.orientation
    rot = PyKDL.Rotation.Quaternion(quaterion_msg.x, quaterion_msg.y,
                                    quaterion_msg.z, quaterion_msg.w)
    return PyKDL.Frame(V=pos, R=rot)


def transform_from_msg(msg: geometry_msgs.msg.Transform) -> PyKDL.Frame:
    position_msg = msg.translation
    pos = PyKDL.Vector(position_msg.x, position_msg.y, position_msg.z)
    quaterion_msg = msg.rotation
    rot = PyKDL.Rotation.Quaternion(quaterion_msg.x, quaterion_msg.y,
                                    quaterion_msg.z, quaterion_msg.w)
    return PyKDL.Frame(V=pos, R=rot)


def pose_from_msg(msg: geometry_msgs.msg.Pose, frame: PyKDL.Frame) -> Pose:
    pos = PyKDL.Vector(msg.position.x, msg.position.y, msg.position.z)
    rot = PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w)
    pose = frame * PyKDL.Frame(rot, pos)
    return np.asarray(tuple(pose.p)[:2]), pose.M.GetEulerZYX()[0]


class TF:

    def __init__(self, node: rclpy.node.Node) -> None:
        super().__init__()
        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(buffer=self.buffer,
                                                  node=node)

    def get_transform(self,
                      from_frame: str,
                      to_frame: str,
                      at: Optional[rclpy.time.Time] = None,
                      timeout: float = 0.0) -> Optional[PyKDL.Frame]:
        if from_frame == to_frame:
            return PyKDL.Frame()
        if at is None:
            at = rclpy.time.Time()
        try:
            transform_msg = self.buffer.lookup_transform(
                to_frame,
                from_frame,
                at,
                timeout=rclpy.duration.Duration(nanoseconds=timeout * 1e9))
            return transform_from_msg(transform_msg.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def get_path_in_frame(
            self,
            msg: nav_msgs.msg.Path,
            frame: str,
            orientation_interpolation: str = 'linear') -> Optional[Path]:
        t = self.get_transform(msg.header.frame_id, frame)
        if t is None:
            return None
        xys = []
        thetas = []
        for p_msg in msg.poses:
            xy, theta = pose_from_msg(p_msg.pose, t)
            xys.append(xy)
            thetas.append(theta)
        return Path(np.asarray(xys),
                    np.asarray(thetas),
                    orientation_interpolation=orientation_interpolation)

    def get_pose_in_frame(self, msg: nav_msgs.msg.Odometry,
                          frame: str) -> Optional[Pose]:
        t = self.get_transform(msg.header.frame_id, frame)
        if t is None:
            return None
        return pose_from_msg(msg.pose.pose, t)
