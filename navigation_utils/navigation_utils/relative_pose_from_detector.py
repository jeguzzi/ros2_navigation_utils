from typing import Any, Dict, Optional, Tuple

import PyKDL
import rclpy
import rclpy.node
import robomaster_msgs.msg
import sensor_msgs.msg
import yaml
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel

from .tf import TF


def project(t: PyKDL.Frame, camera: PinholeCameraModel,
            pixel: Tuple[float, float], z: float) -> PyKDL.Vector:
    pixel = camera.rectifyPoint(pixel)
    u = PyKDL.Vector(*camera.projectPixelTo3dRay(pixel))
    # map frame
    u = t.M * u
    o = t.p
    # we assume that is lies at z = <z>
    s = (z - o.z()) / u.z()
    return o + s * u


def camera_info_from_calibration(
        calibration: Dict[str, Any],
        frame_id: str = '') -> sensor_msgs.msg.CameraInfo:
    msg = sensor_msgs.msg.CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = calibration['image_height']
    msg.width = calibration['image_width']
    msg.distortion_model = calibration['distortion_model']
    msg.d = calibration['distortion_coefficients']['data']
    msg.k = calibration['camera_matrix']['data']
    msg.r = calibration['rectification_matrix']['data']
    msg.p = calibration['projection_matrix']['data']
    return msg


class Node(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("test")
        self.camera: Optional[PinholeCameraModel] = None
        self.tf = TF(self)
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        # A robomaster is about 23 x 29 cm
        self.robot_size = self.declare_parameter('robot_size', 0.26).value
        calibration_file = self.declare_parameter("calibration_file", '').value
        if calibration_file:
            try:
                with open(calibration_file, 'r') as f:
                    calibration = yaml.load(f, yaml.SafeLoader)
                    if calibration:
                        msg = camera_info_from_calibration(calibration)
                        self.has_received_camera_info(msg)
            except FileNotFoundError:
                self.get_logger().warn(
                    f"Camera calibration file not found at {calibration_file}")
        if not self.camera:
            self.create_subscription(sensor_msgs.msg.CameraInfo,
                                     'camera/camera_info',
                                     self.has_received_camera_info, 1)
        self.create_subscription(robomaster_msgs.msg.Detection, "vision",
                                 self.has_received_detection, 1)
        self.pub = self.create_publisher(PointStamped, 'robot', 1)

    def has_received_camera_info(self,
                                 msg: sensor_msgs.msg.CameraInfo) -> None:
        if self.camera is None:
            self.camera = PinholeCameraModel()
            self.camera.fromCameraInfo(msg)
            self.image_width = msg.width
            self.image_height = msg.height

    def has_received_detection(self, msg: robomaster_msgs.msg.Detection):
        if not msg.robots or not self.camera:
            # self.get_logger().warning(f"No camera or robots {msg} {self.camera}")
            return
        frame = self.tf.get_transform(msg.header.frame_id, self.frame_id, None)
        if not frame:
            self.get_logger().warning(
                f"Could not transform from {msg.header.frame_id} to {self.frame_id}"
            )
            return
        roi = msg.robots[0].roi
        # Crude estimation of position:
        # 1. compute middle of lower pixel
        pixel = (roi.x_offset * self.image_width,
                 (roi.y_offset + roi.height * 0.5) * self.image_height)
        # self.get_logger().info(f"Pixel {pixel}")
        # 2. project it on the floor
        p = project(frame, self.camera, pixel, z=0)
        # 3. move out by half the robot's size
        if p.Norm() == 0:
            return
        e = p / p.Norm()
        p = p + e * self.robot_size * 0.5
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.point.x = p.x()
        out.point.y = p.y()
        out.point.z = 0.0
        self.pub.publish(out)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Node()
    rclpy.spin(node)
