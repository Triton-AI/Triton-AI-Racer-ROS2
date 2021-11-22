"""
Interface between Donkeysim and TritonAIRacer
"""
import os
import yaml
import math
import ctypes
import multiprocessing
from multiprocessing import sharedctypes

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles
from ament_index_python.packages import get_package_share_directory
import cv_bridge
from sensor_msgs.msg import PointCloud2, Image, Imu, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tai_interface.msg import VehicleControl


from .donkeysim_client import LidarConfig, TelemetryPack, GymInterface, TelemetryInterface

LIDAR_CONFIG = None
LIDAR_DATA = None


def process_lidar_point(lidar_point: dict):
    dist = lidar_point['d']
    x_deg = lidar_point['rx']
    y_deg = lidar_point['ry']
    indices = (y_deg // LIDAR_CONFIG.deg_ang_delta,
               x_deg // LIDAR_CONFIG.deg_per_sweep_inc)

    x, y, z = polar2cart(dist, y_deg + 90, - x_deg)
    LIDAR_DATA[indices[0]][indices[1]][0] = x
    LIDAR_DATA[indices[0]][indices[1]][1] = y
    LIDAR_DATA[indices[0]][indices[1]][2] = z
    LIDAR_DATA[indices[0]][indices[1]][3] = 1


class DonkeysimClientNode(Node, TelemetryInterface):
    def __init__(self):
        super().__init__("donkeysim_interface_node")
        self.bridge = cv_bridge.CvBridge()

        self._img_pub = self.create_publisher(
            Image, 'cam/front', QoSPresetProfiles.SENSOR_DATA.value)
        self._lidar_pub = self.create_publisher(
            PointCloud2, 'lidar', QoSPresetProfiles.SENSOR_DATA.value)
        self._imu_pub = self.create_publisher(
            Imu, 'imu/raw', QoSPresetProfiles.SENSOR_DATA.value)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'pose', QoSPresetProfiles.SENSOR_DATA.value)
        self._odom_pub = self.create_publisher(
            Odometry, 'odom', QoSPresetProfiles.SENSOR_DATA.value)

        self.declare_parameter('client_config_pkg', 'donkeysim_tai_interface')
        self.declare_parameter('client_config_file',
                               'param/donkeysim_client.yaml')
        self.declare_parameter('send_control_interval_ms', 0.05)

        client_config_file = os.path.join(
            get_package_share_directory(self.get_parameter(
                'client_config_pkg').get_parameter_value().string_value),
            self.get_parameter(
                'client_config_file').get_parameter_value().string_value
        )

        with open(client_config_file, 'r') as f:
            client_config = yaml.load(f, Loader=yaml.SafeLoader)
        self.gym_interface = GymInterface(
            debug=self.get_logger().info, config=client_config, tele_callback=self)

        self.steer = 0.0
        self.throttle = 0.0
        self.brake = 0.0

        self._control_sub = self.create_subscription(
            VehicleControl, "vehicle_cmd", self._vehicle_control_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self._control_timer = self.create_timer(
            self.get_parameter('send_control_interval_ms').get_parameter_value().integer_value / 1000, self._control_timer_callback)

        self._lidar_timer = self.create_timer(0.05, self._lidar_timer_callback)

    def image_callback(self, img):
        img_msg = self.bridge.cv2_to_imgmsg(img[..., ::-1])
        img_msg.header.frame_id = 'cam_front_link'
        if self._img_pub:
            self._img_pub.publish(img_msg)

    def lidar_callback(self, lidar: list, lidar_config: LidarConfig):
        self.lidar_list = lidar
        self.lidar_config = lidar_config

    def telemetry_callback(self, tele: TelemetryPack):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'
        pose.pose.pose.position.x = tele.pos_z
        pose.pose.pose.position.y = -tele.pos_x
        pose.pose.pose.position.z = tele.pos_y
        w, x, y, z = self.quaternion_from_euler(
            tele.roll * math.pi / 180, tele.pitch * math.pi / 180, (360.0 - tele.yaw) * math.pi / 180)
        pose.pose.pose.orientation.x = x
        pose.pose.pose.orientation.y = y
        pose.pose.pose.orientation.z = z
        pose.pose.pose.orientation.w = w
        if self._pose_pub:
            self._pose_pub.publish(pose)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        imu.linear_acceleration.x = tele.accel_z
        imu.linear_acceleration.y = -tele.accel_x
        imu.linear_acceleration.z = tele.accel_y
        if self._imu_pub:
            self._imu_pub.publish(imu)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'base_link'
        odom.pose = pose.pose
        odom.twist.twist.linear.x = tele.vel_z
        odom.twist.twist.linear.y = -tele.vel_x
        odom.twist.twist.linear.z = tele.vel_y
        if self._odom_pub:
            self._odom_pub.publish(odom)

    def _vehicle_control_callback(self, msg: VehicleControl):
        if self.gym_interface.car_loaded:
            self.steer = msg.steering_openloop
            self.throttle = msg.throttle
            self.brake = msg.brake

    def _control_timer_callback(self):
        self.gym_interface.send_controls(self.steer, self.throttle, self.brake)

    def _lidar_timer_callback(self):
        global LIDAR_CONFIG
        global LIDAR_DATA
        lidar_msg = PointCloud2()
        lidar_msg.header.frame_id = "lidar_link"
        lidar_msg.header.stamp = self.get_clock().now().to_msg()

        lidar_msg.height = self.lidar_config.num_sweep_levels
        lidar_msg.width = 360 // self.lidar_config.deg_per_sweep_inc

        field_names = ('x', 'y', 'z', 'intensity')
        for i in range(4):
            field = PointField()
            field.name = field_names[i]
            field.offset = 4 * (i)
            field.datatype = PointField.FLOAT32
            field.count = 1
            lidar_msg.fields.append(field)
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 16
        lidar_msg.row_step = lidar_msg.point_step * lidar_msg.width
        lidar_msg.is_dense = False
        LIDAR_CONFIG = self.lidar_config
        LIDAR_DATA = sharedctypes.RawArray(lidar_msg.width * (4 * ctypes.c_float), np.ctypeslib.as_ctypes(
            np.zeros((lidar_msg.height, lidar_msg.width, 4), dtype=np.float32)))
        with multiprocessing.Pool() as p:
            p.map(process_lidar_point, self.lidar_list)
        lidar_msg.data = np.ctypeslib.as_array(LIDAR_DATA).flatten().tobytes()
        self._lidar_pub.publish(lidar_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [w, x, y, z]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q


def polar2cart(r, theta, phi):
    theta = math.radians(theta)
    phi = math.radians(phi)
    return (
        r * math.sin(theta) * math.cos(phi),
        r * math.sin(theta) * math.sin(phi),
        r * math.cos(theta)
    )


def main(args=None):
    rclpy.init(args=args)
    node = DonkeysimClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
