"""
Interface between Donkeysim and TritonAIRacer
"""
import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import cv_bridge

from sensor_msgs.msg import PointCloud2, Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from tai_interface.msg import VehicleControl


from .donkeysim_client import TelemetryPack, GymInterface, TelemetryInterface


class DonkeysimClientNode(Node, TelemetryInterface):
    def __init__(self):
        super().__init__("donkeysim_interface_node")
        self.bridge = cv_bridge.CvBridge()

        self._img_pub = self.create_publisher(
            Image, 'cam_front', qos_profile_sensor_data)
        self._lidar_pub = self.create_publisher(
            PointCloud2, 'lidar', qos_profile_sensor_data)
        self._imu_pub = self.create_publisher(
            Imu, 'imu/raw', qos_profile_sensor_data)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'pose', qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(
            Odometry, 'odom', qos_profile_sensor_data)

        self.declare_parameter('client_config_pkg', 'donkeysim_tai_interface')
        self.declare_parameter('client_config_file',
                               'param/donkeysim_client.yaml')

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

        ctl_sub_qos = QoSProfile(depth=1)
        ctl_sub_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._control_sub = self.create_subscription(
            VehicleControl, "vehicle_cmd", self._vehicle_control_callback, ctl_sub_qos)

        self._control_timer = self.create_timer(
            0.05, self._control_timer_callback)

    def image_callback(self, img):
        img_msg = self.bridge.cv2_to_imgmsg(img[..., ::-1])
        if self._img_pub:
            self._img_pub.publish(img_msg)

    def lidar_callback(self, lidar):
        pass

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
            self.steer = msg.steering_openloop.steer
            self.throttle = msg.throttle.throttle
            self.brake = msg.brake.brake

    def _control_timer_callback(self):
        self.gym_interface.send_controls(self.steer, self.throttle, self.brake)

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


def main(args=None):
    rclpy.init(args=args)
    node = DonkeysimClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
