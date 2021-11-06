"""
Haoru Xue | July 2021 | hxue@ucsd.edu
"""

from abc import abstractmethod
import numpy as np
from threading import Lock

import rclpy
from rclpy.node import Node

from .wp_follow import WaypointFollower, DEFAULT_PID_CONFIG, DEFAULT_WAYPOINT_CONFIG
from sensor_msgs.msg import NavSatFix, Imu
from tai_interface.msg import VehicleControl
from geometry_msgs.msg import TwistStamped, Twist


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__("wp_follow_node")

        # Params
        DEFAULT_WAYPOINT_CONFIG['n_look_ahead'] = self.declare_parameter(
            'n_look_ahead', 20).get_parameter_value().integer_value
        DEFAULT_WAYPOINT_CONFIG['waypoint_data_file'] = self.declare_parameter(
            'waypoint_data_file', 'waypoints.json').get_parameter_value().string_value
        DEFAULT_PID_CONFIG['steering']['kp'] = self.declare_parameter(
            'steering_kp', 1.0).get_parameter_value().double_value
        DEFAULT_PID_CONFIG['steering']['ki'] = self.declare_parameter(
            'steering_ki', 0.0).get_parameter_value().double_value
        DEFAULT_PID_CONFIG['steering']['kd'] = self.declare_parameter(
            'steering_kd', 0.0).get_parameter_value().double_value
        DEFAULT_PID_CONFIG['throttle']['kp'] = self.declare_parameter(
            'throttle_kp', 1.0).get_parameter_value().double_value
        DEFAULT_PID_CONFIG['throttle']['ki'] = self.declare_parameter(
            'throttle_ki', 0.0).get_parameter_value().double_value
        DEFAULT_PID_CONFIG['throttle']['kd'] = self.declare_parameter(
            'throttle_kd', 0.0).get_parameter_value().double_value
        self.max_wheel_angle_rad = self.declare_parameter(
            'max_wheel_angle_rad', 0.5).get_parameter_value().double_value
        # self.wheel_angle_change_rate = self.declare_parameter(
        #     'wheel_angle_change_rate', 0.5).get_parameter_value().double_value

        self.follower_ = WaypointFollower(
            DEFAULT_PID_CONFIG, DEFAULT_WAYPOINT_CONFIG, self)
        self.gnss_sub_ = self.create_subscription(
            NavSatFix, "/fix", self.gnss_callback_, 1)
        self.vel_sub_ = self.create_subscription(
            TwistStamped, "/vel", self.vel_callback_, 1)
        self.imu_sub_ = self.create_subscription(
            Imu, "/imu/imu_raw", self.imu_callback_, 1)

        self.control_pub_ = self.create_publisher(
            VehicleControl, "vehicle_cmd", 1)

        self.telemetry_ = {"lon": 0.0, "lat": 0.0,
                           "alt": 0.0, "speed": 0.0, "heading": 0.0}
        self.tele_lock_ = Lock()

        self.step_timer_ = self.create_timer(0.05, self.run)

    def gnss_callback_(self, msg: NavSatFix):
        self.tele_lock_.acquire()
        self.telemetry_['lon'] = msg.longitude
        self.telemetry_['lat'] = msg.latitude
        self.telemetry_['alt'] = msg.altitude
        self.tele_lock_.release()

    def vel_callback_(self, msg: TwistStamped):
        self.tele_lock_.acquire()
        self.telemetry_['speed'] = self.__twist_to_vel(msg.twist)
        self.tele_lock_.release()

    def imu_callback_(self, msg: Imu):
        euler = self.euler_from_quaternion(msg.orientation)
        heading = np.degrees(euler[2])
        if heading < 0:
            heading = 360 + heading
        self.tele_lock_.acquire()
        self.telemetry_['heading'] = heading
        self.tele_lock_.release()

    def run(self):
        self.tele_lock_.acquire()
        tele = dict(self.telemetry_)
        self.tele_lock_.release()
        str, thr, _ = self.follower_.step(tele)
        self.send_control(str, thr)

    def send_control(self, steering, throttle):
        msg = VehicleControl()
        if throttle > 0:
            msg.throttle.throttle = throttle
        else:
            msg.brake.brake = -throttle
        msg.steering_rad.steer = self.map_steering(steering)
        self.control_pub_.publish(msg)

    def map_steering(self, steering):
        return steering * self.max_wheel_angle_rad * -1

    def __twist_to_vel(self, twist:Twist):
        lin_vel = np.array([twist.linear.x, twist.linear.y, twist.linear.z], dtype=float)
        for i in range(len(lin_vel)):
            if np.isnan(lin_vel[i]):
                lin_vel[i] = 0.0
        return np.linalg.norm(lin_vel)

    @abstractmethod
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
