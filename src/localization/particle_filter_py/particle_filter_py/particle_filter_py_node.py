import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tai_interface.msg import FloatArray
from particle_filter import ParticleFilter, ParticleFilterConfig
from tf2_ros import TransformBroadcaster

FEATURE_POINTS = np.array([
    [-51.587, 49.8],
    [7.059, 26.322],
    [-31.534, 15.743],
    [-46.468, 12.922],
    [-19.183, 4.6049],
    [-52.915, -1.8627],
    [17.499, -6.5173]
], dtype=np.float32)

INITIAL_POS = (0.0, 0.0)
INITIAL_YAW = 180.0


class ParticleFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("particle_filer_py_node")

        self.filter_ = ParticleFilter(ParticleFilterConfig(
            INITIAL_POS, INITIAL_YAW), FEATURE_POINTS, self.get_logger().info)

        self.pose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, "pose_estimate", QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self.pc_sub_ = self.create_subscription(
            PointCloud2, "lidar", self.pc_callback_, QoSPresetProfiles.SENSOR_DATA.value)
        self.br_ = TransformBroadcaster(self)
        self.pose_timer_ = self.create_timer(0.05, self.pose_timer_callback_)

    def _pointcloud2_to_array(self, pc2: PointCloud2) -> np.ndarray:
        arr = np.frombuffer(pc2.data.tobytes(), dtype=np.float32)
        arr = np.reshape(
            arr, (len(arr) // len(pc2.fields), len(pc2.fields)))[:, 0:2]
        # arr[:, 0] = -arr[:, 0]
        # arr[:,[1,0]] = arr[:,[0,1]]
        return arr

    def pc_callback_(self, msg: PointCloud2):
        arr = self._pointcloud2_to_array(msg)
        arr = arr[np.all(arr != 0, axis=1)][:, 0:2]
        self.filter_.update_lidar(arr)

    def pose_timer_callback_(self):
        position, orientation = self.filter_.get_pose()
        position = np.append(position, 0.0)
        position = position.astype(float)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.w = orientation[0]
        msg.pose.pose.orientation.x = orientation[1]
        msg.pose.pose.orientation.y = orientation[2]
        msg.pose.pose.orientation.z = orientation[3]
        self.pose_pub_.publish(msg)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        t.transform.rotation.w = orientation[0]
        t.transform.rotation.x = orientation[1]
        t.transform.rotation.y = orientation[2]
        t.transform.rotation.z = orientation[3]
        self.br_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
