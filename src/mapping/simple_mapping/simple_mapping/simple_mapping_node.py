import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge

from simple_mapping import SimpleMapping, SimpleMappingConfig, UNKNOWN, WALL, SPACE


class SimpleMappingNode(Node):
    def __init__(self):
        super().__init__("simple_mapping_node")
        self._lidar_sub = self.create_subscription(
            PointCloud2, "lidar", self.lidar_callback, QoSPresetProfiles.SENSOR_DATA.value)

        self._test_grid_pub = self.create_publisher(
            Image, "test_grid", QoSPresetProfiles.SENSOR_DATA.value)

        self.bridge = CvBridge()
        self.count = 0

        self.config = SimpleMappingConfig()
        self._mapping = SimpleMapping(self.config)

        self.count = 1

    def lidar_callback(self, msg: PointCloud2):
        arr = self._pointcloud2_to_grid(msg)
        # import pickle
        # if self.count <= 2:
        #     with open(f"scan_{self.count}.pkl", "wb") as f:
        #         pickle.dump(arr, f)
        #         self.count += 1
        self._mapping.on_new_map_part_received(arr)

    def _pointcloud2_to_grid(self, pc2: PointCloud2) -> np.ndarray:
        arr = np.frombuffer(pc2.data.tobytes(), dtype=np.float32)
        arr = np.reshape(
            arr, (len(arr) // len(pc2.fields), len(pc2.fields)))[:, 0:2]
        arr = (arr / self.config.grid_size_m)
        # arr[:, 0] = -arr[:, 0]
        # arr[:,[1,0]] = arr[:,[0,1]]
        return arr

    


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
