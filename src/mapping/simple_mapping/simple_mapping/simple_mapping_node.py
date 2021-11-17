import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class SimpleMappingNode(Node):
    def __init__(self):
        super().__init__("simple_mapping_node")

    def lidar_callback(self, msg: PointCloud2):
        pass


def pointcloud2_to_grid(pc2: PointCloud2) -> np.ndarray:
    arr = np.fromstring(pc2.data, dtype=float)
    # TODO


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
