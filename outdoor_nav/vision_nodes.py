"""Launch adapters for the original xiangruicheng visual pipeline."""
import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException


def main():
    kind = sys.argv[1]
    rclpy.init(args=sys.argv[2:])
    if kind == "depth":
        from semantic_direct_navigation.depth_node import DepthObstacles
        node = DepthObstacles()
    elif kind == "segmentation":
        from outdoor_nav.segmentation_node import SegmentationNode
        node = SegmentationNode()
    elif kind == "projection":
        from semantic_local_grid.projection_node import Projection
        node = Projection()
    else:
        raise ValueError("Unknown original vision node")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
