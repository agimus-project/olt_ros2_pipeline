import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from olt_ros2_pipeline.olt_ros2_pipeline.detection_pose_filter import DetectionFilter
from olt_ros2_pipeline.happypose_labeler import HappyposeLabeler

from m3t_tracker_ros.real_time_tracker_node import RealTimeTrackerNode
from m3t_tracker_ros.time_catchup_node import TimeCatchupNode


def main() -> None:
    """Creates the multi threaded executor and adds all expected nodes to it."""
    rclpy.init()
    try:
        real_time_tracker_node = RealTimeTrackerNode()
        time_catchup_node = TimeCatchupNode()
        detection_filter = DetectionFilter()
        happypose_labeler = HappyposeLabeler()

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(real_time_tracker_node)
        executor.add_node(time_catchup_node)
        executor.add_node(detection_filter)
        executor.add_node(happypose_labeler)
        executor.spin()

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)

    real_time_tracker_node.destroy_node()


if __name__ == "__main__":
    main()
