import sys

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from olt_ros2_pipeline.detection_pose_filter import DetectionPoseFilter
from olt_ros2_pipeline.happypose_labeler import HappyposeLabeler

from m3t_tracker_ros.real_time_tracker_node import RealTimeTrackerNode
from m3t_tracker_ros.time_catchup_node import TimeCatchupNode


def main(args=None):
    """Creates the multi threaded executor and adds all expected nodes to it."""
    rclpy.init(args=args)
    try:
        happypose_labeler = HappyposeLabeler(namespace="stage_1")
        time_catchup_node = TimeCatchupNode(
            namespace="stage_2", spin_tf_in_thread=False
        )
        real_time_tracker_node = RealTimeTrackerNode(
            namespace="stage_3", spin_tf_in_thread=False
        )
        detection_filter = DetectionPoseFilter(
            namespace="stage_4", spin_tf_in_thread=False
        )

        executor = MultiThreadedExecutor()
        executor.add_node(real_time_tracker_node)
        executor.add_node(time_catchup_node)
        executor.add_node(detection_filter)
        executor.add_node(happypose_labeler)
        executor.spin()

    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == "__main__":
    main()
