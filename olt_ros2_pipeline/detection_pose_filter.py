import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import Point, Pose, Quaternion
from vision_msgs.msg import Detection2DArray, VisionInfo

from olt_ros2_pipeline.translation_filer import (
    TranslationFilter,
    TranslationFilterException,
)

# Automatically generated file
from olt_ros2_pipeline.detection_pose_filter_parameters import (
    detection_pose_filter,
)  # noqa: E402


class DetectionPoseFilter(Node):
    """ROS node ensuring there is no pose flickering between consecutive detections."""

    def __init__(self, *args, **kwargs):
        """Initializes ROS node. Creates subscribers and publishers."""
        # Get the node name. If not set, default to ``detection_pose_filter``
        node_name = kwargs.pop("node_name", "detection_pose_filter")
        super().__init__(node_name, *args, **kwargs)

        try:
            self._param_listener = detection_pose_filter.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # dict[str, TranslationFilter]
        self._filtered_tracks = {}

        # Detections subscribers
        detection_approx_time_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "unfiltered/detections"),
                Subscriber(self, VisionInfo, "unfiltered/vision_info"),
            ],
            queue_size=5,
            slop=0.01,
        )
        # Register callback depending on the configuration
        detection_approx_time_sync.registerCallback(self._detection_data_cb)

        # Publishers
        self._detection_pub = self.create_publisher(
            Detection2DArray, "filtered/detections", 10
        )
        self._vision_info_pub = self.create_publisher(
            VisionInfo, "filtered/vision_info", 10
        )

        self.get_logger().info("Node initialized.")

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback triggered every time synchronized detection
        and vision info messages arrive. Registers observed tracks and
        applies filtering and threshold onto them.

        :param detections: Received detections array.
        :type detections: vision_msgs.msg.Detection2DArray
        :param detections: Received vision info message.
        :type detections: vision_msgs.msg.VisionInfo
        """
        # Update parameters
        if self._param_listener.is_old(self._params):
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()
            for filter in self._filtered_tracks.values():
                filter.update_params(
                    self._params.buffer_size.min,
                    self._params.buffer_size.max,
                    self._params.alpha_t,
                    self._params.alpha_o,
                    self._params.max_delta_angle,
                    self._params.max_delta_distance,
                )

        # Create new output list of detections
        filtered_detections = Detection2DArray()
        filtered_detections.header.stamp = self.get_clock().now().to_msg()
        filtered_detections.header.frame_id = detections.header.frame_id

        # Add postfix to the method
        vision_info.method += "-smoothed"
        for detection in detections.detections:
            # If track id was unknown, register it
            if detection.id not in self._filtered_tracks:
                self._filtered_tracks[detection.id] = TranslationFilter(
                    self._params.buffer_size.min,
                    self._params.buffer_size.max,
                    self._params.alpha_t,
                    self._params.alpha_o,
                    self._params.max_delta_angle,
                    self._params.max_delta_distance,
                )
                self.get_logger().info(
                    f"New track registered with id: '{detection.id}'"
                )

            # Convert ROS message for the pinocchio SE3
            p = detection.results[0].pose.pose
            pose = pin.XYZQUATToSE3(
                np.array(
                    [
                        p.position.x,
                        p.position.y,
                        p.position.z,
                        p.orientation.x,
                        p.orientation.y,
                        p.orientation.z,
                        p.orientation.w,
                    ]
                )
            )
            # Apply filtering
            if not self._filtered_tracks[detection.id].add_pose(pose):
                continue

            try:
                filtered = self._filtered_tracks[detection.id].get_filtered()
            except TranslationFilterException as err:
                self.get_logger().warn(
                    f"Pose of the object with id '{detection.id}' "
                    f"had filtering error: '{str(err)}'"
                )
                continue

            # Recover ROS message from pinocchio type
            filtered_detection = detection
            pose_vec = pin.SE3ToXYZQUAT(filtered)
            filtered_detection.results[0].pose.pose = Pose(
                position=Point(**dict(zip("xyz", pose_vec[:3]))),
                orientation=Quaternion(**dict(zip("xyzw", pose_vec[3:]))),
            )

            filtered_detections.detections.append(filtered_detection)

        self._detection_pub.publish(filtered_detections)
        self._vision_info_pub.publish(vision_info)


def main(args=None):
    rclpy.init(args=args)
    detection_pose_filter = DetectionPoseFilter()
    rclpy.spin(detection_pose_filter)
    detection_pose_filter.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
