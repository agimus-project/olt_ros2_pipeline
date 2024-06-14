import numpy as np
import pinocchio as pin

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header, String
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

    def __init__(self, spin_tf_in_thread: bool = True, *args, **kwargs) -> None:
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

        # Transform buffers
        self._buffer = Buffer()
        self._listener = TransformListener(
            self._buffer, self, spin_thread=spin_tf_in_thread
        )

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
        self._track_pub = self.create_publisher(PoseStamped, "filtered/track", 10)

        # Subscribers
        self._filtered_track_id = ""
        self._track_id_sub = self.create_subscription(
            String, "set_filter_track", self._set_track_cb, 10
        )

        self.get_logger().info("Node initialized.")

    def _set_track_cb(self, msg: String) -> None:
        self._filtered_track_id = msg.data

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
            for track_filter in self._filtered_tracks.values():
                track_filter.update_params(
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
        filtered_detections.header.frame_id = self._params.filtering_frame_id

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
            try:
                transform = self._buffer.lookup_transform(
                    self._params.filtering_frame_id,
                    detection.header.frame_id,
                    Time.from_msg(detection.header.stamp),
                )
                # Transform pose to a static frame
                p = do_transform_pose(detection.results[0].pose.pose, transform)
                # Convert ROS message for the pinocchio SE3
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
                filtered_detection.header.frame_id = self._params.filtering_frame_id
                pose_vec = pin.SE3ToXYZQUAT(filtered)
                filtered_detection.results[0].pose.pose = Pose(
                    position=Point(**dict(zip("xyz", pose_vec[:3]))),
                    orientation=Quaternion(**dict(zip("xyzw", pose_vec[3:]))),
                )

                filtered_detections.detections.append(filtered_detection)
            except Exception as err:
                self.get_logger().warn(f"Got exception: '{str(err)}'.")

        self._detection_pub.publish(filtered_detections)
        self._vision_info_pub.publish(vision_info)

        try:
            filtered_detection = next(
                filter(
                    lambda detection: detection.id == self._filtered_track_id,
                    filtered_detections.detections,
                )
            )
            transform = self._buffer.lookup_transform(
                self._params.tracking_frame_id,
                filtered_detection.header.frame_id,
                Time.from_msg(filtered_detection.header.stamp),
            )
            self._track_pub.publish(
                PoseStamped(
                    header=Header(
                        frame_id=self._params.tracking_frame_id,
                        stamp=filtered_detection.header.stamp,
                    ),
                    pose=do_transform_pose(
                        filtered_detection.results[0].pose.pose, transform
                    ),
                )
            )
        except Exception as err:
            self.get_logger().warn(
                f"Failed to obtain tracked pose. Reason: '{str(err)}'.",
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    detection_pose_filter = DetectionPoseFilter()
    rclpy.spin(detection_pose_filter)
    detection_pose_filter.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
