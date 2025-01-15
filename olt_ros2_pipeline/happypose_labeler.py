import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from message_filters import ApproximateTimeSynchronizer, Subscriber

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from vision_msgs.msg import Detection2D, Detection2DArray, VisionInfo

from happypose_msgs_py.symmetries import discretize_symmetries
from olt_ros2_pipeline.track_matcher import TrackMatcher
from olt_ros2_pipeline.detection_buffer import DetectionBuffer

from happypose_msgs.msg import ObjectSymmetriesArray

# Automatically generated file
from olt_ros2_pipeline.happypose_labeler_parameters import (
    happypose_labeler,
)  # noqa: E402


class HappyposeLabeler(Node):
    """ROS node class applying dummy labels to objects detected by HappyPose ROS node."""

    def __init__(self, spin_tf_in_thread: bool = True, *args, **kwargs):
        """Initializes ROS node. Creates subscribers and publishers."""
        # Get the node name. If not set, default to ``happypose_labeler``
        node_name = kwargs.pop("node_name", "happypose_labeler")
        super().__init__(node_name, *args, **kwargs)

        try:
            self._param_listener = happypose_labeler.ParamListener(self)
            self._params = self._param_listener.get_params()
        except Exception as e:
            self.get_logger().error(str(e))
            raise e

        # Transform buffers
        self._tf_buffer = Buffer()
        self._listener = TransformListener(
            self._tf_buffer, self, spin_thread=spin_tf_in_thread
        )

        # Detections subscribers
        self._happypose_detection_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "unlabeled/detections"),
                Subscriber(self, VisionInfo, "unlabeled/vision_info"),
            ],
            queue_size=5,
            slop=0.01,
        )

        self._tracker_detection_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "upsampled/detections"),
                Subscriber(self, VisionInfo, "upsampled/vision_info"),
            ],
            queue_size=5,
            slop=0.01,
        )

        # Symmetries subscriber
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._symmetries_sub = self.create_subscription(
            ObjectSymmetriesArray,
            "happypose/object_symmetries",
            self._object_symmetries_cb,
            qos,
        )

        # Register callback depending on the configuration
        self._happypose_detection_sync.registerCallback(self._detection_data_cb)
        self._tracker_detection_sync.registerCallback(self._tracker_data_cb)

        # Publishers
        self._detection_pub = self.create_publisher(
            Detection2DArray, "labeled/detections", 10
        )
        self._vision_info_pub = self.create_publisher(
            VisionInfo, "labeled/vision_info", 10
        )

        self._track_matcher = None
        self._tracks_buffer = DetectionBuffer(self, self._params.buffer_timeout)

        self.get_logger().info("Node initialized.")

    def _object_symmetries_cb(self, msg: ObjectSymmetriesArray) -> None:
        """Callback of the object symmetries message topic

        :param msg: Message containing object symmetries
        :type msg: happypose_msgs.msg.ObjectSymmetriesArray
        """
        symmetries = {
            obj.class_id: (
                discretize_symmetries(obj, n_symmetries_continuous=32)
                if self._params.use_symmetry_minimization
                # Create single symmetry representing no transformation disabling this feature
                else np.eye(4).reshape(1, 4, 4)
            )
            for obj in msg.objects
        }

        if self._track_matcher is None:
            self._track_matcher = TrackMatcher(
                symmetries,
                self._params.track_max_dist,
                self._params.distance_cost_weight,
                self._params.angle_cost_weight,
            )
        else:
            self._track_matcher.symmetries = symmetries

    def _transform_detection(
        self, detection: Detection2D, target_frame: str, source_frame: str
    ) -> Detection2D:
        """Transforms pose within detection message and updates ``frame_id``
        in the header of the message.

        :param detection: Input detection message to transform pose.
        :type detection: vision_msgs.msg.Detection2D
        :param target_frame: Frame to transform pose to.
        :type target_frame: str
        :param source_frame: Frame from which pose is transformed.
        :type source_frame: str
        :return: Detection with transformed pose.
        :rtype: vision_msgs.msg.Detection2D
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(detection.header.stamp),
            )
            detection.results[0].pose.pose = do_transform_pose(
                detection.results[0].pose.pose, transform
            )
            detection.header.frame_id = target_frame
            return detection
        except Exception as err:
            self.get_logger().warn(f"Got exception: '{str(err)}'.")
            return detection

    def _tracker_data_cb(self, detections: Detection2DArray, _: VisionInfo) -> None:
        """Callback called on new synchronized arrival of detection and vision info messages.
        Applies labeling of the tracks, by combining class_id with its number of instances
        in the received message. If new detection is close to previously assigned track,
        this track will be applied to new detection.

        :param detections: Incoming detections.
        :type detections: vision_msgs.msg.Detection2DArray
        :param _: Discarded incoming vision info.
        :type _: vision_msgs.msg.VisionInfo
        """
        self._tracks_buffer.append(detections)

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback called on new synchronized arrival of detection and vision info messages.
        Applies labeling of the tracks, by combining class_id with its number of instances
        in the received message. If new detection is close to previously assigned track,
        this track will be applied to new detection.

        :param detections: Incoming detections.
        :type detections: vision_msgs.msg.Detection2DArray
        :param vision_info: Incoming vision info.
        :type vision_info: vision_msgs.msg.VisionInfo
        """
        if self._track_matcher is None:
            self.get_logger().info(
                "No symmetries received yet!", throttle_duration_sec=5.0
            )
            return

        # Update dynamic parameters
        if self._param_listener.is_old(self._params):
            self._param_listener.refresh_dynamic_parameters()
            self._params = self._param_listener.get_params()
            self._track_matcher.track_max_dist = self._params.track_max_dist
            self._track_matcher.distance_cost_weight = self._params.distance_cost_weight
            self._track_matcher.angle_cost_weight = self._params.angle_cost_weight
            self._tracks_buffer.timeout = self._params.buffer_timeout

        # If detections are an empty array, not compute anything
        if len(detections.detections) == 0:
            self._detections = detections
        else:
            # Transform detections to the world frame
            detections.detections = [
                self._transform_detection(
                    det, self._params.static_frame, det.header.frame_id
                )
                for det in detections.detections
            ]

            # Seed the tracker
            if not self._track_matcher.is_seeded:
                self._detections = self._track_matcher.seed_track(detections)
            else:
                # If buffer is not empty try to find closets value in time to current detection
                if len(self._tracks_buffer) == 0:
                    tracked_detections = self._detections
                else:
                    try:
                        tracked_detections = self._tracks_buffer.get_closest(
                            Time.from_msg(detections.detections[0].header.stamp)
                        )
                        # Transform detections to the world frame
                        tracked_detections.detections = [
                            self._transform_detection(
                                det, self._params.static_frame, det.header.frame_id
                            )
                            for det in tracked_detections.detections
                        ]
                    except ValueError as err:
                        self.get_logger().warn(str(err))

                self._track_matcher.update_tracked_objects(tracked_detections)
                self._detections = self._track_matcher.match_tracks(detections)

            # Transform detections back to the camera frame
            self._detections.detections = [
                self._transform_detection(
                    det, detections.header.frame_id, self._params.static_frame
                )
                for det in self._detections.detections
            ]

        self._detections.header.stamp = self.get_clock().now().to_msg()
        vision_info.header.stamp = self._detections.header.stamp

        # Publish modified data
        self._detection_pub.publish(self._detections)
        self._vision_info_pub.publish(vision_info)


def main(args=None):
    rclpy.init(args=args)
    happypose_labeler = HappyposeLabeler()
    rclpy.spin(happypose_labeler)
    happypose_labeler.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
