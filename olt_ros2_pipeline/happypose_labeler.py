import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber

from vision_msgs.msg import Detection2DArray, VisionInfo


class HappyposeLabeler(Node):
    """ROS node class applying dummy labels to objects detected by HappyPose ROS node."""

    def __init__(self, *args, **kwargs):
        """Initializes ROS node. Creates subscribers and publishers."""
        # Get the node name. If not set, default to ``happypose_labeler``
        node_name = kwargs.pop("node_name", "happypose_labeler")
        super().__init__(node_name, *args, **kwargs)

        # Detections subscribers
        detection_approx_time_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "unlabeled/detections"),
                Subscriber(self, VisionInfo, "unlabeled/vision_info"),
            ],
            queue_size=5,
            slop=0.01,
        )
        # Register callback depending on the configuration
        detection_approx_time_sync.registerCallback(self._detection_data_cb)

        # Publishers
        self._detection_pub = self.create_publisher(
            Detection2DArray, "labeled/detections", 10
        )
        self._vision_info_pub = self.create_publisher(
            VisionInfo, "labeled/vision_info", 10
        )

        self.get_logger().info("Node initialized.")

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        """Callback called on new synchronized arrival of detection and vision info messages.
        Applies simple labeling of the tracks, by combining class_id with its number
        of instances in the received message.

        :param detections: Incoming detections.
        :type detections: Detection2DArray
        :param vision_info: Incoming vision info.
        :type vision_info: VisionInfo
        """
        # Initialize counter of class ids
        object_types = {}
        for det in detections.detections:
            object_types[det.results[0].hypothesis.class_id] = 0

        # Append id fields in the message
        for i in range(len(detections.detections)):
            class_id = detections.detections[i].results[0].hypothesis.class_id
            detections.detections[i].id = f"{class_id}_{object_types[class_id]}"
            object_types[class_id] += 1

        detections.header.stamp = self.get_clock().now().to_msg()
        vision_info.header.stamp = detections.header.stamp

        # Publish modified data
        self._detection_pub.publish(detections)
        self._vision_info_pub.publish(vision_info)


def main(args=None):
    rclpy.init(args=args)
    happypose_labeler = HappyposeLabeler()
    rclpy.spin(happypose_labeler)
    happypose_labeler.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
