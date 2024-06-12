import rclpy
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber

from vision_msgs.msg import Detection2DArray, VisionInfo


class HappyposeLabeler(Node):
    def __init__(self):
        super().__init__("happypose_labeler")

        self.declare_parameter("skip_msg", 1)
        self._skip_msg = (
            self.get_parameter("skip_msg").get_parameter_value().integer_value
        )

        # Detections subscribers
        detection_approx_time_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "happypose/detections"),
                Subscriber(self, VisionInfo, "happypose/vision_info"),
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

        self._counter = 0

        self.get_logger().info("Node initialized, waiting for keyboard node.")

    def _detection_data_cb(
        self, detections: Detection2DArray, vision_info: VisionInfo
    ) -> None:
        object_types = {}
        for det in detections.detections:
            object_types[det.results[0].hypothesis.class_id] = 0

        if self._counter % self._skip_msg == 0:
            for i in range(len(detections.detections)):
                class_id = detections.detections[i].results[0].hypothesis.class_id
                detections.detections[i].id = f"{class_id}_{object_types[class_id]}"
                object_types[class_id] += 1
            self._detection_pub.publish(detections)
            self._vision_info_pub.publish(vision_info)
        self._counter += 1


def main(args=None):
    rclpy.init(args=args)
    happypose_labeler = HappyposeLabeler()
    rclpy.spin(happypose_labeler)
    happypose_labeler.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
