import cv2

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import Vector3, PoseArray
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection2D, Detection2DArray, VisionInfo
from visualization_msgs.msg import Marker, MarkerArray


class TrackVisualizer(Node):
    """ROS adding labels with detection IDs strings and bounding boxes onto images"""

    def __init__(self, *args, **kwargs):
        """Initializes ROS node. Creates subscribers and publishers."""
        # Get the node name. If not set, default to ``track_visualizer``
        node_name = kwargs.pop("node_name", "track_visualizer")
        super().__init__(node_name, *args, **kwargs)

        self.declare_parameter("text_color", [0.0, 1.0, 0.0, 0.8])
        self._text_color = list(
            self.get_parameter("text_color").get_parameter_value().double_array_value
        )
        self.declare_parameter("text_size", 0.1)
        self._text_size = (
            self.get_parameter("text_size").get_parameter_value().double_value
        )

        self.declare_parameter("marker_lifetime", 2.0)
        self._marker_lifetime = (
            self.get_parameter("marker_lifetime").get_parameter_value().double_value
        )

        # Detections subscribers
        detection_approx_time_sync = ApproximateTimeSynchronizer(
            [
                Subscriber(self, Detection2DArray, "detections"),
                Subscriber(self, VisionInfo, "vision_info"),
            ],
            queue_size=5,
            slop=0.05,
        )
        # Register callback depending on the configuration
        detection_approx_time_sync.registerCallback(self._detection_data_cb)

        self._markers_pub = self.create_publisher(MarkerArray, "markers", 10)
        self._pose_pub = self.create_publisher(PoseArray, "poses", 10)

        self.get_logger().info("Node initialized.")

    def _generate_marker_msg(self, detection: Detection2D, idx: int) -> Marker:
        result = detection.results[0]
        return Marker(
            id=idx,
            mesh_resource="",
            mesh_use_embedded_materials=False,
            type=Marker.TEXT_VIEW_FACING,
            header=detection.header,
            scale=Vector3(**dict(zip("xyz", [self._text_size] * 3))),
            color=ColorRGBA(
                **dict(zip("rgba", self._text_color)),
            ),
            lifetime=Duration(seconds=self._marker_lifetime).to_msg(),
            pose=result.pose.pose,
            text=detection.id,
        )

    def _detection_data_cb(self, detections: Detection2DArray, _: VisionInfo) -> None:
        """Callback triggered every time synchronized detection
        and vision info messages arrive. Converts those messages into marker
        messages and publishes them.

        :param detections: Received detections array.
        :type detections: vision_msgs.msg.Detection2DArray
        :param detections: Received vision info message.
        :type detections: vision_msgs.msg.VisionInfo
        """

        self._markers_pub.publish(
            MarkerArray(
                markers=[
                    self._generate_marker_msg(detection, i)
                    for i, detection in enumerate(detections.detections)
                ],
            )
        )

        self._pose_pub.publish(
            PoseArray(
                header=detections.header,
                poses=[det.results[0].pose.pose for det in detections.detections],
            )
        )


def main(args=None):
    rclpy.init(args=args)
    track_visualizer = TrackVisualizer()
    rclpy.spin(track_visualizer)
    track_visualizer.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
