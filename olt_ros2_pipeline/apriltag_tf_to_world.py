#!/usr/bin/env python
import rclpy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped, Transform
from tf2_ros.transform_broadcaster import TransformBroadcaster


class ApriltagTfToWorldPose(rclpy.node.Node):
    def __init__(self):
        super().__init__("apriltag_to_msg")
        # TODO: make these parameters configurable through launch file
        self.world_frame = "fer_link0"
        self.object_frame = "tless-obj_000031"
        # self.object_frame = "big_tag1"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_pub = self.create_publisher(PoseStamped, "/object/detections", 5)
        self.timer = self.create_timer(0.01, self.publish_pose_detection_in_world_frame)

    def get_transform(self, parent_frame, child_frame, stamp):
        try:
            transform_msg = self.tf_buffer.lookup_transform(
                parent_frame, child_frame, stamp
            )
            return transform_msg
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {parent_frame} to {child_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return None

    def publish_pose_detection_in_world_frame(self):
        # retrieve in tf pose detection from apriltag
        try:
            wMo_msg: Transform = self.tf_buffer.lookup_transform(
                self.world_frame, self.object_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.world_frame} to {self.object_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        # convert the Transform msg as a PoseStamped and publish it
        ps = PoseStamped(header=wMo_msg.header)
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.world_frame
        ps.pose.position.x = wMo_msg.transform.translation.x
        ps.pose.position.y = wMo_msg.transform.translation.y
        ps.pose.position.z = wMo_msg.transform.translation.z
        ps.pose.orientation = wMo_msg.transform.rotation
        self.pose_pub.publish(ps)


def main():
    rclpy.init()
    node = ApriltagTfToWorldPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
