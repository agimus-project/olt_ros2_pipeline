from collections import deque, namedtuple

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import CONVERSION_CONSTANT, Time

from vision_msgs.msg import Detection2DArray

DetectionStamped = namedtuple("DetectionStamped", ["stamp", "detection_array"])


class DetectionBuffer:
    """Class handling time buffering for Detection2DArray messages."""

    def __init__(self, node: Node, timeout: float = 5.0) -> None:
        """Initializes the class.

        :param node: ROS node used to obtain time base for filtration.
        :type node: rclpy.Node
        :param timeout: Timeout in seconds used to filter too old
            values in the buffer, defaults to 5.0.
        :type timeout: float, optional
        """
        self._timeout = Duration(seconds=timeout)
        self._queue = deque()
        self._node = node

    def __len__(self) -> int:
        """Returns current number of items in the buffer.

        :return: Current number of items in the buffer.
        :rtype: int
        """
        return len(self._queue)

    @property
    def timeout(self) -> float:
        """Getter for buffer timeout value. Returns the timeout in seconds.

        :return: Timeout in seconds.
        :rtype: float
        """
        return self._timeout.nanoseconds / CONVERSION_CONSTANT

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        """Setter for buffer timeout value. If new timeout is smaller then old one
        length of the queue is updated.

        :param timeout: Time in seconds.
        :type timeout: float
        """
        old = self._timeout
        self._timeout = Time(seconds=timeout)
        if old > self._timeout:
            self._remove_too_old()

    def _remove_too_old(self) -> None:
        """Removes all detections that are older than timeout."""
        now = self._node.get_clock().now()
        # Remove all detections that are too old
        while len(self._queue) > 0 and (now - self._queue[0].stamp) > self._timeout:
            self._queue.popleft()

    def _is_queue_sorted(self) -> bool:
        """Sorts the detection queue in time.

        :return: ``True`` if queue is sorted.
        :rtype: bool
        """
        return all(
            self._queue[i].stamp <= self._queue[i + 1].stamp
            for i in range(len(self._queue) - 1)
        )

    def append(self, detections: Detection2DArray) -> None:
        """Inserts new detection to the queue.

        :param detections: New detection to insert into queue.
        :type detections: vision_msgs.msg.Detection2DArray
        """
        if detections.detections:
            stamp = Time.from_msg(detections.detections[0].header.stamp)
            self._queue.append(DetectionStamped(stamp, detections))
            if not self._is_queue_sorted():
                self._queue = sorted(self._queue, key=lambda val: val.stamp)

        self._remove_too_old()

    def get_closest(self, stamp: Time) -> Detection2DArray:
        """Returns detection from the queue with a time stamp closest
        to one provided in the parameter.

        :param stamp: Time stamp used to find closes detection.
        :type stamp: rclpy.Time
        :raises ValueError: Time stamp is older than oldest value in the queue.
        :return: Detections closest in time to requested time stamp.
        :rtype: vision_msgs.msg.Detection2DArray
        """
        if (self._node.get_clock().now() - stamp) > self._timeout:
            t = stamp.seconds_nanoseconds()
            raise ValueError(
                "When obtaining closest detection from buffer "
                f"expected stamp '{t[0]}.{t[1]}' is too old!"
            )

        return min(
            self._queue, key=lambda val: abs((val.stamp - stamp).nanoseconds)
        ).detection_array
