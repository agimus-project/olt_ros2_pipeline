from collections import deque
import numpy as np
import pinocchio as pin


class TranslationFilterException(Exception):
    """Custom exception used to handle too big translations."""

    pass


class TranslationFilter:
    def __init__(
        self,
        min_buffer_size: int,
        max_buffer_size: int,
        alpha_t: float,
        alpha_o: float,
        max_delta_angle: float,
        max_delta_distance: float,
    ) -> None:
        """Initializes Translation filter class.

        :param min_buffer_size: Minimum number od poses to start checking pose consistency.
        :type min_buffer_size: int
        :param max_buffer_size: Maximum size of pose buffer.
        :type max_buffer_size: int
        :param alpha_t: First order filter coefficient for translation pose.
        :type alpha_t: float
        :param alpha_o: First order filter coefficient for translation rotation.
        :type alpha_o: float
        :param max_delta_angle: Maximum angle difference between detections.
        :type max_delta_angle: float
        :param max_delta_distance: Maximum pose difference between detections.
        :type max_delta_distance: float
        """
        self._min_buffer_size = min_buffer_size
        self._alpha_t = alpha_t
        self._alpha_o = alpha_o
        self._max_delta_angle = max_delta_angle
        self._max_delta_distance = max_delta_distance

        self._buffer = deque(maxlen=max_buffer_size)

    def update_params(
        self,
        new_min_buffer_size: int,
        new_max_buffer_size: int,
        new_alpha_t: float,
        new_alpha_o: float,
        new_max_delta_angle: float,
        new_max_delta_distance: float,
    ) -> None:
        """Dynamically updates parameters of the filer.

        :param new_min_buffer_size: Minimum number od poses to start checking pose consistency.
        :type new_min_buffer_size: int
        :param new_max_buffer_size: Maximum size of pose buffer.
        :type new_max_buffer_size: int
        :param new_alpha_t: First order filter coefficient for translation pose.
        :type new_alpha_t: float
        :param new_alpha_o: First order filter coefficient for translation rotation.
        :type new_alpha_o: float
        :param new_max_delta_angle: Maximum angle difference between detections.
        :type new_max_delta_angle: float
        :param new_max_delta_distance: Maximum pose difference between detections.
        :type new_max_delta_distance: float
        """
        self._min_buffer_size = new_min_buffer_size
        self._alpha_t = new_alpha_t
        self._alpha_o = new_alpha_o
        self._max_delta_angle = new_max_delta_angle
        self._max_delta_distance = new_max_delta_distance

        new_buff = deque(maxlen=new_max_buffer_size)
        buff_len = len(self._buffer)
        start = 0 if new_max_buffer_size >= buff_len else buff_len - new_max_buffer_size
        for i in range(buff_len - start):
            new_buff.append(self._buffer[i + start])

        self._buffer = new_buff

    def add_pose(self, pose: pin.SE3) -> bool:
        """Adds new pose to the cyclic buffer.

        :param pose: Pose to add to the buffer.
        :type pose: pin.SE3
        :return: Indication whether number of detections in the buffer passed minimum.
        :rtype: bool
        """
        self._buffer.append(pose)
        return len(self._buffer) >= self._min_buffer_size

    def get_filtered(self) -> pin.SE3:
        """Ensures pose of the tracked object is not flickering too much. If pose is
        valid, performs firs order filtering.

        :raises RuntimeError: Not enough samples in the buffer.
        :raises TranslationFilterException: Relative distance between poses exceeded threshold.
        :raises TranslationFilterException: Relative rotation between poses exceeded threshold.
        :return: Filtered pose of the tracked object.
        :rtype: pin.SE3
        """
        if len(self._buffer) < self._min_buffer_size:
            raise RuntimeError("Not enough observations to perform validation!")

        # Maximum recorded difference between two consecutive frames.
        max_relative_angle = 0.0
        max_relative_distance = 0.0
        for i in range(1, len(self._buffer)):
            # Compute difference of two poses
            pose_diff = self._buffer[i - 1].inverse() * self._buffer[i]
            # Find distance and rotation corresponding to the difference
            relative_angle = np.linalg.norm(pin.log(pose_diff.rotation))
            relative_distance = np.linalg.norm(pose_diff.translation)
            # If this difference is bigger than old maximum, set is as a new maximum
            if relative_angle > max_relative_angle:
                max_relative_angle = relative_angle
            if relative_distance > max_relative_distance:
                max_relative_distance = relative_distance

        if not np.rad2deg(max_relative_angle) < self._max_delta_distance:
            raise TranslationFilterException("Relative angle is too big.")

        if not max_relative_distance < self._max_delta_angle:
            raise TranslationFilterException("Relative distance is too big.")

        # Compute smoothing of the translation
        tx = self._buffer[-1].translation
        ty = self._buffer[-2].translation
        # alpha * tx + (1 - alpha) * ty
        tf = self._alpha_t * tx + (1.0 - self._alpha_t) * ty

        # Use slerp to smooth rotation
        Rx = self._buffer[-1].rotation
        Ry = self._buffer[-2].rotation
        Rf = Ry @ pin.exp3(pin.log3(Ry.T @ Rx) * self._alpha_o)

        return pin.SE3(Rf, tf)
