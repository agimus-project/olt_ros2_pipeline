import pinocchio as pin
import numpy as np
from collections import deque


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

    def __call__(self, pose: pin.SE3) -> pin.SE3 | None:
        self._buffer.append(pose)

        if len(self._buffer) < self._min_buffer_size:
            return None

        max_rel_angle = 0.0
        max_rel_distance = 0.0
        for i in range(1, len(self._buffer)):
            T_co_1 = self._buffer[i - 1]
            T_co_2 = self._buffer[i]
            T_o1_o2 = T_co_1.inverse() * T_co_2
            rel_angle_o1_o2 = np.linalg.norm(pin.log(T_o1_o2.rotation))
            rel_distance_o1_o2 = np.linalg.norm(T_o1_o2.translation)
            if rel_angle_o1_o2 > max_rel_angle:
                max_rel_angle = rel_angle_o1_o2
            if rel_distance_o1_o2 > max_rel_distance:
                max_rel_distance = rel_distance_o1_o2

        is_valid_rel_angle = np.rad2deg(max_rel_angle) < self._max_delta_distance
        is_valid_rel_distance = max_rel_distance < self._max_delta_angle

        if not is_valid_rel_angle or not is_valid_rel_distance:
            return None

        Tx = self._buffer[-1]
        Ty = self._buffer[-2]

        # Simple for translation
        tx, ty = Tx.translation, Ty.translation
        tf = self._alpha_t * tx + (1 - self._alpha_t) * ty

        # For orientation, implement filtering as slerp
        Rx, Ry = Tx.rotation, Ty.rotation
        # proof reading slerp formula:
        # - alpha_o = 0 (maximum filtering) -> Rf=Ry
        # - alpha_o = 1 (zero filtering) -> Rf=Rx
        Rf = Ry @ pin.exp3(pin.log3(Ry.T @ Rx) * self._alpha_o)

        return pin.SE3(Rf, tf)
