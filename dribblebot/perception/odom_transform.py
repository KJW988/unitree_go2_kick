"""Odometry pose로 odom-frame point를 base-frame으로 바꾸는 순수 NumPy 보조 함수."""

from typing import Iterable

import numpy as np


def quaternion_xyzw_to_rotation(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    """ROS quaternion이 표현하는 base->odom 회전 행렬을 반환한다."""

    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm <= 1e-12:
        raise ValueError("odometry quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    ), dtype=np.float64)


def odom_points_to_base(
    points_odom: np.ndarray,
    translation_odom_from_base: Iterable[float],
    quaternion_odom_from_base_xyzw: Iterable[float],
) -> np.ndarray:
    """``p_odom = R_odom_from_base p_base + t``의 역변환을 row point에 적용한다."""

    points = np.asarray(points_odom, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_odom must have shape (N, 3)")
    translation = np.asarray(tuple(translation_odom_from_base), dtype=np.float64)
    if translation.shape != (3,):
        raise ValueError("translation_odom_from_base must have shape (3,)")
    rotation = quaternion_xyzw_to_rotation(quaternion_odom_from_base_xyzw)
    return (points - translation) @ rotation
