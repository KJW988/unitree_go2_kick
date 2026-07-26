"""OpenCV AprilTag camera adapter for the targeted-kick perception contract.

선택된 upright AprilTag의 pose를 보정된 pinhole camera로 추정해 Go2 base
frame의 ground-plane XY로 반환한다. 검출·보정 실패에는 측정치를 만들지 않아
supervisor가 SEARCH에 남도록 한다.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


Vec2 = Tuple[float, float]


@dataclass(frozen=True)
class AprilTagCameraConfig:
    """Camera intrinsics와 rigid camera-to-base transform.

    ``rotation_base_from_camera``는 OpenCV camera vector(x right, y down, z
    forward)를 Go2 base frame(x forward, y left, z up)으로 변환한다.
    """

    camera_matrix: Sequence[Sequence[float]]
    distortion_coefficients: Sequence[float]
    tag_size_m: float
    rotation_base_from_camera: Sequence[Sequence[float]]
    translation_base_from_camera_m: Sequence[float]
    dictionary_name: str = "DICT_APRILTAG_36h11"
    min_area_px: float = 250.0


@dataclass(frozen=True)
class AprilTagGroundObservation:
    tag_id: int
    tag_base_xy: Vec2
    confidence: float
    camera_translation_m: Tuple[float, float, float]


class AprilTagCameraDetector:
    """Lazy OpenCV wrapper; import만으로 camera device를 요구하지 않는다."""

    def __init__(self, config: AprilTagCameraConfig):
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError("Install opencv-contrib-python-headless for AprilTag detection") from error
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, config.dictionary_name):
            raise RuntimeError("OpenCV ArUco AprilTag support is unavailable")
        self.cv2, self.config = cv2, config
        self.camera_matrix = np.asarray(config.camera_matrix, dtype=np.float64).reshape(3, 3)
        self.distortion = np.asarray(config.distortion_coefficients, dtype=np.float64).reshape(-1, 1)
        self.rotation = np.asarray(config.rotation_base_from_camera, dtype=np.float64).reshape(3, 3)
        self.translation = np.asarray(config.translation_base_from_camera_m, dtype=np.float64).reshape(3)
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.dictionary_name))
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    @staticmethod
    def _area(corners: np.ndarray) -> float:
        points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        return 0.5 * abs(float(np.dot(points[:, 0], np.roll(points[:, 1], -1))
                               - np.dot(points[:, 1], np.roll(points[:, 0], -1))))

    def _estimate_pose(self, corners: np.ndarray) -> Optional[np.ndarray]:
        """OpenCV 4 legacy ArUco API와 OpenCV 5 solvePnP를 모두 지원한다."""
        if hasattr(self.cv2.aruco, "estimatePoseSingleMarkers"):
            _, tvecs, _ = self.cv2.aruco.estimatePoseSingleMarkers(
                [corners], float(self.config.tag_size_m), self.camera_matrix, self.distortion
            )
            return np.asarray(tvecs[0, 0], dtype=np.float64)
        half = float(self.config.tag_size_m) * 0.5
        object_points = np.asarray(
            ((-half, half, 0.0), (half, half, 0.0), (half, -half, 0.0), (-half, -half, 0.0)),
            dtype=np.float64,
        )
        flag = getattr(self.cv2, "SOLVEPNP_IPPE_SQUARE", self.cv2.SOLVEPNP_ITERATIVE)
        success, _, tvec = self.cv2.solvePnP(
            object_points, np.asarray(corners, dtype=np.float64).reshape(4, 2),
            self.camera_matrix, self.distortion, flags=flag,
        )
        if not success:
            return None
        return np.asarray(tvec, dtype=np.float64).reshape(3)

    def detect_selected(self, image_bgr_or_gray: np.ndarray, selected_tag_id: int) -> Optional[AprilTagGroundObservation]:
        """요청한 Tag ID 하나만 반환하고, 모호하면 안전하게 거부한다."""
        corners, ids, _ = self.detector.detectMarkers(image_bgr_or_gray)
        if ids is None:
            return None
        matches = np.flatnonzero(ids.reshape(-1).astype(int) == int(selected_tag_id))
        if len(matches) != 1:
            return None
        selected = corners[int(matches[0])]
        area = self._area(selected)
        if area < self.config.min_area_px:
            return None
        camera_translation = self._estimate_pose(selected)
        if camera_translation is None:
            return None
        base_translation = self.rotation @ camera_translation + self.translation
        return AprilTagGroundObservation(
            tag_id=int(selected_tag_id),
            tag_base_xy=(float(base_translation[0]), float(base_translation[1])),
            confidence=float(min(1.0, area / max(4.0 * self.config.min_area_px, 1.0))),
            camera_translation_m=tuple(float(value) for value in camera_translation),
        )
