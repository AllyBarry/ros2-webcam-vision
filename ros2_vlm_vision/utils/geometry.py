"""Pinhole projection and depth-sampling helpers."""
from __future__ import annotations

import numpy as np


class PinholeModel:
    """Minimal pinhole camera model built from a sensor_msgs/CameraInfo K matrix."""

    __slots__ = ('fx', 'fy', 'cx', 'cy', 'width', 'height')

    def __init__(self, fx: float, fy: float, cx: float, cy: float,
                 width: int, height: int):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.width = width
        self.height = height

    @classmethod
    def from_camera_info(cls, info) -> 'PinholeModel':
        k = info.k
        return cls(fx=k[0], fy=k[4], cx=k[2], cy=k[5],
                   width=info.width, height=info.height)

    def deproject(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        """Back-project a pixel (u, v) with depth z (metres) into the camera frame."""
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z


def sample_depth(depth: np.ndarray, u: int, v: int,
                 window: int = 5, depth_scale: float = 1.0) -> float | None:
    """Return the median valid depth (metres) in a window around (u, v).

    Rejects zero / NaN / inf samples. Returns None if no valid pixels remain.
    """
    h, w = depth.shape[:2]
    half = window // 2
    u0 = max(0, u - half)
    u1 = min(w, u + half + 1)
    v0 = max(0, v - half)
    v1 = min(h, v + half + 1)
    patch = depth[v0:v1, u0:u1].astype(np.float32) * depth_scale
    mask = np.isfinite(patch) & (patch > 0.0)
    if not mask.any():
        return None
    return float(np.median(patch[mask]))
