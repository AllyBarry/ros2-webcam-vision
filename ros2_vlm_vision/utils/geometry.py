"""Pinhole projection, depth sampling, and world-frame back-projection helpers."""
from __future__ import annotations

from typing import Optional

import numpy as np


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """[x, y, z, w] quaternion -> 3x3 rotation matrix."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
    xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ])


def ray_plane_intersect_z(origin_world: np.ndarray,
                          direction_world: np.ndarray,
                          plane_z: float) -> Optional[np.ndarray]:
    """Intersect a ray with the world-frame plane z = plane_z.

    Returns world-frame [X, Y, Z] of the intersection, or None if the
    ray is parallel to the plane (no intersection) or if the
    intersection sits behind the origin (s <= 0, i.e. the object would
    have to be behind the camera to lie on the table).
    """
    dz = float(direction_world[2])
    if abs(dz) < 1e-9:
        return None
    s = (plane_z - float(origin_world[2])) / dz
    if s <= 0.0:
        return None
    return origin_world + s * direction_world


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

    def ray_direction(self, u: float, v: float) -> np.ndarray:
        """Unit-length-z ray direction in the camera frame through pixel (u, v).

        Useful for back-projection into a world-frame plane: rotate this
        into world frame using the calibrated R_world_camera, then call
        ray_plane_intersect_z.
        """
        return np.array([(u - self.cx) / self.fx,
                         (v - self.cy) / self.fy,
                         1.0])


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
