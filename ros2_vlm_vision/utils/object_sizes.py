"""Per-class real-world dimensions for 3D estimation.

Loaded from config/object_dimensions.yaml. Two consumers:

  - distance_from_height(class_name, bbox_h_px, fy): the legacy
    monocular-depth heuristic used when no extrinsic / table plane is
    available. Solves Z = real_h * fy / bbox_h_px.
  - get_dimensions(class_name): returns the full ObjectDimensions
    (height / width / depth / shape) for the world-frame back-projection
    path -- lifts centroid above the table and populates BoundingBox3D.

When a class isn't in the YAML, the loader returns the `defaults`
block's values. Tune by editing the YAML, not this module.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


@dataclass(frozen=True)
class ObjectDimensions:
    height: float       # vertical extent in metres
    width: float        # horizontal extent (axis A)
    depth: float        # horizontal extent (axis B)
    shape: str          # "box" | "cylinder" | "sphere"
    known: bool         # True if class was in the YAML; False = defaults used


class DimensionTable:
    """Per-class dimension lookup loaded from a YAML config file."""

    def __init__(self, yaml_path: str | Path) -> None:
        path = Path(yaml_path)
        with path.open() as f:
            data = yaml.safe_load(f) or {}
        d = data.get('defaults', {}) or {}
        self._defaults = ObjectDimensions(
            height=float(d.get('height', 0.05)),
            width=float(d.get('width', 0.05)),
            depth=float(d.get('depth', d.get('width', 0.05))),
            shape=str(d.get('shape', 'box')),
            known=False,
        )

        raw_classes = data.get('classes', {}) or {}
        table: dict[str, ObjectDimensions] = {}
        for name, fields in raw_classes.items():
            if not isinstance(fields, dict):
                continue
            h = float(fields.get('height', self._defaults.height))
            w = float(fields.get('width', self._defaults.width))
            de = float(fields.get('depth', w))
            sh = str(fields.get('shape', self._defaults.shape))
            table[str(name).lower()] = ObjectDimensions(
                height=h, width=w, depth=de, shape=sh, known=True,
            )
        self._table = table

    def get(self, class_name: str) -> ObjectDimensions:
        return self._table.get(class_name.lower(), self._defaults)

    def has(self, class_name: str) -> bool:
        return class_name.lower() in self._table

    def __len__(self) -> int:
        return len(self._table)


# ---------------------------------------------------------------------
# Legacy fallback (no extrinsic / no table plane)
# ---------------------------------------------------------------------
def distance_from_height(class_name: str, bbox_height_px: float,
                         fy: float, dims: DimensionTable) -> Optional[float]:
    """Solve Z = real_h * fy / bbox_h_px from the class's stored height.

    Returns None if the inputs are degenerate (zero pixels / focal). When
    the class isn't in the YAML, falls through to the defaults' height
    -- imprecise but always defined.
    """
    if bbox_height_px <= 0.0 or fy <= 0.0:
        return None
    real_h = dims.get(class_name).height
    return real_h * fy / bbox_height_px
