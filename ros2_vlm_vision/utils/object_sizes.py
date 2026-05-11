"""Canonical object heights (metres) used to back out distance from a
single RGB frame.

Heuristic: assumes the object is roughly upright in the image and that the
bbox vertical extent maps to the object's real-world height. Z is then
solved from the pinhole relation  bbox_h_px / fy = real_h / Z.

This is intentionally coarse. Errors come from (a) atypical pose --- a
person lying down, a knocked-over bottle --- and (b) intra-class size
variance (a sedan vs. an SUV). For a more accurate estimate, swap this
out for a monocular-depth network downstream.

Sizes are medians for an adult or full-size instance, sourced from
manufacturer specs / Wikipedia for COCO's 80 classes. Tune for your domain
by editing this table.
"""
from __future__ import annotations

COCO_HEIGHTS_M: dict[str, float] = {
    'person': 1.70,
    'bicycle': 1.10,
    'car': 1.50,
    'motorcycle': 1.20,
    'airplane': 4.00,
    'bus': 3.20,
    'train': 3.80,
    'truck': 2.80,
    'boat': 1.50,
    'traffic light': 0.70,
    'fire hydrant': 0.80,
    'stop sign': 0.75,
    'parking meter': 1.20,
    'bench': 0.85,
    'bird': 0.20,
    'cat': 0.25,
    'dog': 0.50,
    'horse': 1.60,
    'sheep': 0.90,
    'cow': 1.40,
    'elephant': 3.00,
    'bear': 1.50,
    'zebra': 1.40,
    'giraffe': 5.00,
    'backpack': 0.50,
    'umbrella': 1.00,
    'handbag': 0.30,
    'tie': 0.40,
    'suitcase': 0.65,
    'frisbee': 0.27,
    'skis': 1.60,
    'snowboard': 1.55,
    'sports ball': 0.22,
    'kite': 0.60,
    'baseball bat': 0.85,
    'baseball glove': 0.30,
    'skateboard': 0.80,
    'surfboard': 1.80,
    'tennis racket': 0.70,
    'bottle': 0.25,
    'wine glass': 0.22,
    'cup': 0.10,
    'fork': 0.20,
    'knife': 0.25,
    'spoon': 0.18,
    'bowl': 0.08,
    'banana': 0.18,
    'apple': 0.08,
    'sandwich': 0.10,
    'orange': 0.08,
    'broccoli': 0.15,
    'carrot': 0.20,
    'hot dog': 0.05,
    'pizza': 0.30,
    'donut': 0.10,
    'cake': 0.15,
    'chair': 0.90,
    'couch': 0.85,
    'potted plant': 0.40,
    'bed': 0.60,
    'dining table': 0.75,
    'toilet': 0.75,
    'tv': 0.55,
    'laptop': 0.25,
    'mouse': 0.04,
    'remote': 0.18,
    'keyboard': 0.03,
    'cell phone': 0.15,
    'microwave': 0.30,
    'oven': 0.70,
    'toaster': 0.20,
    'sink': 0.25,
    'refrigerator': 1.70,
    'book': 0.22,
    'clock': 0.30,
    'vase': 0.25,
    'scissors': 0.18,
    'teddy bear': 0.30,
    'hair drier': 0.20,
    'toothbrush': 0.18,
}


def distance_from_height(class_name: str, bbox_height_px: float,
                         fy: float) -> float | None:
    """Solve Z = real_h * fy / bbox_h_px. Returns None if class is unknown."""
    real_h = COCO_HEIGHTS_M.get(class_name)
    if real_h is None or bbox_height_px <= 0.0 or fy <= 0.0:
        return None
    return real_h * fy / bbox_height_px
