"""HSV vision reused from the verified OAK navigation approach.

Processing consumes supplied BGR frames only; camera acquisition and web
streaming remain separate so neither can block the navigation loop.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .config import HsvFilter, VisionConfig, Zone


@dataclass(frozen=True)
class VisionResult:
    timestamp_s: float
    target_x: float | None
    bud_in_trigger_zone: bool
    bud_in_pick_zone: bool
    marker_found: bool
    masks: dict[str, np.ndarray]
    overlay: np.ndarray


class VisionProcessor:
    def __init__(self) -> None:
        self._history: deque[float] = deque()

    @staticmethod
    def _mask(hsv: np.ndarray, hsv_filter: HsvFilter) -> np.ndarray:
        return cv2.inRange(hsv, np.asarray(hsv_filter.low, dtype=np.uint8),
                           np.asarray(hsv_filter.high, dtype=np.uint8))

    @staticmethod
    def _components(mask: np.ndarray, min_area: int) -> tuple[np.ndarray, list[tuple[float, float, float]]]:
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
        accepted = np.zeros_like(mask); parts = []
        for index in range(1, count):
            area = float(stats[index, cv2.CC_STAT_AREA])
            if area >= min_area:
                accepted[labels == index] = 255
                parts.append((float(centers[index][0]), float(centers[index][1]), area))
        return accepted, parts

    @staticmethod
    def _in_zone(mask: np.ndarray, zone: Zone) -> np.ndarray:
        height, width = mask.shape[:2]
        x0, x1, y0, y1 = zone.pixels(width, height)
        result = np.zeros_like(mask)
        result[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
        return result

    @staticmethod
    def _weighted_x(parts: list[tuple[float, float, float]]) -> float | None:
        area = sum(item[2] for item in parts)
        return None if area == 0 else sum(x * size for x, _y, size in parts) / area

    def process(self, frame: np.ndarray, timestamp_s: float, config: VisionConfig) -> VisionResult:
        config.validate()
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("BGR-frame med tre kanaler krävs")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        buds = self._in_zone(self._mask(hsv, config.buds), config.navigation_zone)
        leaves = self._in_zone(self._mask(hsv, config.leaves), config.navigation_zone)
        marker = self._in_zone(self._mask(hsv, config.marker), config.turn_marker_zone)
        buds, bud_parts = self._components(buds, config.buds.min_area)
        leaves, leaf_parts = self._components(leaves, config.leaves.min_area)
        marker, marker_parts = self._components(marker, config.marker.min_area)
        values = [self._weighted_x(bud_parts)]
        if config.navigation_mode == "buds_and_leaves": values.append(self._weighted_x(leaf_parts))
        values = [value for value in values if value is not None]
        raw_x = None if not values else sum(values) / len(values)
        self._history = deque(self._history, maxlen=config.x_filter_window_frames)
        if raw_x is not None and (config.x_outlier_threshold_px is None or not self._history
                                  or abs(raw_x - self._history[-1]) <= config.x_outlier_threshold_px):
            self._history.append(raw_x)
        target_x = None if raw_x is None or not self._history else sum(self._history) / len(self._history)
        trigger = self._in_zone(buds, config.trigger_zone)
        pick = self._in_zone(buds, config.pick_zone)
        overlay = self._overlay(frame, config, target_x, buds, leaves, marker)
        return VisionResult(timestamp_s, target_x, bool(cv2.countNonZero(trigger)),
                            bool(cv2.countNonZero(pick)), bool(marker_parts),
                            {"buds": buds, "leaves": leaves, "marker": marker}, overlay)

    @staticmethod
    def draw_zones(image: np.ndarray, config: VisionConfig) -> np.ndarray:
        """Return a copy with precisely the shared diagnostic zone lines."""
        image = image.copy(); height, width = image.shape[:2]
        for zone, colour in ((config.navigation_zone, (255, 180, 0)), (config.trigger_zone, (0, 255, 255)),
                             (config.pick_zone, (255, 0, 255)), (config.turn_marker_zone, (0, 180, 255))):
            x0, x1, y0, y1 = zone.pixels(width, height)
            cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), colour, 1)
        return image

    @staticmethod
    def draw_navigation_guides(image: np.ndarray, config: VisionConfig) -> np.ndarray:
        """Return camera evidence with the shared zones and configured x goal.

        This deliberately excludes segmentation and target annotations.  The
        Original dashboard view can therefore show its operational reference
        without pretending that a target was detected.
        """
        image = VisionProcessor.draw_zones(image, config)
        height, width = image.shape[:2]
        goal = round(config.x_goal * (width - 1))
        cv2.line(image, (goal, 0), (goal, height - 1), (0, 0, 255), 2)
        return image

    @staticmethod
    def _overlay(frame: np.ndarray, config: VisionConfig, target_x: float | None,
                 buds: np.ndarray, leaves: np.ndarray, marker: np.ndarray) -> np.ndarray:
        image = frame.copy(); height, width = frame.shape[:2]
        image[leaves == 255] = (0, 220, 0); image[buds == 255] = (230, 0, 230); image[marker == 255] = (0, 180, 255)
        image = VisionProcessor.draw_navigation_guides(image, config)
        if target_x is not None:
            cv2.line(image, (round(target_x), 0), (round(target_x), height - 1), (255, 255, 0), 1)
        return image
