"""HSV vision reused from the verified OAK navigation approach.

Processing consumes supplied BGR frames only; camera acquisition and web
streaming remain separate so neither can block the navigation loop.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .config import HsvFilter, TrapezoidZone, VisionConfig, Zone


@dataclass(frozen=True)
class VisionResult:
    timestamp_s: float
    target_x: float | None
    target_y: float | None
    bud_in_trigger_zone: bool
    bud_in_pick_zone: bool
    marker_found: bool
    masks: dict[str, np.ndarray]
    overlay: np.ndarray
    raw_frame: np.ndarray
    # The legacy target is always the selected master.  Per-row values make
    # the selection observable and prevent callers from inferring it from x.
    master_row: int | None = None
    row_1_target_x: float | None = None
    row_1_target_y: float | None = None
    row_2_target_x: float | None = None
    row_2_target_y: float | None = None


class VisionProcessor:
    def __init__(self) -> None:
        self._history: dict[int, deque[float]] = {1: deque(), 2: deque()}

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
    def _zone_polygon(zone: Zone | TrapezoidZone, width: int, height: int) -> np.ndarray:
        if isinstance(zone, Zone):
            x0, x1, y0, y1 = zone.pixels(width, height)
            points = ((x0, y0), (x1 - 1, y0), (x1 - 1, y1 - 1), (x0, y1 - 1))
        else:
            points = zone.pixels(width, height)
        return np.asarray([(min(width - 1, max(0, x)), min(height - 1, max(0, y)))
                           for x, y in points], dtype=np.int32)

    @classmethod
    def _in_zone(cls, mask: np.ndarray, zone: Zone | TrapezoidZone) -> np.ndarray:
        height, width = mask.shape[:2]
        result = np.zeros_like(mask)
        polygon_mask = np.zeros_like(mask)
        cv2.fillConvexPoly(polygon_mask, cls._zone_polygon(zone, width, height), 255)
        result[polygon_mask == 255] = mask[polygon_mask == 255]
        return result

    @staticmethod
    def _weighted_x(parts: list[tuple[float, float, float]]) -> float | None:
        area = sum(item[2] for item in parts)
        return None if area == 0 else sum(x * size for x, _y, size in parts) / area

    @staticmethod
    def _weighted_y(parts: list[tuple[float, float, float]]) -> float | None:
        area = sum(item[2] for item in parts)
        return None if area == 0 else sum(y * size for _x, y, size in parts) / area

    def process(self, frame: np.ndarray, timestamp_s: float, config: VisionConfig) -> VisionResult:
        config.validate()
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("BGR-frame med tre kanaler krävs")
        frame_height, frame_width = frame.shape[:2]
        x0, x1, y0, y1 = config.first_crop.pixels(frame_width, frame_height)
        if x1 <= x0 or y1 <= y0:
            raise ValueError("first_crop ger en tom arbetsbild vid aktuell upplösning")
        # Crop before HSV conversion so every remaining operation, including
        # zone coordinates and the dashboard's Original image, shares one
        # processed coordinate system.
        frame = frame[y0:y1, x0:x1].copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        navigation_zone_1 = config.effective_zone(config.navigation_zone, 1)
        navigation_zone_2 = config.effective_zone(config.navigation_zone_2, 2)
        trigger_zone_1 = config.effective_zone(config.trigger_zone, 1)
        trigger_zone_2 = config.effective_zone(config.trigger_zone_2, 2)
        pick_zone_1 = config.effective_zone(config.pick_zone, 1)
        pick_zone_2 = config.effective_zone(config.pick_zone_2, 2)
        marker_zone = config.effective_zone(config.turn_marker_zone)
        bud_mask = self._mask(hsv, config.buds)
        leaf_mask = self._mask(hsv, config.leaves)
        buds_1 = self._in_zone(bud_mask, navigation_zone_1)
        buds_2 = self._in_zone(bud_mask, navigation_zone_2)
        leaves_1 = self._in_zone(leaf_mask, navigation_zone_1)
        leaves_2 = self._in_zone(leaf_mask, navigation_zone_2)
        marker = self._in_zone(self._mask(hsv, config.marker), marker_zone)
        buds_1, bud_parts_1 = self._components(buds_1, config.buds.min_area)
        buds_2, bud_parts_2 = self._components(buds_2, config.buds.min_area)
        leaves_1, leaf_parts_1 = self._components(leaves_1, config.leaves.min_area)
        leaves_2, leaf_parts_2 = self._components(leaves_2, config.leaves.min_area)
        marker, marker_parts = self._components(marker, config.marker.min_area)
        def target_for(row: int, bud_parts: list[tuple[float, float, float]],
                       leaf_parts: list[tuple[float, float, float]]) -> tuple[float | None, float | None]:
            parts_for_target = [bud_parts]
            if config.navigation_mode == "buds_and_leaves":
                parts_for_target.append(leaf_parts)
            values_x = [value for value in (self._weighted_x(parts) for parts in parts_for_target) if value is not None]
            values_y = [value for value in (self._weighted_y(parts) for parts in parts_for_target) if value is not None]
            raw_x = None if not values_x else sum(values_x) / len(values_x)
            target_y = None if not values_y else sum(values_y) / len(values_y)
            history = deque(self._history[row], maxlen=config.x_filter_window_frames)
            if raw_x is not None and (config.x_outlier_threshold_px is None or not history
                                      or abs(raw_x - history[-1]) <= config.x_outlier_threshold_px):
                history.append(raw_x)
            self._history[row] = history
            return (None if raw_x is None or not history else sum(history) / len(history), target_y)

        row_1_x, row_1_y = target_for(1, bud_parts_1, leaf_parts_1)
        row_2_x, row_2_y = target_for(2, bud_parts_2, leaf_parts_2)
        master_row = 1 if row_1_x is not None else 2 if row_2_x is not None else None
        target_x, target_y = ((row_1_x, row_1_y) if master_row == 1 else
                              (row_2_x, row_2_y) if master_row == 2 else (None, None))
        trigger_1 = self._in_zone(bud_mask, trigger_zone_1)
        trigger_2 = self._in_zone(bud_mask, trigger_zone_2)
        pick_1 = self._in_zone(bud_mask, pick_zone_1)
        pick_2 = self._in_zone(bud_mask, pick_zone_2)
        display_buds = cv2.bitwise_or(buds_1, buds_2)
        display_leaves = cv2.bitwise_or(leaves_1, leaves_2)
        overlay = self._overlay(frame, config, target_x, display_buds, display_leaves, marker)
        return VisionResult(timestamp_s, target_x, target_y,
                            bool(cv2.countNonZero(trigger_1) or cv2.countNonZero(trigger_2)),
                            bool(cv2.countNonZero(pick_1) or cv2.countNonZero(pick_2)), bool(marker_parts),
                            {"buds": display_buds, "leaves": display_leaves, "marker": marker}, overlay, frame,
                            master_row, row_1_x, row_1_y, row_2_x, row_2_y)

    @staticmethod
    def draw_zones(image: np.ndarray, config: VisionConfig) -> np.ndarray:
        """Return a copy with precisely the shared diagnostic zone lines."""
        image = image.copy(); height, width = image.shape[:2]
        # Row 1 is the established palette; row 2 uses lighter/darker
        # companions so the two independent regions remain distinguishable.
        zones = ((config.navigation_zone, 1, (255, 180, 0)), (config.trigger_zone, 1, (0, 255, 255)),
                 (config.pick_zone, 1, (255, 0, 255)), (config.navigation_zone_2, 2, (255, 255, 0)),
                 (config.trigger_zone_2, 2, (0, 180, 180)), (config.pick_zone_2, 2, (180, 0, 255)),
                 (config.turn_marker_zone, 1, (0, 180, 255)))
        for configured_zone, row, colour in zones:
            zone = config.effective_zone(configured_zone, row)
            if isinstance(zone, Zone):
                x0, x1, y0, y1 = zone.pixels(width, height)
                cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), colour, 1)
            else:
                cv2.polylines(image, [VisionProcessor._zone_polygon(zone, width, height)], True, colour, 1)
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
        # Draw row 2 first: row 1 retains its established red guide at a
        # crossing, which also makes master priority visually unsurprising.
        for row, colour in ((2, (0, 80, 255)), (1, (0, 0, 255))):
            goal_top = round(config.goal_x_normalized(0, height, row) * (width - 1))
            goal_bottom = round(config.goal_x_normalized(height - 1, height, row) * (width - 1))
            cv2.line(image, (goal_top, 0), (goal_bottom, height - 1), colour, 2)
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
