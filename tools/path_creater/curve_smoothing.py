"""Safety-aware curve smoothing and fixed-interval sampling."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Sequence, Tuple


Point = Tuple[float, float]
SegmentValidator = Callable[[Point, Point], bool]


@dataclass(frozen=True)
class CurveSmoothingConfig:
    enabled: bool = True
    sample_interval_m: float = 1.0
    corner_cut_fraction: float = 0.35
    max_corner_cut_m: float = 8.0
    min_corner_cut_m: float = 0.2
    safety_check_step_m: float = 0.25
    max_corner_attempts: int = 8


def _clean(value: float) -> float:
    if abs(value) < 1e-12:
        return 0.0
    return round(float(value), 12)


def _clean_point(point: Point) -> Point:
    return (_clean(point[0]), _clean(point[1]))


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _lerp(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _unit(a: Point, b: Point) -> Point:
    d = _distance(a, b)
    if d <= 0.0:
        return (0.0, 0.0)
    return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)


def _quadratic_bezier(a: Point, control: Point, b: Point, t: float) -> Point:
    mt = 1.0 - t
    return (
        mt * mt * a[0] + 2.0 * mt * t * control[0] + t * t * b[0],
        mt * mt * a[1] + 2.0 * mt * t * control[1] + t * t * b[1],
    )


def _append_unique(points: List[Point], point: Point) -> None:
    point = _clean_point(point)
    if not points or _distance(points[-1], point) > 1e-9:
        points.append(point)


def _segments_are_safe(points: Sequence[Point], is_segment_free: SegmentValidator) -> bool:
    return all(is_segment_free(a, b) for a, b in zip(points, points[1:]))


def _sample_bezier(a: Point, control: Point, b: Point, step_m: float) -> List[Point]:
    approx_len = _distance(a, control) + _distance(control, b)
    steps = max(2, int(math.ceil(approx_len / max(step_m, 1e-6))))
    return [_clean_point(_quadratic_bezier(a, control, b, i / float(steps))) for i in range(steps + 1)]


def _rounded_corner(
    prev_point: Point,
    corner: Point,
    next_point: Point,
    is_segment_free: SegmentValidator,
    cfg: CurveSmoothingConfig,
) -> List[Point]:
    prev_len = _distance(prev_point, corner)
    next_len = _distance(corner, next_point)
    if prev_len <= 0.0 or next_len <= 0.0:
        return [corner]

    cut = min(
        float(cfg.max_corner_cut_m),
        float(cfg.corner_cut_fraction) * prev_len,
        float(cfg.corner_cut_fraction) * next_len,
    )
    if cut < float(cfg.min_corner_cut_m):
        return [corner]

    from_prev = _unit(corner, prev_point)
    to_next = _unit(corner, next_point)
    for _ in range(max(1, int(cfg.max_corner_attempts))):
        entry = (corner[0] + from_prev[0] * cut, corner[1] + from_prev[1] * cut)
        exit_point = (corner[0] + to_next[0] * cut, corner[1] + to_next[1] * cut)
        candidate = _sample_bezier(entry, corner, exit_point, float(cfg.safety_check_step_m))
        if _segments_are_safe(candidate, is_segment_free):
            return candidate
        cut *= 0.5
        if cut < float(cfg.min_corner_cut_m):
            break
    return [corner]


def round_corners(points: Sequence[Point], is_segment_free: SegmentValidator, cfg: CurveSmoothingConfig) -> List[Point]:
    if len(points) <= 2 or not cfg.enabled:
        return [_clean_point(p) for p in points]

    rounded: List[Point] = []
    _append_unique(rounded, points[0])
    for i in range(1, len(points) - 1):
        corner_points = _rounded_corner(points[i - 1], points[i], points[i + 1], is_segment_free, cfg)
        for point in corner_points:
            _append_unique(rounded, point)
    _append_unique(rounded, points[-1])
    return rounded


def resample_by_interval(points: Sequence[Point], interval_m: float) -> List[Point]:
    if len(points) == 0:
        return []
    if len(points) == 1:
        return [_clean_point(points[0])]

    interval = float(interval_m)
    if interval <= 0.0:
        raise ValueError("sample_interval_m must be positive.")

    cumulative = [0.0]
    cleaned = [_clean_point(p) for p in points]
    for a, b in zip(cleaned, cleaned[1:]):
        cumulative.append(cumulative[-1] + _distance(a, b))

    total = cumulative[-1]
    if total <= 0.0:
        return [cleaned[0]]

    targets: List[float] = []
    d = 0.0
    while d < total:
        targets.append(d)
        d += interval
    if not math.isclose(targets[-1], total, abs_tol=1e-9):
        targets.append(total)

    sampled: List[Point] = []
    seg = 0
    for target in targets:
        while seg + 1 < len(cumulative) and cumulative[seg + 1] < target:
            seg += 1
        if seg + 1 >= len(cumulative):
            point = cleaned[-1]
        else:
            span = cumulative[seg + 1] - cumulative[seg]
            t = 0.0 if span <= 0.0 else (target - cumulative[seg]) / span
            point = _lerp(cleaned[seg], cleaned[seg + 1], t)
        _append_unique(sampled, point)
    return sampled


def smooth_and_sample_path(
    points: Sequence[Point],
    is_segment_free: SegmentValidator,
    cfg: CurveSmoothingConfig,
) -> List[Point]:
    if len(points) <= 1:
        return [_clean_point(p) for p in points]

    base = [_clean_point(p) for p in points]
    curve = round_corners(base, is_segment_free, cfg)
    sampled = resample_by_interval(curve, float(cfg.sample_interval_m))
    if _segments_are_safe(sampled, is_segment_free):
        return sampled
    return resample_by_interval(base, float(cfg.sample_interval_m))
