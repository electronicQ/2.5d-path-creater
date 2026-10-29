"""Safety-aware curve smoothing and shape-adaptive tracking-point sampling."""

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


@dataclass(frozen=True)
class TrackingSamplingConfig:
    enabled: bool = True
    max_spacing_m: float = 10.0
    min_spacing_m: float = 1.0
    chord_tolerance_m: float = 0.15
    max_heading_change_deg: float = 3.0


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


def _point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-18:
        return _distance(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    projection = min(1.0, max(0.0, projection))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    return _distance(point, closest)


def _heading(a: Point, b: Point) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _angle_difference(first: float, second: float) -> float:
    return abs((second - first + math.pi) % (2.0 * math.pi) - math.pi)


def adaptive_resample_path(points: Sequence[Point], cfg: TrackingSamplingConfig) -> List[Point]:
    """Select sparse tracking points while bounding straight-line approximation error."""
    min_spacing = float(cfg.min_spacing_m)
    max_spacing = float(cfg.max_spacing_m)
    chord_tolerance = float(cfg.chord_tolerance_m)
    max_turn = math.radians(float(cfg.max_heading_change_deg))
    if min_spacing <= 0.0:
        raise ValueError("min_spacing_m must be positive.")
    if max_spacing < min_spacing:
        raise ValueError("max_spacing_m must be greater than or equal to min_spacing_m.")
    if chord_tolerance <= 0.0:
        raise ValueError("chord_tolerance_m must be positive.")
    if not 0.0 < float(cfg.max_heading_change_deg) <= 180.0:
        raise ValueError("max_heading_change_deg must be in (0, 180].")
    if len(points) <= 1:
        return [_clean_point(point) for point in points]
    if not cfg.enabled:
        return [_clean_point(point) for point in points]

    fine = resample_by_interval(points, min_spacing)
    if len(fine) <= 2:
        return fine

    cumulative = [0.0]
    headings: List[float] = []
    for first, second in zip(fine, fine[1:]):
        segment_length = _distance(first, second)
        cumulative.append(cumulative[-1] + segment_length)
        headings.append(_heading(first, second))

    selected = [fine[0]]
    anchor = 0
    while anchor < len(fine) - 1:
        best = anchor + 1
        accumulated_turn = 0.0
        for candidate in range(anchor + 1, len(fine)):
            if cumulative[candidate] - cumulative[anchor] > max_spacing + 1e-9:
                break
            if candidate > anchor + 1:
                accumulated_turn += _angle_difference(headings[candidate - 2], headings[candidate - 1])
                if accumulated_turn > max_turn + 1e-12:
                    break
            max_deviation = max(
                _point_to_segment_distance(fine[index], fine[anchor], fine[candidate])
                for index in range(anchor + 1, candidate)
            ) if candidate > anchor + 1 else 0.0
            if max_deviation > chord_tolerance + 1e-12:
                break
            best = candidate
        _append_unique(selected, fine[best])
        anchor = best

    _append_unique(selected, fine[-1])
    return selected


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


def smooth_and_adaptive_sample_path(
    points: Sequence[Point],
    is_segment_free: SegmentValidator,
    curve_cfg: CurveSmoothingConfig,
    sampling_cfg: TrackingSamplingConfig,
) -> List[Point]:
    if len(points) <= 1:
        return [_clean_point(point) for point in points]
    if not sampling_cfg.enabled:
        return smooth_and_sample_path(points, is_segment_free, curve_cfg)

    base = [_clean_point(point) for point in points]
    curve = round_corners(base, is_segment_free, curve_cfg)
    sampled = adaptive_resample_path(curve, sampling_cfg)
    if _segments_are_safe(sampled, is_segment_free):
        return sampled
    return adaptive_resample_path(base, sampling_cfg)


def ensure_path_endpoints(points: Sequence[Point], start: Point, end: Point) -> List[Point]:
    """Ensure every generated tracking path contains its exact start and end."""
    result = [_clean_point(point) for point in points]
    clean_start = _clean_point(start)
    clean_end = _clean_point(end)
    if not result:
        return [clean_start, clean_end] if _distance(clean_start, clean_end) > 1e-9 else [clean_start]
    if _distance(result[0], clean_start) > 1e-9:
        result.insert(0, clean_start)
    if _distance(result[-1], clean_end) > 1e-9:
        result.append(clean_end)
    return result
