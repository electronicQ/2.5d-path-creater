"""Continuous grayscale map semantics for trajectory planning."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MapSemantics:
    unknown_gray: int = 205
    absolute_obstacle_gray: int = 90
    gray_cost_weight: float = 1.0


@dataclass(frozen=True)
class SemanticLayers:
    forbidden: np.ndarray
    free: np.ndarray
    gray_cost: np.ndarray


def build_semantic_layers(gray: np.ndarray, semantics: MapSemantics) -> SemanticLayers:
    gray_u8 = np.asarray(gray, dtype=np.uint8)
    unknown = gray_u8 == int(semantics.unknown_gray)
    obstacle = gray_u8 <= int(semantics.absolute_obstacle_gray)
    forbidden = unknown | obstacle
    free = ~forbidden
    normalized = 1.0 - gray_u8.astype(np.float32) / 255.0
    gray_cost = normalized * float(semantics.gray_cost_weight)
    gray_cost[forbidden] = 0.0
    return SemanticLayers(forbidden=forbidden, free=free, gray_cost=gray_cost.astype(np.float32))
