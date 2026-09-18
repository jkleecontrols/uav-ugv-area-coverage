"""Road-constrained UGV motion on an explicit polyline."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


Point = tuple[float, float]


@dataclass(frozen=True)
class PolylineRoute:
    """A traversable UGV route in metric coordinates.

    ``position_at_distance`` wraps by default, implementing the fixed cyclic
    route stated in the ICUAS paper.  A scenario records the exact ordered
    vertices, so no off-road interpolation can be introduced silently.
    """

    vertices: tuple[Point, ...]

    def __post_init__(self):
        if len(self.vertices) < 2:
            raise ValueError("a route needs at least two vertices")
        arr = np.asarray(self.vertices, dtype=float)
        if not np.isfinite(arr).all():
            raise ValueError("route coordinates must be finite")
        lengths = np.linalg.norm(arr[1:] - arr[:-1], axis=1)
        if np.any(lengths <= 0):
            raise ValueError("route has a zero-length segment")
        object.__setattr__(self, "_lengths", lengths)
        object.__setattr__(self, "_cumulative", np.concatenate(([0.0], np.cumsum(lengths))))

    @property
    def length_m(self) -> float:
        return float(self._cumulative[-1])

    def position_at_distance(self, distance_m: float, loop: bool = True) -> Point:
        if distance_m < 0:
            raise ValueError("distance must be non-negative")
        if loop:
            if not np.allclose(self.vertices[0], self.vertices[-1], atol=1e-9, rtol=0):
                raise ValueError("a looping route must be explicitly closed")
            distance_m %= self.length_m
        elif distance_m > self.length_m:
            distance_m = self.length_m
        segment = int(np.searchsorted(self._cumulative, distance_m, side="right") - 1)
        segment = min(segment, len(self.vertices) - 2)
        fraction = (distance_m - self._cumulative[segment]) / self._lengths[segment]
        a, b = self.vertices[segment], self.vertices[segment + 1]
        return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))

    def position_at_time(self, time_s: float, speed_mps: float, loop: bool = True) -> Point:
        if time_s < 0 or speed_mps <= 0:
            raise ValueError("time must be non-negative and speed must be positive")
        return self.position_at_distance(time_s * speed_mps, loop=loop)
