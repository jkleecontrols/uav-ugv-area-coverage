"""Geometric, grid-sampled metrics for FOV-aware area surveillance.

All planners and baselines use this module.  A cell is covered when its centre
is inside the AOI and inside an observation footprint.  Redundancy is the
fraction of all valid cell-observations that are repeat observations:
``(sum(counts) - number_of_covered_cells) / sum(counts)``.  This explicit
definition replaces the undocumented legacy metric.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
from shapely import covers, points
from shapely.geometry import Polygon


@dataclass(frozen=True)
class FOV:
    """Rectangular ground footprint dimensions in metres."""

    forward_m: float = 100.0
    cross_track_m: float = 67.0


@dataclass(frozen=True)
class Observation:
    """One camera exposure, with heading measured counter-clockwise from +x."""

    x_m: float
    y_m: float
    heading_rad: float


@dataclass(frozen=True)
class CoverageMetrics:
    """Metrics and counts needed to audit a coverage result."""

    coverage_percent: float
    redundancy_percent: float
    aoi_cells: int
    covered_cells: int
    cell_observations: int
    grid_resolution_m: float


def footprint_polygon(observation: Observation, fov: FOV) -> Polygon:
    """Return the rectangular footprint for one observation."""

    forward = fov.forward_m / 2.0
    cross_track = fov.cross_track_m / 2.0
    cos_heading = float(np.cos(observation.heading_rad))
    sin_heading = float(np.sin(observation.heading_rad))

    # Local coordinates use +x as the camera forward direction.
    local_corners = ((forward, cross_track), (forward, -cross_track),
                     (-forward, -cross_track), (-forward, cross_track))
    corners = [
        (
            observation.x_m + longitudinal * cos_heading - lateral * sin_heading,
            observation.y_m + longitudinal * sin_heading + lateral * cos_heading,
        )
        for longitudinal, lateral in local_corners
    ]
    return Polygon(corners)


def _cell_centres(aoi: Polygon, resolution_m: float) -> tuple[np.ndarray, np.ndarray]:
    if resolution_m <= 0:
        raise ValueError("grid resolution must be positive")
    if aoi.is_empty or aoi.area <= 0:
        raise ValueError("AOI must be a non-empty polygon with positive area")

    min_x, min_y, max_x, max_y = aoi.bounds
    xs = np.arange(min_x + resolution_m / 2.0, max_x, resolution_m)
    ys = np.arange(min_y + resolution_m / 2.0, max_y, resolution_m)
    grid_x, grid_y = np.meshgrid(xs, ys)
    return grid_x.ravel(), grid_y.ravel()


def evaluate_coverage(
    aoi: Polygon,
    observations: Sequence[Observation] | Iterable[Observation],
    fov: FOV = FOV(),
    grid_resolution_m: float = 1.0,
) -> CoverageMetrics:
    """Evaluate unique coverage and repeat-observation redundancy over an AOI.

    The returned cell counts are deliberately retained in the result so every
    reported percentage can be independently recomputed from saved artifacts.
    """

    observations = tuple(observations)
    xs, ys = _cell_centres(aoi, grid_resolution_m)
    candidates = points(xs, ys)
    in_aoi = np.asarray(covers(aoi, candidates), dtype=bool)
    aoi_cells = int(in_aoi.sum())
    if aoi_cells == 0:
        raise ValueError("grid resolution produced no AOI cell centres")

    counts = np.zeros(xs.size, dtype=np.int32)
    for observation in observations:
        seen = np.asarray(covers(footprint_polygon(observation, fov), candidates), dtype=bool)
        counts += (seen & in_aoi).astype(np.int32)

    covered_cells = int(np.count_nonzero(counts))
    cell_observations = int(counts.sum())
    coverage_percent = 100.0 * covered_cells / aoi_cells
    redundancy_percent = (
        0.0
        if cell_observations == 0
        else 100.0 * (cell_observations - covered_cells) / cell_observations
    )
    return CoverageMetrics(
        coverage_percent=coverage_percent,
        redundancy_percent=redundancy_percent,
        aoi_cells=aoi_cells,
        covered_cells=covered_cells,
        cell_observations=cell_observations,
        grid_resolution_m=grid_resolution_m,
    )
