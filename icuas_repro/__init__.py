"""Maintained, reproducible implementation of the ICUAS 2025 study.

The legacy ``TOP/`` and ``map/`` directories are retained as historical
reference.  New experiments must use this package and write self-describing
artifacts.
"""

from .coverage import CoverageMetrics, FOV, Observation, evaluate_coverage
from .scheduling import PeriodicSchedule, solve_max_flight_schedule
from .route import PolylineRoute
from .simulation import SortieEvent, sortie_events

__all__ = [
    "CoverageMetrics", "FOV", "Observation", "evaluate_coverage",
    "PeriodicSchedule", "solve_max_flight_schedule",
    "PolylineRoute",
    "SortieEvent", "sortie_events",
]
