"""Deterministic temporal backbone for fixed-route UAV--UGV missions."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Iterable

from .route import Point, PolylineRoute
from .scheduling import PeriodicSchedule


@dataclass(frozen=True)
class SortieEvent:
    uav_id: int
    takeoff_s: int
    landing_s: int
    launch_point_m: Point
    rendezvous_point_m: Point

    def as_dict(self) -> dict:
        return asdict(self)


def sortie_events(
    schedule: PeriodicSchedule,
    route: PolylineRoute,
    ugv_speed_mps: float,
    horizon_s: int,
) -> list[SortieEvent]:
    """Generate all complete scheduled sorties in a mission horizon.

    The ILP phase is a charging start.  Each following flight interval starts
    after the UAV's charging block and lasts exactly its configured flight
    duration.  There is no hidden queue or delay: pad feasibility is already
    guaranteed by the schedule ILP and is auditable separately.
    """

    if horizon_s <= 0:
        raise ValueError("horizon_s must be positive")
    events: list[SortieEvent] = []
    for uav_id in schedule.selected_uavs:
        cycle_s = schedule.slot_seconds * (schedule.charge_slots[uav_id] + schedule.flight_slots[uav_id])
        charge_s = schedule.slot_seconds * schedule.charge_slots[uav_id]
        flight_s = schedule.slot_seconds * schedule.flight_slots[uav_id]
        first_charge_start = schedule.charge_start_slot[uav_id] * schedule.slot_seconds
        # Include the previous cycle because its flight can begin at t >= 0.
        cycle_index = -1
        while True:
            takeoff_s = first_charge_start + cycle_index * cycle_s + charge_s
            landing_s = takeoff_s + flight_s
            if takeoff_s >= horizon_s:
                break
            if takeoff_s >= 0 and landing_s <= horizon_s:
                events.append(SortieEvent(
                    uav_id=uav_id,
                    takeoff_s=takeoff_s,
                    landing_s=landing_s,
                    launch_point_m=route.position_at_time(takeoff_s, ugv_speed_mps),
                    rendezvous_point_m=route.position_at_time(landing_s, ugv_speed_mps),
                ))
            cycle_index += 1
    return sorted(events, key=lambda event: (event.takeoff_s, event.uav_id))


def sample_ugv_trajectory(
    route: PolylineRoute, ugv_speed_mps: float, horizon_s: int, step_s: int,
) -> list[tuple[int, Point]]:
    """Sample the route without changing event timing."""

    if step_s <= 0:
        raise ValueError("step_s must be positive")
    return [(t, route.position_at_time(t, ugv_speed_mps)) for t in range(0, horizon_s + 1, step_s)]
