"""Periodic charging schedules for the ICUAS fixed-route simulation."""

from __future__ import annotations

from dataclasses import dataclass
from functools import reduce
from math import gcd
from typing import Mapping

import gurobipy as gp
from gurobipy import GRB


def _lcm(values: list[int]) -> int:
    return reduce(lambda a, b: a * b // gcd(a, b), values, 1)


@dataclass(frozen=True)
class PeriodicSchedule:
    """A charge-then-flight periodic plan; phase is each UAV's charge start."""

    slot_seconds: int
    horizon_slots: int
    charge_slots: tuple[int, ...]
    flight_slots: tuple[int, ...]
    charge_start_slot: Mapping[int, int]

    @property
    def selected_uavs(self) -> tuple[int, ...]:
        return tuple(sorted(self.charge_start_slot))

    def is_charging(self, uav_id: int, time_slot: int) -> bool:
        if uav_id not in self.charge_start_slot:
            return False
        cycle = self.charge_slots[uav_id] + self.flight_slots[uav_id]
        return (time_slot - self.charge_start_slot[uav_id]) % cycle < self.charge_slots[uav_id]


def solve_max_flight_schedule(
    charge_seconds: list[int], flight_seconds: list[int], pads: int, slot_seconds: int = 60,
) -> PeriodicSchedule:
    """Solve the Section-IV periodic charging ILP with deterministic tie-breaks."""

    if len(charge_seconds) != len(flight_seconds) or not charge_seconds:
        raise ValueError("charge and flight durations must be non-empty and aligned")
    if pads <= 0 or slot_seconds <= 0:
        raise ValueError("pads and slot_seconds must be positive")
    if any(t <= 0 or t % slot_seconds for t in charge_seconds + flight_seconds):
        raise ValueError("durations must be positive multiples of slot_seconds")
    charge = tuple(t // slot_seconds for t in charge_seconds)
    flight = tuple(t // slot_seconds for t in flight_seconds)
    cycles = [c + f for c, f in zip(charge, flight)]
    horizon = _lcm(cycles)

    model = gp.Model("icuas_periodic_schedule")
    model.Params.OutputFlag = 0
    phase = {(u, p): model.addVar(vtype=GRB.BINARY, name=f"phase_{u}_{p}")
             for u, cycle in enumerate(cycles) for p in range(cycle)}
    selected = {u: gp.quicksum(phase[u, p] for p in range(cycles[u])) for u in range(len(cycles))}
    for u in selected:
        model.addConstr(selected[u] <= 1, name=f"one_phase_{u}")
    for t in range(horizon):
        model.addConstr(gp.quicksum(
            phase[u, p] for u, cycle in enumerate(cycles) for p in range(cycle)
            if (t - p) % cycle < charge[u]
        ) <= pads, name=f"pad_{t}")

    model.ModelSense = GRB.MAXIMIZE
    # Count work over the COMMON horizon, not one unequal cycle per robot.
    n = len(cycles)
    model.setObjectiveN(gp.quicksum((horizon // cycles[u]) * flight[u] * selected[u]
                                   for u in selected), 0, priority=2*n+1,
                        abstol=0, reltol=0)
    for u in selected:
        model.setObjectiveN(selected[u], 1+u, priority=2*n-u, abstol=0, reltol=0)
        model.setObjectiveN(-gp.quicksum(p * phase[u, p] for p in range(cycles[u])),
                            1+n+u, priority=n-u, abstol=0, reltol=0)
    model.optimize()
    if model.Status != GRB.OPTIMAL:
        raise RuntimeError(f"schedule ILP did not solve optimally (status {model.Status})")
    starts = {u: p for (u, p), variable in phase.items() if variable.X > 0.5}
    return PeriodicSchedule(slot_seconds, horizon, charge, flight, starts)
