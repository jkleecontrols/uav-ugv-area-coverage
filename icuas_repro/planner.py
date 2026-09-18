"""Area-reward TOP construction and feasible trajectory local search.

The paper does not specify how heading-constrained walks meet a moving landing
deadline. Here a constant-speed aircraft always reserves a feasible arc/line
return and a complete circular loiter. This supplies an exact-duration suffix.
TOP operators act on guidance waypoints; each proposal is decoded through the
same turn-limited dynamics and accepted only for increased team area reward.
"""
from __future__ import annotations

import math
import numpy as np

TAU = 2*math.pi


def wrap(angle):
    return (angle+math.pi) % TAU-math.pi


def advance(pose, length, curvature):
    x, y, h = pose
    angle = length*curvature
    # Midpoint/sinc form avoids subtractive cancellation on almost-straight
    # waypoint tracking, where sin(h+epsilon)-sin(h) loses precision.
    chord = length*(math.sin(angle/2)/(angle/2) if abs(angle) > 1e-10 else 1.)
    return np.array([x+chord*math.cos(h+angle/2),
                     y+chord*math.sin(h+angle/2), wrap(h+angle)])


def return_segments(pose, end, radius):
    """Shortest of left/right fixed-radius arc plus forward tangent line."""
    x, y, h = pose
    options = []
    for sign in (-1, 1):
        center = np.array([x-sign*radius*math.sin(h), y+sign*radius*math.cos(h)])
        vector = np.asarray(end)-center
        distance = float(np.linalg.norm(vector))
        if distance < radius-1e-8:
            continue
        phi = math.atan2(vector[1], vector[0])-sign*math.acos(min(1., radius/max(distance, 1e-12)))
        phi0 = h-sign*math.pi/2
        angle = (sign*(phi-phi0)) % TAU
        arc = radius*angle
        line = math.sqrt(max(0., distance*distance-radius*radius))
        options.append([(arc, sign/radius), (line, 0.)])
    if not options:
        raise RuntimeError('No tangent return exists')
    return min(options, key=lambda segments: sum(length for length, _ in segments))


def sample_segments(start, segments, speed, duration):
    """Integrate exact arcs, including sub-second boundaries between segments."""
    pose = np.array(start, dtype=float)
    output = [pose.copy()]
    remaining = [(float(length), float(k)) for length, k in segments if length > 1e-9]
    index = 0
    for _ in range(duration):
        distance = speed
        while distance > 1e-7 and index < len(remaining):
            length, curvature = remaining[index]
            travelled = min(distance, length)
            pose = advance(pose, travelled, curvature)
            length -= travelled
            distance -= travelled
            remaining[index] = (length, curvature)
            if length < 1e-7:
                index += 1
        if distance > 1e-5:
            raise RuntimeError('Insufficient trajectory length')
        output.append(pose.copy())
    return np.asarray(output)


def construct(event, config, raster, forecast, rng, method, guides=None,
              reinitialize=False, previous_trajectories=()):
    speed = config['uav_speed_mps']
    duration = event.landing_s-event.takeoff_s
    turn = math.radians(config['heading_step_deg'])
    radius = speed/turn
    constrained = method not in ('naive', 'no_heading', 'neither')
    physical = config.get('physical_heading_limit', False)
    adaptive = reinitialize
    point_based = method == 'point'
    deltas = np.linspace(-turn, turn, 5) if constrained else np.linspace(-math.pi, math.pi, 17)[:-1]
    pose = np.array([*event.launch_point_m, config['initial_heading_rad']])
    end = np.asarray(event.rendezvous_point_m)
    poses = [pose.copy()]
    counts = np.array(forecast, copy=True)
    counts[raster.footprint(pose)] += 1
    guide_index = 0
    # Sparse targets are used only by the point planner, with the SAME camera
    # used for all final evaluation. This avoids different sensing denominators.
    spacing = max(1, int(round(config['point_target_spacing_m']/raster.resolution)))
    point_mask = np.zeros(raster.shape, dtype=bool)
    point_mask[::spacing, ::spacing] = True
    target_ids = np.flatnonzero(raster.valid & point_mask.ravel())
    point_visited = np.zeros(len(target_ids), dtype=bool)
    point_xy = np.column_stack((raster.x[target_ids % raster.shape[1]],
                                raster.y[target_ids // raster.shape[1]]))
    if point_based:
        for previous in previous_trajectories:
            point_visited |= np.any(np.linalg.norm(previous[:, None, :2]-point_xy[None, :, :], axis=2) < 15, axis=0)
    destination = end.copy()
    for t in range(duration):
        reserve = return_segments(pose, end, radius)
        left = speed*(duration-t)
        if left < sum(l for l, _ in reserve)+TAU*radius-1e-6:
            raise RuntimeError('Return invariant violated')
        if guides is not None:
            while guide_index < len(guides) and np.linalg.norm(pose[:2]-guides[guide_index]) < 25:
                guide_index += 1
            destination = guides[min(guide_index, len(guides)-1)] if len(guides) else end
        elif point_based:
            if len(point_xy):
                distances = np.linalg.norm(point_xy-pose[:2], axis=1)
                point_visited |= distances < 15
                remaining = np.where(~point_visited)[0]
                if physical and not len(remaining):
                    # A persistent point baseline repeats its target patrol;
                    # it must not stop surveying after a single completed tour.
                    point_visited[:] = distances < 15
                    remaining = np.where(~point_visited)[0]
                if len(remaining):
                    destination = point_xy[remaining[np.argmin(distances[remaining])]]
                else:
                    destination = end
        elif t % 20 == 0:
            pool = np.flatnonzero(raster.valid)
            candidates = rng.choice(pool, min(256, len(pool)), replace=False)
            xx = raster.x[candidates % raster.shape[1]]
            yy = raster.y[candidates // raster.shape[1]]
            distance = np.hypot(xx-pose[0], yy-pose[1])
            priority = (1/(1+counts[candidates]) if adaptive else (counts[candidates] == 0).astype(float))
            merit = priority/(50+distance)
            best = int(np.argmax(merit))
            destination = np.array([xx[best], yy[best]])
        target_angle = math.atan2(destination[1]-pose[1], destination[0]-pose[0])
        if point_based or guides is not None:
            limit = turn if constrained or point_based or physical else math.pi
            if physical and guides is not None:
                guidance_arc = return_segments(pose, destination, radius)
                # Sample across arc/line boundary rather than pure pursuit,
                # which can orbit a nearby target without reaching it.
                guided_next = (sample_segments(pose, guidance_arc, speed, 1)[-1]
                               if sum(length for length, _ in guidance_arc) >= speed
                               else advance(pose, speed, 0.))
                changes = [wrap(guided_next[2]-pose[2])]
            else:
                guided_next = None
                changes = [np.clip(wrap(target_angle-pose[2]), -limit, limit)]
        else:
            changes = deltas
        proposals = []
        for delta in changes:
            if physical and not constrained and guides is None and not point_based:
                # Candidate ranking deliberately ignores steering effort in
                # this ablation. Execution obeys exactly the common dynamics.
                hypothetical = np.array([pose[0]+speed*math.cos(pose[2]+delta),
                                         pose[1]+speed*math.sin(pose[2]+delta), wrap(pose[2]+delta)])
            else:
                hypothetical = None
            executed_delta = np.clip(delta, -turn, turn) if physical else delta
            nxt = advance(pose, speed, float(executed_delta)/speed)
            if physical and guides is not None:
                nxt = guided_next
            back = return_segments(nxt, end, radius)
            if speed*(duration-t-1) < sum(l for l, _ in back)+TAU*radius:
                continue
            if point_based or guides is not None:
                value = -np.linalg.norm(nxt[:2]-destination)
            else:
                ids = raster.footprint(hypothetical if hypothetical is not None else nxt)
                new = float(np.count_nonzero(counts[ids] == 0))
                value = new
                if adaptive:
                    value += .08*float(np.sum(1/(1+counts[ids])))
                value += .02*math.cos(wrap(target_angle-nxt[2]))
                value += float(rng.uniform(0, 1e-6))
            proposals.append((value, nxt))
        if not proposals:
            break
        pose = max(proposals, key=lambda item: item[0])[1]
        poses.append(pose.copy())
        counts[raster.footprint(pose)] += 1
    # Fill surplus with one full circle of radius >= minimum turn radius;
    # return finishes at the scheduled rendezvous exactly, with no hovering.
    remaining_s = duration-(len(poses)-1)
    back = return_segments(pose, end, radius)
    surplus = speed*remaining_s-sum(l for l, _ in back)
    if surplus < TAU*radius-1e-5:
        raise RuntimeError('Insufficient loiter reserve')
    segments = [(surplus, TAU/surplus)] + back
    suffix = sample_segments(pose, segments, speed, remaining_s)
    trajectory = np.vstack((np.asarray(poses), suffix[1:]))
    if np.linalg.norm(trajectory[-1, :2]-end) > 1e-5:
        raise RuntimeError('Rendezvous miss')
    return trajectory


def sweep_guides(raster, config, uav_id):
    """Parallel strips along AOI principal axis, partitioned among UAVs."""
    from shapely.geometry import LineString
    boundary = np.asarray(raster.aoi.exterior.coords)[:-1]
    origin = boundary.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov((boundary-origin).T))
    along = vectors[:, -1]
    if along[0] < 0:
        along = -along
    across = np.array([-along[1], along[0]])
    rotation = np.column_stack((along, across))
    local = (boundary-origin) @ rotation
    spacing = config['fov_cross_m']*config.get('sweep_spacing_fraction', .9)
    levels = np.arange(local[:, 1].min()+spacing/2, local[:, 1].max(), spacing)
    targets = []
    for index, level in enumerate(levels):
        if index % config['uavs'] != uav_id:
            continue
        endpoints = np.array([[local[:, 0].min()-1, level], [local[:, 0].max()+1, level]]) @ rotation.T+origin
        intersection = raster.aoi.intersection(LineString(endpoints))
        pieces = list(intersection.geoms) if hasattr(intersection, 'geoms') else [intersection]
        segments = [np.asarray(piece.coords)[[0, -1]] for piece in pieces if piece.geom_type == 'LineString']
        segments.sort(key=lambda segment: float((segment[0]-origin) @ along))
        row = [point for segment in segments for point in segment]
        if len(targets)//2 % 2:
            row.reverse()
        targets.extend(row)
    if not targets:
        raise ValueError('No sweep strips assigned to UAV')
    # Repeated patrol if the available flight time exceeds one assigned scan.
    return np.asarray(targets*4)


def plan_team(events, config, raster, seed, method):
    if not events:
        return [], [{'stage': 'no_airborne_sorties', 'covered_planning_cells': 0}]
    rng = np.random.default_rng(seed)
    trajectories, counts = [], []
    total = np.zeros(raster.size, dtype=np.int32)
    log = []
    def contribution_for(index, trajectory):
        # Team objective uses exactly the reporting horizon, including partial
        # final sorties, at the configured planning spatial resolution.
        end = min(len(trajectory)-1, config['horizon_s']-events[index].takeoff_s)
        return raster.route_counts(trajectory[:end])
    for index, event in enumerate(events):
        guides = sweep_guides(raster, config, event.uav_id) if method == 'sweep' else None
        trajectory = construct(event, config, raster, total, rng, method,
                               previous_trajectories=trajectories, guides=guides)
        contribution = contribution_for(index, trajectory)
        trajectories.append(trajectory)
        counts.append(contribution)
        total += contribution
    log.append({'stage': 'construction', 'covered_planning_cells': int(np.count_nonzero(total))})

    if method in ('point', 'sweep', 'greedy'):
        return trajectories, log
    # Same operators in full and factorial ablations; naive is the greedy
    # no-heading/no-reinitialization baseline described in the paper.
    if method != 'naive':
        for iteration in range(config['search_iterations']):
            for operator in ('two_point_exchange', 'one_point_movement', 'two_opt'):
                u = int(rng.integers(len(events)))
                indices = [u]
                guides = {u: trajectories[u][30:-120:60, :2].copy()}
                if len(guides[u]) < 3:
                    continue
                if operator == 'two_point_exchange' and len(events) > 1:
                    v = (u+1) % len(events)
                    indices.append(v)
                    guides[v] = trajectories[v][30:-120:60, :2].copy()
                    a, b = int(rng.integers(len(guides[u]))), int(rng.integers(len(guides[v])))
                    guides[u][a], guides[v][b] = guides[v][b].copy(), guides[u][a].copy()
                elif operator == 'one_point_movement':
                    a = int(rng.integers(len(guides[u])))
                    guides[u][a] += rng.normal(0, 67, size=2)
                else:
                    a, b = sorted(rng.choice(len(guides[u]), 2, replace=False))
                    guides[u][a:b+1] = guides[u][a:b+1][::-1]
                others = total.copy()
                for i in indices:
                    others -= counts[i]
                proposals, proposal_counts = {}, {}
                for i in indices:
                    proposals[i] = construct(events[i], config, raster, others, rng, method, guides[i])
                    proposal_counts[i] = contribution_for(i, proposals[i])
                candidate = others.copy()
                for values in proposal_counts.values():
                    candidate += values
                accepted = np.count_nonzero(candidate) > np.count_nonzero(total)
                if accepted:
                    total = candidate
                    for i in indices:
                        trajectories[i], counts[i] = proposals[i], proposal_counts[i]
                log.append({'stage': operator, 'iteration': iteration, 'accepted': bool(accepted),
                            'covered_planning_cells': int(np.count_nonzero(total))})
    if method in ('full', 'no_heading'):
        for cycle in range(config['reinitializations']):
            for u, event in enumerate(events):
                # Diversification penalizes frequent visits in the incumbent
                # team solution, then compares against the original area score.
                proposal = construct(event, config, raster, total, rng, method, reinitialize=True)
                contribution = contribution_for(u, proposal)
                candidate = total-counts[u]+contribution
                accepted = np.count_nonzero(candidate) > np.count_nonzero(total)
                if accepted:
                    total = candidate
                    trajectories[u], counts[u] = proposal, contribution
                log.append({'stage': 'reward_reinitialization', 'cycle': cycle, 'uav': event.uav_id,
                            'accepted': bool(accepted), 'covered_planning_cells': int(np.count_nonzero(total))})
    return trajectories, log
