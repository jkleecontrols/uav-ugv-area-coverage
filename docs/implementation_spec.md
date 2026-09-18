# ICUAS simulation specification, version 1

Historical v1 specification. For the completed current physical comparison,
read `EXPERIMENT_PROTOCOL_V2.md` and `PROFESSOR_BRIEF_KO.md`. V2 supersedes
the mission duration, baseline heading execution, waypoint decoder and metrics
described below. The frozen numerical source is retained with each batch.

This is a measured reimplementation based on the supplied ICUAS manuscript.
The author recalls that the manuscript may not have been simulated; its numeric
tables are unverified. No manuscript number is an optimization target.

## Parameters directly taken from the manuscript

Three UAVs, one UGV, two charging pads; flight 1200 s, charge 840 s; UAV
10 m/s, UGV 5 m/s; footprint 100 m forward by 67 m cross-track; heading
change at most 5 degrees per 1 s construction step. Reporting uses 1 m AOI
cell centres. All default weights are one because no priority map was supplied.
The stated 4:3 aspect ratio conflicts with 100:67; explicit footprint dimensions
take precedence. No altitude-dependent camera calibration is invented.

## Choices needed to make the manuscript executable

- Scenario: `scenarios/tamu.json`; AOI and road linework are from the existing
  `map/mapofroads.geojson`. WGS84 is projected to UTM 14N (EPSG:32614), then
  translated to an AOI-local origin. This is not a claim to recover the exact
  old figure's road sequence.
- Exact road intersections are split; disconnected segments are not joined
  across gaps. The longest shortest path in the component with most road
  length is traversed out and back at 5 m/s. The actual ordered vertices and
  original map hash are saved in every result. No dynamic UGV routing or
  UGV battery swaps are modeled: the simulation section specifies a fixed
  road route and gives no UGV energy model.
- Duration: 4080 s (two 34-minute cycles) including startup. UAVs start on
  the UGV with zero usable charge. Each begins its first charge at its chosen
  phase, waits on board before that phase, and then flies. Physical storage
  space aboard the UGV is not constrained; only active charging pads are.
- ILP slots use the paper-duration GCD of 120 s. Binary variables select
  one phase per admitted robot; charging occupancy cannot exceed two pads.
  Flight objective is summed over the common LCM horizon. Lexicographic
  tie-breaking prefers low-index admission and then earlier charge starts.
  With all three admitted and fixed duty cycles the flight objective is
  constant across feasible phases; the ILP is chiefly a feasibility problem.
- Takeoff/landing are instantaneous event transitions. The 20-minute airborne
  interval includes all surveying, return and loiter. Battery decreases
  linearly from 100% to 0% in 1200 s, charges linearly in 840 s; no reserve,
  vertical-flight, wind or collision-avoidance dynamics are asserted.
- The AOI is a sensing reward region, not a flight geofence. Return and loiter
  trajectories may pass outside it; only footprints intersecting AOI cells
  contribute to coverage. No unprovided airspace restrictions are assumed.
- Camera evaluation is at 1 Hz, not the manuscript's 60 fps video rate.
  Footprints are evaluated at exposure times, not swept continuously.
  Both spatial and temporal convergence must be considered before strong
  quantitative claims. Video playback rate is independent of simulated time.

## Executable interpretation of the algorithm

The original AIAA implementation in the neighboring DEVCOM repository was
consulted for construction, two-point exchange, one-point movement and 2-opt
structure. Its node-count objective is not reused as an area objective.

1. Heading-limited greedy construction propagates exact constant-curvature
   motion over 1 s, evaluates new footprint cells, and selects the highest
   gain. Five steering increments span -5 to +5 degrees. Positions are
   continuous; observation cells, not aircraft positions, form the raster.
   Initial heading is zero, as in the paper pseudocode.
2. Every accepted step preserves a feasible landing suffix. A minimum-radius
   left/right circular arc followed by its tangent line reaches the future
   UGV position. One full circle consumes remaining time at radius at least
   v/turn-rate. Thus the full sortie has exact duration, constant analytic
   speed and a continuous heading. Constant-curvature integration means the
   distance between sampled poses is a chord slightly shorter than v*dt.
3. Team search extracts guidance waypoints every 60 s, starting at 30 s,
   applies cross-sortie two-point exchange, one-point displacement, and 2-opt
   segment reversal. Each proposal is decoded using the same aircraft model
   and return rule. A guidance waypoint may be bypassed by the decoder;
   evaluation always uses the resulting feasible trajectory. This is an
   explicit continuous-trajectory adaptation, not an exact reproduction of
   an unspecified waypoint-level implementation.
4. A bounded reward-reinitialization pass penalizes frequently observed cells
   by 1/(1+visit count), constructs alternatives, and retains only improvements
   in original team unique area. The incumbent score is never replaced by a
   rescaled score. Search is bounded by the configuration, rather than an
   unbounded stagnation loop. Search logs retain every acceptance decision.
5. This is offline team planning: previously planned sortie footprints are
   known, including future footprints, to discourage inter-UAV redundancy.
   The objective only counts exposures within the reporting horizon. A final
   partial sortie is fully planned to verify landing but clipped for metrics.

Planning uses a 5 m raster by default for runtime. Final evaluation is a
separate 1 m raster. A 1 m planning sensitivity run is needed to quantify this
approximation; changing planning resolution is not relabeling a 5 m metric.

## Baselines and ablations

- `point`: nearest unvisited sparse targets (150 m spacing, 15 m visit
  tolerance), shared visit knowledge across sorties, same aircraft and camera.
  This is a point-driven greedy baseline, not the full AIAA solver.
- `naive`: greedy area construction without heading cone or reinitialization.
- `full`: heading cone, local operators, reward reinitialization.
- `no_heading`: identical to full except construction and waypoint decoding
  may change heading freely; the return suffix is still feasible at 5 deg/s.
- `no_reinit`: identical to full through local search; skips reinitialization.
- `neither`: both switches off, retaining local search. It differs from
  `naive`, which also skips local search.

Every method is evaluated with the SAME rectangular camera and AOI denominator.
The manuscript's description of evaluating point coverage solely by centres
would conflate sensing and planning; it is not used for physical coverage.
Methods without the heading cone are idealized kinematic controls, not a
claim that such instantaneous turn rates are physically achievable.

## Reported metrics

- Coverage: number of AOI cell centres observed at least once / AOI cells.
- Team score: unique covered cells, with all priority weights one. At 1 m this
  numerically equals the cell-count area estimate in square metres. No old
  arbitrary team-score scaling is inferred.
- Exposure redundancy: (total UAV-cell exposures - unique cells) / exposures.
  This includes adjacent frames and simultaneous cameras, and depends strongly
  on camera sample rate.
- Revisited covered cells: fraction of covered cells with two or more distinct
  team-coverage episodes. A new episode requires at least one absent sampled
  second; simultaneous camera footprints form one team observation.
- Charging delay: first charging start minus landing time, over completed
  sorties. It is calculated from saved events. With no disturbances and a
  feasible schedule, all methods should have zero charging delay. A future
  delay study would need an explicit disturbance model.

These definitions resolve missing manuscript details. They cannot be compared
directly to the old unverified redundancy or team-score values.

## Verification and provenance

Tests compare windowed rasterization against independent polygon geometry,
exercise 100 random tangent returns, and check that every UGV route segment
lies on source roads. Every simulation checks sampled speed, heading for
constrained methods, exact launch/landing rendezvous, battery bounds and pad
assignments. Results save the configuration, seed, schedule, search log,
complete sortie trajectories, sampled mission states, map/source/artifact
hashes and measured metrics. Plots and MP4 read those same saved artifacts.

Success means an executable and inspectable interpretation with honest
measurements. It does not establish that the manuscript's performance claims
hold; comparisons and convergence checks are required to assess them.
