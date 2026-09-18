# Model, feasibility argument and claim boundaries

This document is intended for an advisor discussion alongside actual results.
It distinguishes what is proved by the idealized model from what is only
observed in experiments. The complete frozen design is in
`EXPERIMENT_PROTOCOL_V2.md`; interpretations must follow its measured results.

## Mathematical model

UAV motion: dx/dt = v cos(psi), dy/dt = v sin(psi), |dpsi/dt| <= omega.
With v=10 m/s and omega=5 degrees/s, minimum turning radius R=v/omega=114.59 m.
The UGV position g(t) is arc-length interpolation along a fixed closed road
polyline at 5 m/s. A sortie launched at a and ending at b=a+1200 must satisfy
x(a)=g(a), x(b)=g(b). Camera rectangles rotate with aircraft heading.

At exposure time t, cell j is observed when its centre lies in at least one
airborne UAV footprint. Team reward is the weighted sum of distinct observed
AOI cells. Default priorities are one. Time-varying observation age is evaluated
separately: A_j(0)=0, A_j(t)=0 when observed, otherwise time since last
observation; never-observed cells remain in the denominator. Minimizing age is
NOT yet the planner's objective, so coverage gains cannot be asserted to imply
age gains.

The heading-aware choice optimizes over dynamically feasible next camera
poses. The no-heading-planning control ranks idealized camera poses first and
then clips its steering command to the same physical limit. Consequently both
execute physically feasible trajectories while only one anticipates steering
cost during ranking. Greedy uses the exact same feasible construction as full;
it is the essential control for assessing the added search stages.

## Why the returned trajectories are feasible within this model

From a pose, consider circles of radius R tangent to its heading on the left
and right. Their interiors do not overlap. Therefore any target is outside at
least one circle and admits a forward tangent path: turn on that circle, then
follow a straight tangent to the target (free arrival heading).

The construction accepts a new one-second step only when the remaining flight
distance exceeds the length of such a return plus 2*pi*R. When surveying ends,
let L be remaining distance and L_return the chosen return length. A full
circle of radius (L-L_return)/(2*pi) consumes the surplus, returns to the same
pose, and has radius at least R. The subsequent tangent path reaches g(b) at
exactly b. Both pieces obey speed and turn limits. Initial feasibility is
checked; infeasible configurations must fail rather than fabricate a landing.

This is a constructive feasibility argument, not an optimality proof. The
extra full-circle reserve can sacrifice coverage; it is used by every method.
Landing heading is unconstrained. Constant-curvature integration is analytic;
positions saved every second are samples of these arcs, not teleporting node
visits. Numerical endpoint and turn checks accompany every run.

## What the charging ILP does and does not prove

Each selected robot has a 14-minute charge block and a 20-minute flight block
on a 34-minute periodic cycle. The ILP chooses phases under the two-pad
capacity constraint. Average demand is 3*14/34=1.235 pads, so at least two pads
are necessary. A feasible two-pad schedule exists, establishing sufficiency
and minimal pad count for these idealized durations.

When all three robots are selected, total flight time per steady cycle is
fixed at 60 vehicle-minutes. The solver finds feasible phasing, not a new
method-specific flight-time improvement. With exact deterministic arrivals,
queue delays are zero. Claims of reduced delay require a separately specified
disturbance model and rescheduling experiment; zero-delay runs cannot support
such a claim.

## Questions an advisor should be able to ask

| Question | Evidence or limitation |
| --- | --- |
| Were the original table numbers reproduced? | No. They are unverified manuscript references. New rows come from saved simulation data. |
| Are comparisons physically equivalent? | Yes in v2: same speed, turn limit, battery model, pads, road path, camera and evaluator. v1 unrestricted controls are separate historical diagnostics. |
| Is a weak baseline making the optimizer look good? | Report point, command-clipped naive, parallel-strip survey AND the exact feasible greedy construction. Show every comparison even if greedy wins or ties. |
| Does local search actually help? | Inspect accepted-operator logs, paired coverage differences, and computation time. Zero/negative improvements must remain visible. |
| Is it persistent surveillance? | Missions repeat flight/charge cycles; freshness and blind-interval measurements are reported. There is no theorem guaranteeing bounded age forever. |
| Is it flight-ready? | No. This is 2D mission planning with idealized energy. Collision avoidance, 3D takeoff/landing, sensing uncertainty and landing robustness are outside the demonstrated model. |
| Why can a UAV go outside the AOI? | AOI specifies sensing reward, not restricted airspace. Return/loiter outside the AOI is permitted and produces no reward outside it. |
| Were conditions selected after seeing favorable values? | The v2 matrix, seeds, metrics and interpretation rules were fixed in a local protocol before its runs. Every listed condition is reported. It is not an external preregistration. |

## Claims that require restraint

Do not claim a universal 19% coverage increase, an 11.3% redundancy reduction,
strict superiority to all baselines, reduced charging delay, real-world
robustness or asymptotic persistent-coverage guarantees from these experiments.
Use scenario-specific measured effects and uncertainty, and describe the
continuous-trajectory adaptation of the manuscript's TOP operators explicitly.

The defensible contribution is an inspectable coupling of periodic energy
scheduling, moving-road rendezvous and camera-area planning under common motion
constraints. Whether added optimization stages offer a useful improvement is
an experimental question, not a premise.
