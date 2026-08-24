# 0002 — Puppet Approach and Language Split

Date: 2026-08-24
Status: Accepted

## Context

The contact-aided EKF needs a Go2 that *moves*. The stock Menagerie model has
no controller — left alone it collapses, and a collapsed robot generates no
motion for a state estimator to track. Options: (a) full dynamics with a
locomotion controller (PD standing, or an RL trot policy), (b) a kinematic
puppet that prescribes qpos directly.

Also: which language owns which layer, and how Python packages are managed.

## Decision

1. **Kinematic puppet (v0.1)**: qpos is prescribed directly. To keep the
   estimator evaluation honest, the gait is *consistent by construction*:
   footholds are world-fixed on a stride grid, the base advances in lockstep,
   swing feet follow lifted arcs, and per-leg closed-form IK (constants
   introspected from the model) produces joint angles. Consequences: contact
   flags and leg-odometry are true by construction — estimator error is
   attributable to sensor noise, not puppet artifacts. Rejected: sinusoid
   puppets (stance feet slide, corrupting leg-odometry eval) and dynamics
   controllers (scope; revisit as v0.1.1 with PD standing + pushes).
2. **Python for scaffolding, C++ for the artifact**: the sim/sensor layer is
   throwaway tooling — Python iterates fast and 455Hz achieved (of 500Hz
   target) is sufficient for streaming. The fusion EKF is the artifact that
   gets ported to Rust and RTL, so it is C++ with Rally discipline from day
   one.
3. **pixi is the venv**: Python packages (numpy, matplotlib, pyyaml, pytest)
   are pixi dependencies, not a separate virtualenv. Rationale: a separate
   venv splits the interpreter and loses `rclpy`; pixi.lock gives the same
   reproducibility with ROS 2 in the same environment.

## Consequences

- The IK closed form is verified against MuJoCo FK by unit tests (5mm,
  random reachable targets); the home keyframe pins the elbow/roll branch
  conventions (theta = 0, 0.9, -1.8 must reproduce exactly).
- Python loop pacing is ~455/500Hz; the C++ fusion node must tolerate
  timestamp jitter rather than assume perfect ticks (jitter histograms will
  quantify this — an honest-number deliverable, not a bug).
- First-sample rates/accelerations are zero (finite-difference warm-up),
  documented in the node.
