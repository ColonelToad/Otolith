"""Bake a puppet gait to a replayable joint trajectory.

WHY THIS EXISTS
===============
The filter is not the demo's bottleneck. Measured live: predict 330 us + update 679
us = 1009 us of a 2000 us budget at 500 Hz, i.e. 50.5% duty -- comfortably
sustainable. The *puppet* is the bottleneck: 21.95 ms per OP3 sample, a 46 Hz
ceiling, because the DLS IK re-solves from the nominal posture every step and each
iteration costs 13 `sole_world` evaluations in numpy.

So a biped demo fed live from the puppet runs at ~40 Hz and never reaches its
target, which makes the utilisation panel look broken when the filter is fine.

The gait is deterministic and the puppet is kinematic by construction -- footholds
are world-fixed on a stride grid and the joint angles are a pure function of time.
So it can be solved once, offline, and streamed. The estimator still does real work
per step: IMU and encoder noise are applied live, leg odometry is differenced from
the streamed joints, and the covariance propagates. Only the gait authoring moves
offline, which is where the cost was.

`record_g1_log` already produced these trajectories for the eval harness; this just
persists them so the ROS node can replay them at full rate.

Usage:
    pixi run python -m otolith_sim.bake_replay g1   /tmp/g1_replay.npz
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim"))

BAKERS = {
    "go2": None,   # Go2's closed-form 2R IK is already fast enough; no replay needed
    "g1": (".work/g1scene/scene.xml", "g1_puppet", "G1Puppet", "G1GaitConfig"),
    "apollo": ("third_party/menagerie/apptronik_apollo/scene.xml",
               "apollo_puppet", "ApolloPuppet", "ApolloGaitConfig"),
    "op3": (".work/op3scene/scene.xml", "op3_puppet", "OP3Puppet", "OP3GaitConfig"),
}


def bake(robot: str, out_path: str, duration_s: float = 60.0, dt: float = 1 / 500) -> Path:
    import importlib
    import mujoco

    if robot not in BAKERS or BAKERS[robot] is None:
        raise ValueError(f"{robot} does not need a replay (closed-form IK) or is unknown")
    scene, mod, pcls, ccls = BAKERS[robot]
    model = mujoco.MjModel.from_xml_path(str(ROOT / scene))
    m = importlib.import_module(f"otolith_sim.{mod}")
    puppet = getattr(m, pcls)(model, cfg=getattr(m, ccls)())

    n = int(round(duration_s / dt))
    qpos = np.empty((n, model.nq), dtype=np.float64)
    contacts = np.empty((n, 2), dtype=bool)
    for i in range(n):
        s = puppet.sample(i * dt, dt)
        qpos[i] = s.qpos
        contacts[i] = s.contacts
    out = Path(out_path)
    np.savez_compressed(out, qpos=qpos, contacts=contacts, dt=dt, robot=robot)
    return out


class ReplayTraj:
    """Streams a baked trajectory. Loops rather than stopping, for a demo."""

    def __init__(self, path: str | Path):
        d = np.load(path, allow_pickle=False)
        self.qpos = d["qpos"]
        self.contacts = d["contacts"]
        self.dt = float(d["dt"])
        self.robot = str(d["robot"])
        self.n = len(self.qpos)
        self.i = 0

    def __post_init__(self):
        pass

    def _quat_to_rpy(self, q):
        w, x, y, z = q
        return np.array([
            np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y + z * z)),
            np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)),
            np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z + x * x))])

    def sample(self, t: float, dt: float):
        n = self.n
        i = int(round(t / dt)) % n
        j = (i + 1) % n
        k = (i - 1) % n
        vel = (self.qpos[j, 0:3] - self.qpos[k, 0:3]) / (2 * dt)
        accel = (self.qpos[j, 0:3] - 2 * self.qpos[i, 0:3] + self.qpos[k, 0:3]) / (dt * dt)
        rpy = np.stack([self._quat_to_rpy(self.qpos[m, 3:7]) for m in (k, i, j)])
        rpy_rate = (rpy[2] - rpy[0]) / (2 * dt)
        # The live puppet's finite differences are one-sided on the first sample and
        # zero there; matching that avoids a startup spike in the IMU that the
        # estimator would integrate as a real rotation.
        if i == 0:
            vel = np.zeros(3); accel = np.zeros(3); rpy_rate = np.zeros(3)
        return _Replayed(self.qpos[i], self.contacts[i], i * self.dt,
                         vel, accel, rpy_rate)


class _Replayed:
    """One replayed step, shaped like a BipedSample.

    The base pose comes straight out of qpos (the free joint is qpos[0:7]). The
    derivatives are NOT stored -- they are differenced from the replayed
    positions, which is what the live puppet does too. Storing them would mean the
    baked file and the live path could disagree about what the truth is; differencing
    keeps a single definition.
    """

    __slots__ = ("qpos", "contacts", "t", "_vel", "_accel", "_rpy_rate")

    def __init__(self, qpos, contacts, t, vel, accel, rpy_rate):
        self.qpos = qpos
        self.contacts = contacts
        self.t = t
        self._vel, self._accel, self._rpy_rate = vel, accel, rpy_rate

    @property
    def base_pos(self):
        return self.qpos[0:3].copy()

    @property
    def base_quat(self):
        return self.qpos[3:7].copy()

    # Derivatives of the base. These are what the IMU message is built from, so they
    # have to exist even in replay -- an AttributeError here silently costs the whole
    # demo rate, so they are derived rather than stubbed.
    @property
    def base_vel(self):
        return self._vel

    @property
    def base_accel(self):
        return self._accel

    @property
    def base_rpy_rate(self):
        return self._rpy_rate


def main() -> int:
    if len(sys.argv) < 3:
        print("usage: bake_replay.py <robot> <out.npz> [duration_s]")
        print(f"robots needing a replay: {sorted(k for k, v in BAKERS.items() if v)}")
        return 2
    robot, out = sys.argv[1], sys.argv[2]
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    p = bake(robot, out, dur)
    traj = ReplayTraj(str(p))
    print(f"wrote {p}: {traj.n} steps, {traj.n * traj.dt:.1f}s at {1 / traj.dt:.0f} Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())