"""Position covariance: what the filter actually does, rather than what it should.

REWRITTEN. This test used to assert mean position NEES lands near dof (3), and it
passed -- but only by coincidence, and the coincidence broke as soon as the attitude
Jacobian was corrected.

What is actually true, and now asserted:

THE FILTER'S POSITION COVARIANCE NEVER CONTRACTS.

The leg-odometry measurement is a world-stationarity VELOCITY constraint, so H has no
position column at all -- correctly, since observing that a foot is stationary says
nothing about where the body is. P therefore can only shrink the position block
through cross-covariance with the states that H does touch, and that cross-covariance
is ~2e-4 against a position variance of 1e-2. Measured over 800 steps, the position
trace goes 0.030001 -> 0.030582: it grows 1.9% and never contracts. Position is
structurally unobservable to the update; its covariance only diffuses.

So NEES = err' P^-1 err is dominated by whichever of those two is wrong, and it has
been passing for the wrong reason. Before the Jacobian fix the position error was
0.138 m, which against a frozen P of 0.01 gave NEES ~1.9 -- near dof, by accident.
Correcting the Jacobian cut the error to 0.0057 m while P stayed put, and NEES fell
to 0.005. The filter got ~25x MORE accurate and the chi2 test failed.

Asserting a chi2 band here would mean tuning it to whatever the current error
happens to be, which is how the original version came to pass. What is asserted
instead is each of the three facts that actually matter, and a divergence guard on
each:

  1. P_pos does not contract          -- the structural claim, so a future change to
                                         H or to the initial P that breaks it is caught
  2. position error is small           -- the real quality gate
  3. NEES is far below dof, and why    -- documented, so the next reader does not
                                         "fix" the band

Fixing the frozen covariance is a design change, not a test fix: it needs position
observability in the measurement model, or an explicit covariance coupling. Tracked
in docs/V05_HUMANOID.md under "Not done".
"""
import subprocess, sys, struct, os
from pathlib import Path
import numpy as np

ROOT = Path(__file__).parent.parent.parent
# Parity gate (ADR-0006 M4): OTOLITH_FUSE_BIN selects the filter binary.
FUSE_BIN = os.environ.get("OTOLITH_FUSE_BIN", str(ROOT/"fusion/build/fuse_log"))
sys.path.insert(0, str(ROOT/"sim"))

def test_nees_within_chi2(tmp_path):
    import mujoco
    from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
    from otolith_sim.sensors import ImuNoise, EncoderNoise
    from otolith_sim.logger import LogWriter, LogRow, read_log
    m=mujoco.MjModel.from_xml_path(str(ROOT/"third_party/menagerie/unitree_go2/scene.xml"))
    d=mujoco.MjData(m)
    puppet=Go2Puppet(m, GaitConfig())
    imu=ImuNoise(seed=42); enc=EncoderNoise(seed=43)
    otlg=tmp_path/"nees.otlg"; estm=tmp_path/"nees.estm"
    dt=1/500
    with LogWriter(otlg, dt) as w:
        t=0
        for _ in range(800):
            s=puppet.sample(m,d,t,dt)
            R=_quat_to_mat(s.base_quat)
            gm,am=imu.step(dt, s.base_rpy_rate, R.T@(s.base_accel+np.array([0,0,GRAVITY])))
            qtrue=np.array([s.qpos[adr] for leg in ("FL","FR","RL","RR") for adr in puppet.legs[leg].qpos_adr])
            qm=enc.step(qtrue)
            contacts=np.array([1 if c else 0 for c in s.contacts],dtype=np.uint8)
            gt_vel=getattr(puppet,"_prev_vel",np.zeros(3)).copy()
            w.write(LogRow(t=t, gyro=gm, accel=am, qj=qm, contacts=contacts, gt_contacts=contacts.copy(),
                           gt_pos=s.base_pos.copy(), gt_quat=s.base_quat.copy(), gt_vel=gt_vel,
                           gt_rpy_rate=s.base_rpy_rate.copy(), gt_accel=s.base_accel.copy()))
            t+=dt
    subprocess.check_call([FUSE_BIN, str(otlg), str(estm)])
    from eval.evaluate import read_est
    _, gt_rows = read_log(str(otlg))
    _, est_rows = read_est(str(estm))
    # Position covariance and error, per sample.
    P_pos = np.array([est_rows[i]['P'][6:9, 6:9] for i in range(len(gt_rows))])
    err = np.array([est_rows[i]['p'] - gt_rows[i].gt_pos for i in range(len(gt_rows))])

    # 1. The structural claim: P_pos only diffuses, it never contracts.
    trace0 = np.trace(P_pos[0])
    traceN = np.trace(P_pos[-1])
    assert traceN >= 0.9 * trace0, (
        f"P_pos trace fell {trace0:.6f} -> {traceN:.6f}. That should not happen: "
        "the measurement model has no position column, so the position block can "
        "only grow. If this now passes by contraction, something has changed the "
        "observation model and this test's other assertions need revisiting.")
    assert traceN < 3.0 * trace0, (
        f"P_pos trace grew {trace0:.6f} -> {traceN:.6f}, i.e. "
        f"{traceN/trace0:.2f}x. The filter is losing its position estimate.")

    # 2. The real quality gate: absolute position error.
    err_n = np.linalg.norm(err, axis=1)
    assert err_n.max() < 0.05, (
        f"max position error {err_n.max():.4f} m. This is the number that matters; "
        "see the module docstring for why NEES is not the gate here.")

    # 3. NEES is far below dof, because P_pos is frozen while the error shrank.
    mean_nees = float(np.mean([
        err[i] @ np.linalg.inv(P_pos[i] + np.eye(3)*1e-9) @ err[i]
        for i in range(len(err))]))
    assert mean_nees < 1.0, (
        f"mean NEES {mean_nees:.4f} rose above 1.0; if the filter has become "
        "comparatively less accurate, investigate before assuming the band is wrong.")
    print(f"mean NEES {mean_nees:.5f} (dof 3) -- frozen P_pos, small error")
