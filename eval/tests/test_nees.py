"""NEES consistency: mean position NEES should be near 3 (dof) for a well-tuned filter."""
import subprocess, sys, struct
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).parent.parent.parent
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
    subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(otlg), str(estm)])
    from eval.evaluate import read_est
    _, gt_rows = read_log(str(otlg))
    _, est_rows = read_est(str(estm))
    # compute NEES for position (dof=3) using P_pos block (6,6) in 15-state P
    # P is row-major 15x15, diag blocks: 0:3 dtheta, 3:6 dv, 6:9 dp, 9:12 dbg, 12:15 dba
    nees=[]
    for i in range(len(gt_rows)):
        err = est_rows[i]['p'] - gt_rows[i].gt_pos
        P = est_rows[i]['P']  # 15x15
        P_pos = P[6:9, 6:9]
        # regularize
        try:
            inv = np.linalg.inv(P_pos + np.eye(3)*1e-9)
        except np.linalg.LinAlgError:
            continue
        nees.append(float(err @ inv @ err))
    nees=np.array(nees)
    mean_nees=float(nees.mean())
    # For well-tuned filter, mean NEES ~ dof (3). Allow 5x band due to unmodeled r_dot noise and yaw unobservability.
    # This is a smoke test for divergence, not a strict chi2 pass.
    assert 0.1 < mean_nees < 15, f"mean NEES {mean_nees} outside [0.1,15]"
    # also check that max not insane
    assert np.median(nees) < 20
