"""Fault injection: dropouts, bias jumps, stuck encoders — must not crash and must degrade gracefully."""
import subprocess, struct, sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT/"sim"))
from otolith_sim.logger import read_log

def run_with_fault(tmp_path, fault_fn):
    import mujoco
    from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
    from otolith_sim.sensors import ImuNoise, EncoderNoise
    from otolith_sim.logger import LogWriter, LogRow
    m=mujoco.MjModel.from_xml_path(str(ROOT/"third_party/menagerie/unitree_go2/scene.xml"))
    d=mujoco.MjData(m)
    puppet=Go2Puppet(m, GaitConfig())
    imu=ImuNoise(seed=0); enc=EncoderNoise(seed=1)
    otlg=tmp_path/"fault.otlg"; estm=tmp_path/"fault.estm"
    dt=1/500
    rows=[]
    with LogWriter(otlg, dt) as w:
        t=0
        for i in range(1000):
            s=puppet.sample(m,d,t,dt)
            R=_quat_to_mat(s.base_quat)
            gm,am=imu.step(dt, s.base_rpy_rate, R.T@(s.base_accel+np.array([0,0,GRAVITY])))
            qtrue=np.array([s.qpos[adr] for leg in ("FL","FR","RL","RR") for adr in puppet.legs[leg].qpos_adr])
            qm=enc.step(qtrue)
            contacts=np.array([1 if c else 0 for c in s.contacts],dtype=np.uint8)
            # apply fault
            gm, am, qm, contacts = fault_fn(i, dt, gm, am, qm, contacts)
            gt_vel=getattr(puppet,"_prev_vel",np.zeros(3)).copy()
            w.write(LogRow(t=t, gyro=gm, accel=am, qj=qm, contacts=contacts, gt_contacts=np.array([1 if c else 0 for c in s.contacts],dtype=np.uint8),
                           gt_pos=s.base_pos.copy(), gt_quat=s.base_quat.copy(), gt_vel=gt_vel,
                           gt_rpy_rate=s.base_rpy_rate.copy(), gt_accel=s.base_accel.copy()))
            t+=dt
    # must not crash
    subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(otlg), str(estm)])
    # check finite and not insane
    from eval.evaluate import read_est
    _, est=read_est(str(estm))
    for r in est:
        assert np.all(np.isfinite(r['p'])) and np.all(np.isfinite(r['v']))
    dt1, gt_rows=read_log(str(otlg))
    p_err=np.array([np.linalg.norm(est[i]['p']-gt_rows[i].gt_pos) for i in range(len(gt_rows))])
    return float(np.sqrt(np.mean(p_err**2)))

def test_dropout_mid_run(tmp_path):
    def fault(i, dt, gm, am, qm, contacts):
        # drop all contacts for 0.2 s in middle
        if 400 <= i < 500:
            contacts[:] = 0
        return gm, am, qm, contacts
    rmse = run_with_fault(tmp_path, fault)
    # with dropout, error grows but should stay bounded (<0.5 m)
    assert rmse < 0.5

def test_imu_bias_jump(tmp_path):
    def fault(i, dt, gm, am, qm, contacts):
        if i == 500:
            gm = gm + np.array([0.1, 0, 0]) # 0.1 rad/s jump
        return gm, am, qm, contacts
    rmse = run_with_fault(tmp_path, fault)
    assert rmse < 0.5
    # also check that estimate stays finite after jump (already checked)

def test_stuck_encoder(tmp_path):
    frozen=None
    def fault(i, dt, gm, am, qm, contacts):
        nonlocal frozen
        if i == 300:
            frozen = qm.copy()
        if frozen is not None and i >= 300 and i < 400:
            qm = frozen
        return gm, am, qm, contacts
    rmse = run_with_fault(tmp_path, fault)
    assert rmse < 0.6
