"""Scenario suite: N seeded trots, RMSE bounds."""
import subprocess, struct, sys
from pathlib import Path
import pytest
import numpy as np

ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from otolith_sim.logger import record_puppet_log, read_log

EST_HDR = struct.Struct("<4s I I I d Q")

def read_est(path):
    from eval.evaluate import read_est
    return read_est(path)

def rmse(a): return np.sqrt(np.mean(np.asarray(a)**2))

@pytest.mark.parametrize("seed", [0,1,2])
def test_trot_rmse_bounds(tmp_path, seed):
    # 2 sec trot, default noise, seed varies imu/enc bias seeds
    otlg = tmp_path / f"sc_{seed}.otlg"
    estm = tmp_path / f"sc_{seed}.estm"
    # record with seed offset
    import mujoco
    from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
    from otolith_sim.sensors import ImuNoise, EncoderNoise
    from otolith_sim.logger import LogWriter, LogRow
    m=mujoco.MjModel.from_xml_path(str(ROOT / "third_party/menagerie/unitree_go2/scene.xml"))
    d=mujoco.MjData(m)
    puppet=Go2Puppet(m, GaitConfig())
    imu=ImuNoise(seed=seed)
    enc=EncoderNoise(seed=seed+100)
    dt=1/500
    with LogWriter(otlg, dt) as w:
        t=0
        for _ in range(1000):
            s=puppet.sample(m,d,t,dt)
            R=_quat_to_mat(s.base_quat)
            abody=R.T@(s.base_accel+np.array([0,0,GRAVITY]))
            gm,am=imu.step(dt, s.base_rpy_rate, abody)
            qtrue=np.array([s.qpos[adr] for leg in ("FL","FR","RL","RR") for adr in puppet.legs[leg].qpos_adr])
            qm=enc.step(qtrue)
            contacts=np.array([1 if c else 0 for c in s.contacts],dtype=np.uint8)
            gt_vel=getattr(puppet,"_prev_vel",np.zeros(3)).copy()
            w.write(LogRow(t=t, gyro=gm, accel=am, qj=qm, contacts=contacts, gt_contacts=contacts.copy(),
                           gt_pos=s.base_pos.copy(), gt_quat=s.base_quat.copy(), gt_vel=gt_vel,
                           gt_rpy_rate=s.base_rpy_rate.copy(), gt_accel=s.base_accel.copy()))
            t+=dt
    subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(otlg), str(estm)])
    dt_gt, gt_rows = read_log(str(otlg))
    dt_e, est_rows = read_est(str(estm))
    assert len(gt_rows)==len(est_rows)
    p_err=np.array([np.linalg.norm(est_rows[i]['p']-gt_rows[i].gt_pos) for i in range(len(gt_rows))])
    v_err=np.array([np.linalg.norm(est_rows[i]['v']-gt_rows[i].gt_vel) for i in range(len(gt_rows))])
    # bounds: loose enough to be stable across seeds, tight enough to catch regressions
    # M3 baseline ~0.03-0.10 pos, 0.06 vel; allow 2x
    assert rmse(p_err) < 0.25, f"pos rmse {rmse(p_err)}"
    assert rmse(v_err) < 0.30, f"vel rmse {rmse(v_err)}"
    # final drift < 50% of travel (travel ~0.4 m for 2 sec)
    dist=np.linalg.norm(gt_rows[-1].gt_pos - gt_rows[0].gt_pos)
    assert p_err[-1] < 0.5*max(dist,0.1)

def test_novel_gait_still_bounded(tmp_path):
    """Slightly faster stride should not blow up."""
    import mujoco
    from otolith_sim.puppet import Go2Puppet, GaitConfig, _quat_to_mat, GRAVITY
    from otolith_sim.sensors import ImuNoise, EncoderNoise
    from otolith_sim.logger import LogWriter, LogRow
    m=mujoco.MjModel.from_xml_path(str(ROOT/"third_party/menagerie/unitree_go2/scene.xml"))
    d=mujoco.MjData(m)
    puppet=Go2Puppet(m, GaitConfig(stride=0.18)) # faster
    imu=ImuNoise(seed=7); enc=EncoderNoise(seed=8)
    otlg=tmp_path/"fast.otlg"; estm=tmp_path/"fast.estm"
    dt=1/500
    with LogWriter(otlg, dt) as w:
        t=0
        for _ in range(1000):
            s=puppet.sample(m,d,t,dt)
            R=_quat_to_mat(s.base_quat)
            abody=R.T@(s.base_accel+np.array([0,0,GRAVITY]))
            gm,am=imu.step(dt, s.base_rpy_rate, abody)
            qtrue=np.array([s.qpos[adr] for leg in ("FL","FR","RL","RR") for adr in puppet.legs[leg].qpos_adr])
            qm=enc.step(qtrue)
            contacts=np.array([1 if c else 0 for c in s.contacts],dtype=np.uint8)
            gt_vel=getattr(puppet,"_prev_vel",np.zeros(3)).copy()
            w.write(LogRow(t=t, gyro=gm, accel=am, qj=qm, contacts=contacts, gt_contacts=contacts.copy(),
                           gt_pos=s.base_pos.copy(), gt_quat=s.base_quat.copy(), gt_vel=gt_vel,
                           gt_rpy_rate=s.base_rpy_rate.copy(), gt_accel=s.base_accel.copy()))
            t+=dt
    subprocess.check_call([str(ROOT/"fusion/build/fuse_log"), str(otlg), str(estm)])
    from otolith_sim.logger import read_log
    dt1, rows=read_log(str(otlg))
    from eval.evaluate import read_est
    _, est=read_est(str(estm))
    p_err=np.array([np.linalg.norm(est[i]['p']-rows[i].gt_pos) for i in range(len(rows))])
    assert rmse(p_err) < 0.30
