//! Rust `fuse_log` — CLI twin of `fusion/src/fuse_log.cpp` (M4).
//!
//! usage: fuse_log <in.otlg> <out.estm> [dt_override] [--no-leg-update]
//! Identical behavior: init from first GT, predict per row, contact
//! updates unless `--no-leg-update` (predict-only dead reckoning),
//! ESTM-v2 output with row-major covariance.

use std::process::ExitCode;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use otolith_fusion::estm::{write_estimate_v2, EstRowV2};
use otolith_fusion::fusion::{FusionEKF, Vec12};
use otolith_fusion::log::read_log;

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().collect();
    if argv.len() < 3 {
        eprintln!("usage: fuse_log <in.otlg> <out.estm> [dt_override] [--no-leg-update]");
        eprintln!("  --no-leg-update: predict-only dead reckoning (IMU, no contact updates)");
        return ExitCode::from(2);
    }
    let (inp, outp) = (argv[1].clone(), argv[2].clone());
    let mut no_leg_update = false;
    let mut dt_override: Option<f64> = None;
    for a in argv.iter().skip(3) {
        if a == "--no-leg-update" {
            no_leg_update = true;
        } else {
            match a.parse::<f64>() {
                Ok(dt) => dt_override = Some(dt),
                Err(_) => {
                    eprintln!("bad arg {a}");
                    return ExitCode::from(2);
                }
            }
        }
    }

    let lf = match read_log(&inp) {
        Ok(lf) => lf,
        Err(e) => {
            eprintln!("fuse_log error: {e}");
            return ExitCode::FAILURE;
        }
    };
    let dt = dt_override.unwrap_or(lf.dt);

    let mut ekf = FusionEKF::default();
    // Init from first GT to avoid huge initial transient dominating RMSE.
    if let Some(r0) = lf.rows.first() {
        let mut s = ekf.state().clone();
        s.q = UnitQuaternion::from_quaternion(Quaternion::new(
            r0.gt_quat[0],
            r0.gt_quat[1],
            r0.gt_quat[2],
            r0.gt_quat[3],
        ));
        s.p = Vector3::new(r0.gt_pos[0], r0.gt_pos[1], r0.gt_pos[2]);
        s.v = Vector3::new(r0.gt_vel[0], r0.gt_vel[1], r0.gt_vel[2]);
        s.bg = Vector3::zeros();
        s.ba = Vector3::zeros();
        ekf.set_state(s);
    }

    let mut est = Vec::with_capacity(lf.rows.len());
    for row in &lf.rows {
        let gyro = Vector3::new(row.gyro[0], row.gyro[1], row.gyro[2]);
        let accel = Vector3::new(row.accel[0], row.accel[1], row.accel[2]);
        ekf.predict(dt, &gyro, &accel);

        if !no_leg_update {
            let mut qj = Vec12::zeros();
            for i in 0..12 {
                qj[i] = row.qj[i];
            }
            ekf.update_legs(&qj, &row.contacts, &gyro, dt);
        }

        let st = ekf.state();
        let mut p_cov = [0.0; 225];
        for r in 0..15 {
            for c in 0..15 {
                p_cov[r * 15 + c] = st.p_cov[(r, c)];
            }
        }
        est.push(EstRowV2 {
            t: row.t,
            p: [st.p.x, st.p.y, st.p.z],
            quat: [st.q.w, st.q.i, st.q.j, st.q.k],
            v: [st.v.x, st.v.y, st.v.z],
            bg: [st.bg.x, st.bg.y, st.bg.z],
            ba: [st.ba.x, st.ba.y, st.ba.z],
            p_cov_row_major: p_cov,
        });
    }
    if let Err(e) = write_estimate_v2(&outp, dt, &est) {
        eprintln!("fuse_log error: {e}");
        return ExitCode::FAILURE;
    }
    println!(
        "fuse_log: {} rows -> {} estimates, dt={dt}{}",
        lf.rows.len(),
        est.len(),
        if no_leg_update {
            " (predict-only, no leg updates)"
        } else {
            ""
        }
    );
    ExitCode::SUCCESS
}
