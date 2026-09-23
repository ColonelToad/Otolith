//! 15-state error-state MEKF — port of `fusion/src/fusion.cpp`.
//!
//! Nominal `q (body->world) / p / v / bg / ba + P(15x15)`; error
//! `[dtheta(3), dv(3), dp(3), dbg(3), dba(3)]`. Eigen -> nalgebra,
//! transliterated statement-for-statement against `fusion.cpp` so the
//! 1% parity bar (ADR-0006 M4) tests algebra, not interpretation.
//! Deliberate deviations from C++: none in math; `contacts: [u8; 4]`,
//! `leg_geom` returns `Result` (no exceptions).

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, UnitQuaternion, Vector3};

use super::leg::{foot_pos_base, leg_geom};

pub type Mat15 = SMatrix<f64, 15, 15>;
pub type Vec12 = SVector<f64, 12>;

#[derive(Clone, Copy, Debug)]
pub struct FusionConfig {
    pub sigma_gyro: f64,
    pub sigma_accel: f64,
    pub sigma_bg_rw: f64,
    pub sigma_ba_rw: f64,
    pub sigma_leg_vel: f64,
    pub gravity: f64,
}

impl Default for FusionConfig {
    fn default() -> Self {
        Self {
            sigma_gyro: 0.01,
            sigma_accel: 0.15,
            sigma_bg_rw: 1e-5,
            sigma_ba_rw: 1e-4,
            sigma_leg_vel: 0.3,
            gravity: 9.81,
        }
    }
}

#[derive(Clone, Debug)]
pub struct FusionState {
    pub q: UnitQuaternion<f64>,
    pub p: Vector3<f64>,
    pub v: Vector3<f64>,
    pub bg: Vector3<f64>,
    pub ba: Vector3<f64>,
    pub p_cov: Mat15,
}

pub fn make_default_state() -> FusionState {
    let mut p_cov = Mat15::identity() * 1e-2;
    p_cov.fixed_view_mut::<3, 3>(0, 0).scale_mut(10.0); // dtheta
    p_cov.fixed_view_mut::<3, 3>(3, 3).scale_mut(10.0); // dv
    FusionState {
        q: UnitQuaternion::identity(),
        p: Vector3::zeros(),
        v: Vector3::zeros(),
        bg: Vector3::zeros(),
        ba: Vector3::zeros(),
        p_cov,
    }
}

pub fn skew(w: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -w.z, w.y, w.z, 0.0, -w.x, -w.y, w.x, 0.0)
}

pub fn quat_to_mat(q: &UnitQuaternion<f64>) -> Matrix3<f64> {
    q.to_rotation_matrix().matrix().clone_owned()
}

fn exp_quat(w_dt: &Vector3<f64>) -> UnitQuaternion<f64> {
    let angle = w_dt.norm();
    if angle < 1e-12 {
        return UnitQuaternion::identity();
    }
    let h = angle * 0.5;
    let s = h.sin();
    let axis = w_dt / angle;
    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        h.cos(),
        axis.x * s,
        axis.y * s,
        axis.z * s,
    ))
}

#[derive(Clone, Debug)]
pub struct FusionEKF {
    cfg: FusionConfig,
    state: FusionState,
    prev_qj: Vec12,
    has_prev: bool,
}

const LEG_NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];

impl FusionEKF {
    pub fn new(cfg: FusionConfig, state: FusionState) -> Self {
        Self {
            cfg,
            state,
            prev_qj: Vec12::zeros(),
            has_prev: false,
        }
    }

    pub fn state(&self) -> &FusionState {
        &self.state
    }

    pub fn set_state(&mut self, s: FusionState) {
        self.state = s;
    }

    /// IMU propagation: gyro/accel in body frame (gravity included).
    pub fn predict(&mut self, dt: f64, gyro_m: &Vector3<f64>, accel_m: &Vector3<f64>) {
        let w = gyro_m - self.state.bg;
        let a = accel_m - self.state.ba;
        let r = quat_to_mat(&self.state.q);
        let g = Vector3::new(0.0, 0.0, -self.cfg.gravity);

        // --- nominal propagation (first-order, semi-implicit p) ---
        self.state.q *= exp_quat(&(w * dt));
        self.state.v += (r * a + g) * dt;
        self.state.p += self.state.v * dt;

        // --- covariance: Phi = I + F*dt ---
        let mut f = Mat15::zeros();
        f.fixed_view_mut::<3, 3>(0, 0).copy_from(&-skew(&w));
        f.fixed_view_mut::<3, 3>(0, 9)
            .copy_from(&-Matrix3::<f64>::identity());
        f.fixed_view_mut::<3, 3>(3, 0).copy_from(&(-r * skew(&a)));
        f.fixed_view_mut::<3, 3>(3, 12).copy_from(&-r);
        f.fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&Matrix3::<f64>::identity());
        let phi = Mat15::identity() + f * dt;

        // Qd = G Q G^T dt (diagonal blocks)
        let mut qd = Mat15::zeros();
        let (s_g, s_a, s_bg, s_ba) = (
            self.cfg.sigma_gyro,
            self.cfg.sigma_accel,
            self.cfg.sigma_bg_rw,
            self.cfg.sigma_ba_rw,
        );
        qd.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::<f64>::identity() * s_g * s_g * dt));
        qd.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(Matrix3::<f64>::identity() * s_a * s_a * dt));
        qd.fixed_view_mut::<3, 3>(9, 9)
            .copy_from(&(Matrix3::<f64>::identity() * s_bg * s_bg * dt));
        qd.fixed_view_mut::<3, 3>(12, 12)
            .copy_from(&(Matrix3::<f64>::identity() * s_ba * s_ba * dt));

        self.state.p_cov = phi * self.state.p_cov * phi.transpose() + qd;
        // enforce symmetry
        self.state.p_cov = (self.state.p_cov + self.state.p_cov.transpose()) * 0.5;
    }

    /// Leg-odometry update. Returns stance feet used (0 => no update).
    /// `dt` is the sample period (r_dot via finite difference).
    pub fn update_legs(
        &mut self,
        qj: &Vec12,
        contacts: &[u8; 4],
        gyro_m: &Vector3<f64>,
        dt: f64,
    ) -> usize {
        let stance: Vec<usize> = (0..4).filter(|&i| contacts[i] != 0).collect();
        if stance.is_empty() || !self.has_prev {
            self.prev_qj = *qj;
            self.has_prev = true;
            return 0;
        }

        let r = quat_to_mat(&self.state.q);
        let w = gyro_m - self.state.bg;
        let m = stance.len();
        let rows = 3 * m;
        let mut h = DMatrix::<f64>::zeros(rows, 15);
        let mut y = DVector::<f64>::zeros(rows);
        let mut rmat = DMatrix::<f64>::zeros(rows, rows);
        for (k, &leg) in stance.iter().enumerate() {
            let lg = leg_geom(LEG_NAMES[leg]).expect("known leg");
            let qleg = qj.fixed_rows::<3>(leg * 3).clone_owned();
            let r_base = foot_pos_base(&lg, qleg[0], qleg[1], qleg[2]);
            let qprev = self.prev_qj.fixed_rows::<3>(leg * 3).clone_owned();
            let r_prev = foot_pos_base(&lg, qprev[0], qprev[1], qprev[2]);
            let r_dot = (r_base - r_prev) / dt;
            let omega_cross_r = w.cross(&r_base);
            let hh = self.state.v + r * (omega_cross_r + r_dot);
            y.fixed_rows_mut::<3>(k * 3).copy_from(&-hh);

            // Jacobian: H = [-R[omega x r]_x, I, 0, R[r]_x, 0]
            let wr = skew(&omega_cross_r);
            let rr = skew(&r_base);
            h.fixed_view_mut::<3, 3>(k * 3, 0).copy_from(&(-r * wr));
            h.fixed_view_mut::<3, 3>(k * 3, 3)
                .copy_from(&Matrix3::<f64>::identity());
            // dp block 0
            h.fixed_view_mut::<3, 3>(k * 3, 9).copy_from(&(r * rr));
            // dba block 0

            let sigma = self.cfg.sigma_leg_vel;
            rmat.fixed_view_mut::<3, 3>(k * 3, k * 3)
                .copy_from(&(Matrix3::<f64>::identity() * sigma * sigma));
        }

        // Kalman gain
        let p_dyn = DMatrix::from_row_slice(15, 15, self.state.p_cov.as_slice());
        let s = &h * &p_dyn * h.transpose() + &rmat;
        let k = &p_dyn * h.transpose() * s.try_inverse().expect("S invertible");

        let dx = &k * &y;
        let dtheta = dx.fixed_rows::<3>(0).clone_owned();
        let dv = dx.fixed_rows::<3>(3).clone_owned();
        let dp = dx.fixed_rows::<3>(6).clone_owned();
        let dbg = dx.fixed_rows::<3>(9).clone_owned();
        let dba = dx.fixed_rows::<3>(12).clone_owned();

        self.state.q *= exp_quat(&dtheta);
        self.state.v += dv;
        self.state.p += dp;
        self.state.bg += dbg;
        self.state.ba += dba;

        // Joseph form for stability
        let kh = &k * &h; // 15x15 dynamic
        let kh_fix = Mat15::from_iterator(kh.iter().cloned());
        let a_mat = Mat15::identity() - kh_fix;
        let krk = &k * &rmat * k.transpose();
        let krk_fix = Mat15::from_iterator(krk.iter().cloned());
        self.state.p_cov = a_mat * self.state.p_cov * a_mat.transpose() + krk_fix;
        self.state.p_cov = (self.state.p_cov + self.state.p_cov.transpose()) * 0.5;
        self.prev_qj = *qj;
        self.has_prev = true;
        m
    }
}

impl Default for FusionEKF {
    fn default() -> Self {
        Self::new(FusionConfig::default(), make_default_state())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::SymmetricEigen;

    fn home_qj() -> Vec12 {
        let mut qj = Vec12::zeros();
        for leg in 0..4 {
            qj[leg * 3 + 1] = 0.9;
            qj[leg * 3 + 2] = -1.8;
        }
        qj
    }

    fn min_eig(p: &Mat15) -> f64 {
        SymmetricEigen::new(*p)
            .eigenvalues
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min)
    }

    fn all_finite_q(q: &UnitQuaternion<f64>) -> bool {
        [q.i, q.j, q.k, q.w].iter().all(|x| x.is_finite())
    }

    #[test]
    fn predict_no_motion_keeps_state_grows_cov() {
        let mut ekf = FusionEKF::default();
        let tr0 = ekf.state().p_cov.trace();
        ekf.predict(0.002, &Vector3::zeros(), &Vector3::new(0.0, 0.0, 9.81));
        let s1 = ekf.state();
        assert!(s1.q.angle_to(&UnitQuaternion::identity()) < 1e-6);
        assert!(s1.p.norm() < 1e-4);
        assert!(s1.p_cov.trace() > tr0);
        assert!(min_eig(&s1.p_cov) >= -1e-9);
    }

    #[test]
    fn predict_constant_yaw() {
        let mut ekf = FusionEKF::default();
        ekf.predict(
            0.01,
            &Vector3::new(0.0, 0.0, 0.5),
            &Vector3::new(0.0, 0.0, 9.81),
        );
        let (_, angle) = ekf.state().q.axis_angle().expect("nonzero rotation");
        assert!((angle - 0.005).abs() < 1e-4, "angle={angle}");
    }

    #[test]
    fn update_reduces_covariance_in_stance() {
        let mut ekf = FusionEKF::default();
        ekf.predict(0.002, &Vector3::zeros(), &Vector3::new(0.0, 0.0, 9.81));
        let qj = home_qj();
        let contacts = [1, 0, 0, 1];
        ekf.update_legs(&qj, &contacts, &Vector3::zeros(), 0.002); // primes r_dot
        let tr_before = ekf.state().p_cov.trace();
        let m = ekf.update_legs(&qj, &contacts, &Vector3::zeros(), 0.002);
        assert_eq!(m, 2);
        let tr_after = ekf.state().p_cov.trace();
        assert!(tr_after < tr_before, "{tr_after} < {tr_before}");
        assert!(min_eig(&ekf.state().p_cov) >= -1e-9);
    }

    #[test]
    fn survives_many_steps_psd() {
        let mut ekf = FusionEKF::default();
        let qj = home_qj();
        for i in 0..500 {
            ekf.predict(
                0.002,
                &Vector3::new(0.01, -0.01, 0.02),
                &Vector3::new(0.05, -0.03, 9.82),
            );
            if i % 5 == 0 {
                let c = if i % 10 == 5 {
                    [0, 1, 0, 1]
                } else {
                    [1, 0, 1, 0]
                };
                ekf.update_legs(&qj, &c, &Vector3::new(0.01, -0.01, 0.02), 0.002);
            }
            assert!(all_finite_q(&ekf.state().q));
            assert!(ekf.state().v.iter().all(|x| x.is_finite()));
        }
        assert!(min_eig(&ekf.state().p_cov) >= -1e-8);
    }

    #[test]
    fn zero_stance_no_update() {
        let mut ekf = FusionEKF::default();
        ekf.predict(0.002, &Vector3::zeros(), &Vector3::new(0.0, 0.0, 9.81));
        let q_before = ekf.state().q;
        let m = ekf.update_legs(&Vec12::zeros(), &[0, 0, 0, 0], &Vector3::zeros(), 0.002);
        assert_eq!(m, 0);
        let q_after = ekf.state().q;
        assert_eq!(
            [q_after.i, q_after.j, q_after.k, q_after.w],
            [q_before.i, q_before.j, q_before.k, q_before.w]
        );
    }

    /// Cross-language differential: identical 200-step input sequence
    /// through the C++ filter (goldens from `/tmp` `golden.cpp`, 17
    /// digits) and this port. Tolerance is tight (1e-9 relative) because
    /// the transliteration is statement-for-statement; the M4 bar is 1%,
    /// three orders looser. Any genuine algebra divergence fails here
    /// long before it can hide in eval noise.
    ///
    /// The 17-digit literals intentionally exceed f64 precision (clippy's
    /// `excessive_precision` is allowed here): they are verbatim C++
    /// output, kept exact for auditability.
    #[test]
    #[allow(clippy::excessive_precision)]
    fn differential_vs_cpp_golden() {
        let mut ekf = FusionEKF::default();
        let mut qj = Vec12::zeros();
        for leg in 0..4 {
            qj[leg * 3 + 1] = 0.9;
            qj[leg * 3 + 2] = -1.8;
        }
        let gm = Vector3::new(0.01, -0.01, 0.02);
        let am = Vector3::new(0.05, -0.03, 9.82);
        let mut updates = 0;
        for i in 0..200 {
            ekf.predict(0.002, &gm, &am);
            if i % 5 == 0 {
                let c = if i % 10 == 5 {
                    [0, 1, 0, 1]
                } else {
                    [1, 0, 1, 0]
                };
                updates += ekf.update_legs(&qj, &c, &gm, 0.002);
            }
        }
        assert_eq!(updates, 78);
        let s = ekf.state();
        let close = |a: f64, b: f64, what: &str| {
            let tol = 1e-9 * b.abs().max(1e-12);
            assert!(
                (a - b).abs() <= tol,
                "{what}: rust={a:.17} cpp={b:.17} diff={:.3e}",
                (a - b).abs()
            );
        };
        // C++ Eigen (x,y,z,w) vs nalgebra (i,j,k,w): same order.
        let q = [s.q.i, s.q.j, s.q.k, s.q.w];
        let gq = [
            -0.00063928624199029041,
            -0.0032822723195941195,
            0.0026738468687152288,
            0.99999083423021606,
        ];
        for (i, (&a, &b)) in q.iter().zip(gq.iter()).enumerate() {
            close(a, b, &format!("q[{i}]"));
        }
        for (i, (&a, &b)) in
            s.p.iter()
                .zip(
                    [
                        -0.00079708180932707502,
                        -0.00087938620535805599,
                        1.6637204045778209e-05,
                    ]
                    .iter(),
                )
                .enumerate()
        {
            close(a, b, &format!("p[{i}]"));
        }
        for (i, (&a, &b)) in
            s.v.iter()
                .zip(
                    [
                        -0.0028862008720301393,
                        -0.003292990618100314,
                        0.0012641206013093533,
                    ]
                    .iter(),
                )
                .enumerate()
        {
            close(a, b, &format!("v[{i}]"));
        }
        for (i, (&a, &b)) in
            s.bg.iter()
                .zip(
                    [
                        0.0018140357832161953,
                        -0.0025641057476284809,
                        0.0066404224961667856,
                    ]
                    .iter(),
                )
                .enumerate()
        {
            close(a, b, &format!("bg[{i}]"));
        }
        for (i, (&a, &b)) in
            s.ba.iter()
                .zip(
                    [
                        3.8994521946488369e-05,
                        -4.7879924785508365e-05,
                        0.00061657037413447621,
                    ]
                    .iter(),
                )
                .enumerate()
        {
            close(a, b, &format!("ba[{i}]"));
        }
        close(s.p_cov.trace(), 0.2048251027144061, "trace");
        let gdiag = [
            0.0020712882114719208,
            0.0020274089183418961,
            0.10096060947166428,
            0.0069341539307581175,
            0.0070739989417533773,
            0.0034302446789853634,
            0.01026395218652482,
            0.010273949640757387,
            0.010182630199643817,
            0.0082304674804016056,
            0.0073272259655473839,
            0.0066731101511338939,
            0.0099898155470293515,
            0.0099898254203122025,
            0.0093964219700806602,
        ];
        for (i, &b) in gdiag.iter().enumerate() {
            close(s.p_cov[(i, i)], b, &format!("P[{i},{i}]"));
        }
    }
}
