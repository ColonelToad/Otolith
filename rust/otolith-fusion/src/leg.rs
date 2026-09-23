//! Go2 leg forward kinematics — port of `fusion/src/leg_kin.cpp`.
//!
//! Planar 2R + hip roll; constants introspected from
//! `menagerie/unitree_go2/scene.xml` (see `leg_kin.hpp`).

use nalgebra::Vector3;

#[derive(Clone, Copy, Debug)]
pub struct LegGeom {
    pub hip_base: Vector3<f64>,
    pub side: i32, // +1 left, -1 right
    pub a_offset: f64,
    pub x_offset: f64, // 0 for Go2 (kept for contract parity)
    pub l1: f64,
    pub l2: f64,
}

pub fn leg_geom(name: &str) -> Result<LegGeom, String> {
    let (hip_base, side) = match name {
        "FL" => (Vector3::new(0.1934, 0.0465, 0.0), 1),
        "FR" => (Vector3::new(0.1934, -0.0465, 0.0), -1),
        "RL" => (Vector3::new(-0.1934, 0.0465, 0.0), 1),
        "RR" => (Vector3::new(-0.1934, -0.0465, 0.0), -1),
        _ => return Err(format!("unknown leg {name}")),
    };
    Ok(LegGeom {
        hip_base,
        side,
        a_offset: 0.0955,
        x_offset: 0.0,
        l1: 0.213,
        l2: 0.21300938946440834,
    })
}

pub fn foot_pos_base(leg: &LegGeom, hip: f64, thigh: f64, calf: f64) -> Vector3<f64> {
    // Planar 2R: x_in = -L1 sin(thigh) - L2 sin(thigh+calf)
    //           z_in = -L1 cos(thigh) - L2 cos(thigh+calf)
    // y_in = side*a_offset (constant leg-plane offset)
    let x_in = -leg.l1 * thigh.sin() - leg.l2 * (thigh + calf).sin();
    let z_in = -leg.l1 * thigh.cos() - leg.l2 * (thigh + calf).cos();
    let y_in = leg.side as f64 * leg.a_offset;
    let (c, s) = (hip.cos(), hip.sin());
    let dy = c * y_in - s * z_in;
    let dz = s * y_in + c * z_in;
    leg.hip_base + Vector3::new(x_in, dy, dz)
}

pub fn foot_pos_base_q(leg: &LegGeom, q: &Vector3<f64>) -> Vector3<f64> {
    foot_pos_base(leg, q[0], q[1], q[2])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fk_home_plausible() {
        // Mirrors test_fusion.cpp: home joints -> foot z in [-0.35, -0.2].
        let leg = leg_geom("FL").unwrap();
        let p = foot_pos_base(&leg, 0.0, 0.9, -1.8);
        assert!(p.z < -0.2 && p.z > -0.35, "z={}", p.z);
    }

    #[test]
    fn fk_perturb_smooth() {
        let leg = leg_geom("FL").unwrap();
        let p = foot_pos_base(&leg, 0.0, 0.9, -1.8);
        let p2 = foot_pos_base(&leg, 0.0, 1.0, -1.8);
        assert!((p2 - p).norm() > 1e-3);
    }

    #[test]
    fn unknown_leg_errors() {
        assert!(leg_geom("XX").is_err());
    }
}
