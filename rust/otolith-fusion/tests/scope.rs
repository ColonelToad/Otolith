//! Asserts the SCOPE of this crate's leg kinematics, so the gap is not implied.
//!
//! v0.5 added a 6-DoF `Chain` model to `fusion/src/leg_kin.cpp` and three robots
//! that use it (G1, Apollo, OP3). This crate did not follow. That is a deliberate
//! deferral, documented in `src/leg.rs`, and this test exists so the deferral is
//! visible: if someone adds a Chain path, this test tells them to widen the claim,
//! and if someone assumes Chain support, this test says plainly there is none.
//!
//! The claim it protects is narrow and checkable: float and fixed agree
//! bit-identically. That is true for Go2 and unverified for everything else, so
//! anything reading this crate as a general fixed-point mirror of the C++ is wrong.

use otolith_fusion::leg::{foot_pos_base, leg_geom};

#[test]
fn go2_planar_is_supported() {
    let leg = leg_geom("FL").expect("go2 FL geometry must be present");
    let p = foot_pos_base(&leg, 0.0, 0.9, -1.8);
    assert!(p[2] < -0.2 && p[2] > -0.35, "go2 planar FK regressed: {p:?}");
}

#[test]
fn only_go2_leg_names_exist() {
    // Go2's four legs. If a Chain robot were ever added, its names would appear
    // here too and this assertion would fail -- which is the intent: it forces a
    // deliberate edit rather than a silent widening of scope.
    for name in ["FL", "FR", "RL", "RR"] {
        assert!(leg_geom(name).is_ok(), "{name} should resolve");
    }
    // The bipeds' names must NOT resolve. Not a bug: they have no planar model.
    for name in ["left", "right"] {
        assert!(
            leg_geom(name).is_err(),
            "{name} resolved, but this crate has no Chain leg model. \
             If you added one, widen the scope claim in src/leg.rs too."
        );
    }
}
