//! 15-state error-state MEKF + leg FK port (ADR-0006, M3).
//!
//! Transliteration target: `fusion/include/otolith/fusion.hpp` +
//! `fusion/src/fusion.cpp` (predict / update_legs) and `leg_kin`
//! (forward kinematics). Eigen -> nalgebra (M3 dependency).
//! Parity bar: M3 numbers within 1% through the unchanged eval harness.
//!
//! Soundness: this crate is 100% safe Rust — enforced below. The Miri
//! gate lives in `otolith-transport` (raw-pointer protocol logic); here
//! there is no `unsafe` for Miri to check, so the compiler enforces it.

//! Forbid `unsafe` outright: filter math has no business with raw pointers.
//! Any future need must remove this attribute with a written reason.
#![forbid(unsafe_code)]

pub mod fusion;
pub mod leg;

pub use fusion::{FusionConfig, FusionEKF, FusionState, Mat15, Vec12};
pub use leg::{foot_pos_base, foot_pos_base_q, leg_geom, LegGeom};
