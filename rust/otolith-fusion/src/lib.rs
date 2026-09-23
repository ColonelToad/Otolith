//! 15-state error-state MEKF + leg FK port (ADR-0006, M3).
//!
//! Transliteration target: `fusion/include/otolith/fusion.hpp` +
//! `fusion/src/fusion.cpp` (predict / update_legs) and `leg_kin`
//! (forward kinematics). Eigen -> nalgebra (M3 dependency).
//! Parity bar: M3 numbers within 1% through the unchanged eval harness.
