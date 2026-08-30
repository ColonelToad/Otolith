# Otolith M3 — first honest RMSE (trot, 500 Hz, IMU+leg odometry)

*Generated from `/tmp/otolith_auto.otlg` → `/tmp/otolith_auto.estm` — 2500 samples, dt=0.0020s, duration 5.00s*

| metric | RMSE | per-axis (x / y / z) |
|---|---|---|
| position (m) | `0.1064` | `0.0211 / 0.1042 / 0.0040` |
| velocity (m/s) | `0.0674` | `0.0306 / 0.0532 / 0.0277` |
| attitude (deg) | `13.1083` | — |
| final pos err | `0.2067 m` | drift `20.68%` of travel (1.00 m) |

Raw: p_err mean 0.0877 max 0.2067, v_err max 0.1392, ang max 19.74 deg.

*Notes:* puppet is kinematic trot with world-fixed footholds + AR(1) IMU bias; estimator is MEKF@500 Hz init from first GT, zero-alloc fixed-size Eigen. Measurement noise `σ_leg=0.3 m/s` per foot (inflated to cover encoder-noise-amplified r_dot via finite difference). This is the baseline to beat — y drift and yaw remain the weak axes.
