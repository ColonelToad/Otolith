"""Sensor noise models (pure numpy).

Each model is honest about what it emulates: white noise + slowly varying
bias (AR(1)) for the IMU, white noise for encoders. Contact flags are exact
by construction in the kinematic puppet (v0.1 baseline); a flip-noise knob
exists for later robustness experiments.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ImuNoise:
    """Gyroscope (rad/s) and accelerometer (m/s^2) error models.

    bias: first-order Gauss-Markov (AR(1)) — slowly wandering offset, the
    dominant real-world error the EKF must estimate or absorb.
    """
    sigma_gyro_white: float = 0.01       # rad/s
    sigma_gyro_bias: float = 0.001       # rad/s per step drive noise
    tau_gyro_bias: float = 200.0         # s, correlation time
    sigma_accel_white: float = 0.15      # m/s^2
    sigma_accel_bias: float = 0.02       # m/s^2 per step drive noise
    tau_accel_bias: float = 300.0        # s
    seed: int = 0

    rng: np.random.Generator = field(init=False, repr=False)
    gyro_bias: np.ndarray = field(init=False, repr=False)
    accel_bias: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

    def step(self, dt: float, gyro_true: np.ndarray,
             accel_true_body: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """true angular velocity (body) and true linear acceleration
        (body, gravity included) -> measured pair."""
        for bias, sigma_b, tau in ((self.gyro_bias, self.sigma_gyro_bias, self.tau_gyro_bias),
                                   (self.accel_bias, self.sigma_accel_bias, self.tau_accel_bias)):
            bias += (-dt / tau) * bias + sigma_b * np.sqrt(dt) * self.rng.standard_normal(3)
        gyro = gyro_true + self.gyro_bias + \
            self.sigma_gyro_white * self.rng.standard_normal(3) / np.sqrt(dt)
        accel = accel_true_body + self.accel_bias + \
            self.sigma_accel_white * self.rng.standard_normal(3) / np.sqrt(dt)
        return gyro, accel


@dataclass
class EncoderNoise:
    """Joint encoder angle noise (rad)."""
    sigma: float = 0.002
    seed: int = 1

    rng: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)

    def step(self, q_true: np.ndarray) -> np.ndarray:
        return q_true + self.sigma * self.rng.standard_normal(len(q_true))


def contacts_exact(contacts_true: np.ndarray) -> np.ndarray:
    """v0.1: contact flags are exact by construction (kinematic puppet)."""
    return contacts_true.copy()
