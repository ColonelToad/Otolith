"""Metrics helpers for Otolith eval: RMSE, NEES, jitter."""

from __future__ import annotations
import numpy as np


def rmse(a): return float(np.sqrt(np.mean(np.asarray(a)**2)))

def quat_angle_error_deg(q_est, q_gt):
    # q wxyz
    def qmul(a,b):
        aw,ax,ay,az=a; bw,bx,by,bz=b
        return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])
    def qconj(q):
        w,x,y,z=q; return np.array([w,-x,-y,-z])
    err=qmul(qconj(q_gt), q_est)
    err/=np.linalg.norm(err)
    return float(2*np.degrees(np.arccos(np.clip(abs(err[0]), -1, 1))))

def compute_nees(gt_pos, est_pos, cov_pos):
    """Position NEES: (err^T P^{-1} err) for 3D. cov_pos is 3x3."""
    err = est_pos - gt_pos
    try:
        inv = np.linalg.inv(cov_pos)
    except np.linalg.LinAlgError:
        return float("nan")
    return float(err @ inv @ err)

def chi2_bounds(dof, alpha=0.95):
    """Chi-square 95% interval for NEES with dof. Returns (low, high) for mean over N samples."""
    # Use scipy if available, else approx via Wilson-Hilferty
    try:
        from scipy.stats import chi2
        low = chi2.ppf((1-alpha)/2, dof)
        high = chi2.ppf(1-(1-alpha)/2, dof)
        return low, high
    except ImportError:
        # approx for dof=3: 95% interval ~ [0.35, 7.81] (mean 3)
        if dof==3:
            return 0.35, 7.81
        return 0, dof*3
