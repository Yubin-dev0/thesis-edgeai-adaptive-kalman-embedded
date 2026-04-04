"""
gen_verify_csv.py — Generate high-precision CSV for C verification
===================================================================
The publication CSV rounds tof_distance_mm to 2 decimal places,
causing ~0.004mm input drift that propagates through KF recursion.

This script generates verification-only CSVs with 6-digit precision
on all float fields, using the same seed and parameters as the
original simulations.

Outputs:
    verify_fixed_kf.csv   — Fixed KF reference (high precision)
    verify_cm_akf.csv     — CM-AKF reference (high precision)

Author: Yubin
Date: 2026-04-02
"""

import numpy as np
import csv

# Identical parameters
dt = 0.005; N = 2000; v_true = 200.0
A = 1.0; B = dt; H = 1.0; Q = 1.0
R_initial = 400.0; R_MIN = 1.0; R_MAX = 10000.0; W = 20
sigma_process = 0.5; sigma_measurement = 20.0
np.random.seed(42)

# === Synthetic data (identical to kf_simulation_1D.py) ===
x_true = np.zeros(N); x_true[0] = 100.0
u_encoder = np.zeros(N)
z_tof = np.zeros(N)
z_tof[0] = x_true[0] + np.random.randn() * sigma_measurement

for k in range(1, N):
    x_true[k] = x_true[k-1] + v_true * dt + np.random.randn() * sigma_process
    u_encoder[k] = v_true + np.random.randn() * (sigma_process / dt)
    z_tof[k] = x_true[k] + np.random.randn() * sigma_measurement

# === Fixed KF ===
x_est = np.zeros(N); P_est = np.zeros(N)
K_gain = np.zeros(N); residual = np.zeros(N); innov_cov = np.zeros(N)
x_est[0] = z_tof[0]; P_est[0] = R_initial

for k in range(1, N):
    x_pred = A * x_est[k-1] + B * u_encoder[k]
    P_pred = A * P_est[k-1] * A + Q
    innov_cov[k] = P_pred + R_initial
    K_gain[k] = P_pred / (P_pred + R_initial)
    residual[k] = z_tof[k] - x_pred
    x_est[k] = x_pred + K_gain[k] * residual[k]
    P_est[k] = (1 - K_gain[k]) * P_pred

with open('verify_fixed_kf.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['timestamp_ms','tof_distance_mm','tof_signal_rate',
                'tof_range_status','us_distance_mm','encoder_distance_mm',
                'encoder_speed_mms','kf_estimate_mm','tof_residual',
                'tof_residual_var','tof_residual_mean','sensor_disagree',
                'tof_meas_rate','gt_distance_mm','R_label',
                'kalman_gain','innovation_cov','scenario_id'])
    enc_dist = np.zeros(N); enc_dist[0] = x_true[0]
    for k in range(1, N):
        enc_dist[k] = enc_dist[k-1] + u_encoder[k] * dt
    for k in range(N):
        w.writerow([
            k*5,
            f'{z_tof[k]:.6f}',
            '','','',
            f'{enc_dist[k]:.6f}',
            f'{u_encoder[k]:.6f}',
            f'{x_est[k]:.6f}',
            f'{residual[k]:.6f}',
            '','','','',
            f'{x_true[k]:.6f}',
            '',
            f'{K_gain[k]:.6f}',
            f'{innov_cov[k]:.6f}',
            'E0'
        ])
print(f"[Saved] verify_fixed_kf.csv (6-digit precision)")
print(f"  final x={x_est[-1]:.6f} K={K_gain[-1]:.6f}")

# === CM-AKF ===
x_est_a = np.zeros(N); P_est_a = np.zeros(N)
K_gain_a = np.zeros(N); residual_a = np.zeros(N); innov_cov_a = np.zeros(N)
R_adaptive = np.zeros(N)
x_est_a[0] = z_tof[0]; P_est_a[0] = R_initial; R_adaptive[0] = R_initial
res_buf = np.zeros(W); buf_idx = 0; buf_count = 0; R_cur = R_initial

for k in range(1, N):
    x_pred = A * x_est_a[k-1] + B * u_encoder[k]
    P_pred = A * P_est_a[k-1] * A + Q
    residual_a[k] = z_tof[k] - x_pred
    res_buf[buf_idx] = residual_a[k]
    buf_idx = (buf_idx + 1) % W
    buf_count = min(buf_count + 1, W)
    if buf_count >= W:
        res_sq_mean = np.mean(res_buf ** 2)
        R_cur = res_sq_mean - P_pred
        R_cur = np.clip(R_cur, R_MIN, R_MAX)
    R_adaptive[k] = R_cur
    innov_cov_a[k] = P_pred + R_cur
    K_gain_a[k] = P_pred / (P_pred + R_cur)
    x_est_a[k] = x_pred + K_gain_a[k] * residual_a[k]
    P_est_a[k] = (1 - K_gain_a[k]) * P_pred

with open('verify_cm_akf.csv', 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['timestamp_ms','tof_distance_mm','tof_signal_rate',
                'tof_range_status','us_distance_mm','encoder_distance_mm',
                'encoder_speed_mms','kf_estimate_mm','tof_residual',
                'tof_residual_var','tof_residual_mean','sensor_disagree',
                'tof_meas_rate','gt_distance_mm','R_label',
                'kalman_gain','innovation_cov','scenario_id'])
    enc_dist2 = np.zeros(N); enc_dist2[0] = x_true[0]
    for k in range(1, N):
        enc_dist2[k] = enc_dist2[k-1] + u_encoder[k] * dt
    for k in range(N):
        w.writerow([
            k*5,
            f'{z_tof[k]:.6f}',
            '','','',
            f'{enc_dist2[k]:.6f}',
            f'{u_encoder[k]:.6f}',
            f'{x_est_a[k]:.6f}',
            f'{residual_a[k]:.6f}',
            '','','','',
            f'{x_true[k]:.6f}',
            '',
            f'{K_gain_a[k]:.6f}',
            f'{innov_cov_a[k]:.6f}',
            'E0_AKF'
        ])
print(f"[Saved] verify_cm_akf.csv (6-digit precision)")
print(f"  final x={x_est_a[-1]:.6f} K={K_gain_a[-1]:.6f} R={R_adaptive[-1]:.6f}")
