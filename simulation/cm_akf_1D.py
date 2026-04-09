"""
1D Rule-based Adaptive Kalman Filter — Covariance Matching
==============================================================
역할:
  1. 기존 Fixed KF 위에 Covariance Matching 기반 R 적응 로직 추가
  2. E0 합성 데이터에서 Fixed KF vs Rule-AKF 비교 검증
  3. 비교 그래프 생성 (논문 Figure 초안)
  4. CSV 출력 (18컬럼 포맷 유지, scenario=E0_AKF)

Rule-AKF 핵심 — Covariance Matching [Mehra, 1970]:
  이론: innovation r(k)의 공분산 S = H·P_pred·H' + R 이므로,
        R(k) = (1/W)·Σr(i)² − P(k|k−1)   (H=1 단순화)
  - 슬라이딩 윈도우 W=20으로 잔차 제곱 평균을 실시간 계산
  - R 클램핑: R_MIN=1.0, R_MAX=10000.0 (발산 방지)
  - AKF 문헌의 표준 baseline으로, TinyML-AKF와 공정 비교 가능

제안서 연결:
  - 제안서 1.3절: pseudo-label 생성에 동일 공식 사용
  - 제안서 슬라이드 9: Residual statistics → Update R(k)
  - TinyML과의 차이: CM은 잔차 통계만 사용 (사후 대응),
    TinyML은 signal_rate 등 선행 지표도 활용 (선제 대응)

완료 기준: E0 합성 데이터에서 Fixed KF vs Rule-AKF 비교 그래프 생성

Author: Yubin
Date: 2026-03-31
Scenario: E0_AKF (Python Simulation — Covariance Matching AKF)
"""

import numpy as np
import matplotlib.pyplot as plt
import csv

# ============================================================
# 1. PARAMETERS (Fixed KF와 동일한 기본 설정)
# ============================================================
dt = 0.005              # 5ms sampling period (200Hz)
N = 2000                # timesteps → 10초
v_true = 200.0          # mm/s

# KF model parameters
A = 1.0
B = dt
H = 1.0

# KF noise parameters (초기값)
Q = 1.0
R_initial = 400.0       # 초기 R (VL53L0X σ≈20mm → 20²=400)

# Rule-AKF 파라미터
R_MIN = 1.0             # R 하한 (제안서 명시)
R_MAX = 10000.0         # R 상한 (제안서 명시)
W = 20                  # 슬라이딩 윈도우 크기

# Covariance Matching 방식 [Mehra, 1970]:
#   이론: S = H·P_pred·H' + R  →  R = E[r²] − P_pred  (H=1)
#   구현: R(k) = (1/W)·Σr(i)² − P_pred(k)
#   R_MIN/R_MAX 클램핑으로 수치 안정성 보장

# 합성 데이터 노이즈
sigma_process = 0.5
sigma_measurement = 20.0

np.random.seed(42)      # 재현성 (Fixed KF와 동일 시드 → 동일 데이터)

# ============================================================
# 2. SYNTHETIC DATA GENERATION (Fixed KF와 완전 동일)
# ============================================================
x_true = np.zeros(N)
x_true[0] = 100.0

u_encoder = np.zeros(N)

z_tof = np.zeros(N)
z_tof[0] = x_true[0] + np.random.randn() * sigma_measurement

for k in range(1, N):
    x_true[k] = x_true[k-1] + v_true * dt + np.random.randn() * sigma_process
    u_encoder[k] = v_true + np.random.randn() * (sigma_process / dt)
    z_tof[k] = x_true[k] + np.random.randn() * sigma_measurement

encoder_distance = np.zeros(N)
encoder_distance[0] = x_true[0]
for k in range(1, N):
    encoder_distance[k] = encoder_distance[k-1] + u_encoder[k] * dt

# ============================================================
# 3-A. FIXED KF (Baseline — 기존 코드와 동일)
# ============================================================
x_est_fixed = np.zeros(N)
P_est_fixed = np.zeros(N)
K_gain_fixed = np.zeros(N)
residual_fixed = np.zeros(N)
innov_cov_fixed = np.zeros(N)

x_est_fixed[0] = z_tof[0]
P_est_fixed[0] = R_initial

R_fixed = R_initial  # 고정

for k in range(1, N):
    # PREDICT
    x_pred = A * x_est_fixed[k-1] + B * u_encoder[k]
    P_pred = A * P_est_fixed[k-1] * A + Q
    # UPDATE
    innov_cov_fixed[k] = P_pred + R_fixed
    K_gain_fixed[k] = P_pred / (P_pred + R_fixed)
    residual_fixed[k] = z_tof[k] - x_pred
    x_est_fixed[k] = x_pred + K_gain_fixed[k] * residual_fixed[k]
    P_est_fixed[k] = (1 - K_gain_fixed[k]) * P_pred

# ============================================================
# 3-B. RULE-BASED AKF (핵심 구현)
# ============================================================
x_est_akf = np.zeros(N)
P_est_akf = np.zeros(N)
K_gain_akf = np.zeros(N)
residual_akf = np.zeros(N)
innov_cov_akf = np.zeros(N)
R_adaptive = np.zeros(N)       # 매 스텝 적응된 R 기록

x_est_akf[0] = z_tof[0]
P_est_akf[0] = R_initial
R_adaptive[0] = R_initial

# 잔차 버퍼 (슬라이딩 윈도우용)
residual_buffer = np.zeros(W)
buf_idx = 0
buf_count = 0

# 현재 R 값
R_current = R_initial

for k in range(1, N):
    # ---- PREDICT ----
    x_pred = A * x_est_akf[k-1] + B * u_encoder[k]
    P_pred = A * P_est_akf[k-1] * A + Q

    # ---- RESIDUAL ----
    residual_akf[k] = z_tof[k] - x_pred

    # ---- 잔차 버퍼 업데이트 (링버퍼) ----
    residual_buffer[buf_idx] = residual_akf[k]
    buf_idx = (buf_idx + 1) % W
    buf_count = min(buf_count + 1, W)

    # ---- ADAPTIVE R UPDATE (Covariance Matching [Mehra, 1970]) ----
    if buf_count >= W:
        # 잔차 제곱 평균 = innovation 공분산의 샘플 추정
        # E[r²] ≈ (1/W) · Σ r(i)²
        res_sq_mean = np.mean(residual_buffer ** 2)

        # Covariance Matching: R = E[r²] − P_pred  (H=1 이므로 H·P·H' = P)
        R_current = res_sq_mean - P_pred

        # R 클램핑 (음수 방지 + 발산 방지)
        R_current = np.clip(R_current, R_MIN, R_MAX)

    R_adaptive[k] = R_current

    # ---- UPDATE (적응된 R 사용) ----
    innov_cov_akf[k] = P_pred + R_current
    K_gain_akf[k] = P_pred / (P_pred + R_current)
    x_est_akf[k] = x_pred + K_gain_akf[k] * residual_akf[k]
    P_est_akf[k] = (1 - K_gain_akf[k]) * P_pred

# ============================================================
# 4. DERIVED FIELDS
# ============================================================
# 잔차 윈도우 통계 (AKF)
tof_residual_var_akf = np.full(N, np.nan)
tof_residual_mean_akf = np.full(N, np.nan)
for k in range(W, N):
    window = residual_akf[k - W + 1 : k + 1]
    tof_residual_var_akf[k] = np.var(window)
    tof_residual_mean_akf[k] = np.mean(window)

# tof_meas_rate
tof_meas_rate = np.full(N, np.nan)
for k in range(1, N):
    tof_meas_rate[k] = z_tof[k] - z_tof[k - 1]

# R_label (Covariance Matching pseudo-label, 기존과 동일)
R_label = np.full(N, np.nan)
for k in range(W, N):
    window = residual_akf[k - W + 1 : k + 1]
    R_label[k] = np.var(window)

# ============================================================
# 5. PERFORMANCE METRICS
# ============================================================
rmse_sensor = np.sqrt(np.mean((z_tof - x_true) ** 2))
rmse_fixed = np.sqrt(np.mean((x_est_fixed - x_true) ** 2))
rmse_akf = np.sqrt(np.mean((x_est_akf - x_true) ** 2))

mae_sensor = np.mean(np.abs(z_tof - x_true))
mae_fixed = np.mean(np.abs(x_est_fixed - x_true))
mae_akf = np.mean(np.abs(x_est_akf - x_true))

# 수렴 시간 (RMSE가 ε=5mm 이하로 떨어지는 시간)
epsilon = 5.0
conv_window = 50  # 50스텝 연속으로 ε 이하면 수렴
def convergence_time(x_est, x_true, eps, win):
    for k in range(win, N):
        errors = np.abs(x_est[k-win+1:k+1] - x_true[k-win+1:k+1])
        if np.all(errors < eps):
            return k * dt
    return np.nan

conv_fixed = convergence_time(x_est_fixed, x_true, epsilon, conv_window)
conv_akf = convergence_time(x_est_akf, x_true, epsilon, conv_window)

print("=" * 65)
print("  Fixed KF vs Rule-AKF Comparison (E0 Synthetic Data)")
print("=" * 65)
print(f"  Total time:     {N * dt:.1f}s ({N} steps @ {1/dt:.0f}Hz)")
print(f"  True velocity:  {v_true} mm/s")
print(f"  Parameters:     Q={Q}, R_init={R_initial}, W={W}")
print(f"  AKF Method:     Covariance Matching [Mehra, 1970]")
print(f"  R clamp:        [{R_MIN}, {R_MAX}]")
print("-" * 65)
print(f"  {'Method':<20s} {'RMSE (mm)':>10s} {'MAE (mm)':>10s} {'Conv (s)':>10s}")
print("-" * 65)
print(f"  {'Raw Sensor':<20s} {rmse_sensor:>10.2f} {mae_sensor:>10.2f} {'N/A':>10s}")
print(f"  {'Fixed KF':<20s} {rmse_fixed:>10.2f} {mae_fixed:>10.2f} {conv_fixed:>10.3f}")
print(f"  {'CM-AKF':<20s} {rmse_akf:>10.2f} {mae_akf:>10.2f} {conv_akf:>10.3f}")
print("-" * 65)
print(f"  CM-AKF vs Fixed KF RMSE: {(1 - rmse_akf / rmse_fixed) * 100:+.1f}%")
print(f"  CM-AKF R range:  [{R_adaptive[W:].min():.1f}, {R_adaptive[W:].max():.1f}]")
print(f"  CM-AKF R mean:   {R_adaptive[W:].mean():.1f}")
print(f"  CM-AKF steady-state K: {K_gain_akf[-1]:.4f}")
print("=" * 65)

# ============================================================
# 6. VISUALIZATION (Fixed KF vs Rule-AKF 비교)
# ============================================================
time = np.arange(N) * dt

fig, axes = plt.subplots(4, 1, figsize=(14, 14),
                         gridspec_kw={'height_ratios': [3, 1.2, 1.2, 1.2]})
fig.suptitle('Fixed KF vs Covariance Matching AKF: E0 Synthetic Data',
             fontsize=14, fontweight='bold')

# --- Plot 1: 추정값 비교 ---
ax1 = axes[0]
ax1.plot(time, x_true, 'k-', linewidth=2, label='Ground truth', zorder=3)
ax1.scatter(time[::20], z_tof[::20], c='red', s=8, alpha=0.3,
            label='VL53L0X measurement', zorder=1)
ax1.plot(time, x_est_fixed, 'b-', linewidth=1.2, alpha=0.7,
         label=f'Fixed KF (RMSE={rmse_fixed:.2f}mm)', zorder=2)
ax1.plot(time, x_est_akf, 'g-', linewidth=1.2, alpha=0.8,
         label=f'CM-AKF (RMSE={rmse_akf:.2f}mm)', zorder=2)
ax1.set_ylabel('Position (mm)')
ax1.set_title(f'State Estimation: Sensor RMSE={rmse_sensor:.1f}mm → '
              f'Fixed={rmse_fixed:.2f}mm, AKF={rmse_akf:.2f}mm')
ax1.legend(loc='upper left', fontsize=9)
ax1.grid(True, alpha=0.3)

# --- Plot 2: 칼만 게인 비교 ---
ax2 = axes[1]
ax2.plot(time, K_gain_fixed, 'b-', linewidth=0.8, alpha=0.7, label='Fixed KF K')
ax2.plot(time, K_gain_akf, 'g-', linewidth=0.8, alpha=0.8, label='CM-AKF K')
ax2.set_ylabel('Kalman Gain K')
ax2.set_ylim(0, 0.6)
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

# --- Plot 3: 적응 R 변화 ---
ax3 = axes[2]
ax3.axhline(y=R_initial, color='b', linestyle='--', alpha=0.5,
            label=f'Fixed R={R_initial}')
ax3.plot(time, R_adaptive, 'g-', linewidth=1, label='CM-AKF R(k)')
ax3.set_ylabel('R value')
ax3.set_yscale('log')
ax3.set_ylim(R_MIN * 0.5, R_MAX * 2)
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)

# --- Plot 4: 잔차 비교 ---
ax4 = axes[3]
ax4.plot(time, residual_fixed, 'b-', linewidth=0.6, alpha=0.4, label='Fixed KF')
ax4.plot(time, residual_akf, 'g-', linewidth=0.6, alpha=0.5, label='CM-AKF')
ax4.axhline(y=0, color='k', linewidth=0.5)
ax4.axhline(y=2 * np.sqrt(R_initial), color='r', linestyle='--', alpha=0.3,
            label=f'±2√R₀ = ±{2 * np.sqrt(R_initial):.0f}mm')
ax4.axhline(y=-2 * np.sqrt(R_initial), color='r', linestyle='--', alpha=0.3)
ax4.set_ylabel('Residual (mm)')
ax4.set_xlabel('Time (s)')
ax4.legend(fontsize=9)
ax4.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('cm_akf_comparison.png', dpi=150, bbox_inches='tight')
plt.savefig('cm_akf_comparison.pdf', bbox_inches='tight')
print(f"\n[Saved] cm_akf_comparison.png / .pdf")

# ============================================================
# 7. CSV EXPORT (18컬럼 포맷, AKF 결과)
# ============================================================
csv_filename = 'cm_akf_simulation_data.csv'
with open(csv_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'timestamp_ms',        # 1
        'tof_distance_mm',     # 2
        'tof_signal_rate',     # 3  (빈 값)
        'tof_range_status',    # 4  (빈 값)
        'us_distance_mm',      # 5  (빈 값)
        'encoder_distance_mm', # 6
        'encoder_speed_mms',   # 7
        'kf_estimate_mm',      # 8
        'tof_residual',        # 9
        'tof_residual_var',    # 10
        'tof_residual_mean',   # 11
        'sensor_disagree',     # 12 (빈 값)
        'tof_meas_rate',       # 13
        'gt_distance_mm',      # 14
        'R_label',             # 15
        'kalman_gain',         # 16
        'innovation_cov',      # 17
        'scenario_id',         # 18
    ])
    for k in range(N):
        writer.writerow([
            k * 5,
            round(z_tof[k], 2),
            '',
            '',
            '',
            round(encoder_distance[k], 4),
            round(u_encoder[k], 4),
            round(x_est_akf[k], 4),
            round(residual_akf[k], 4),
            round(tof_residual_var_akf[k], 4) if not np.isnan(tof_residual_var_akf[k]) else '',
            round(tof_residual_mean_akf[k], 4) if not np.isnan(tof_residual_mean_akf[k]) else '',
            '',
            round(tof_meas_rate[k], 4) if not np.isnan(tof_meas_rate[k]) else '',
            round(x_true[k], 4),
            round(R_label[k], 4) if not np.isnan(R_label[k]) else '',
            round(K_gain_akf[k], 6),
            round(innov_cov_akf[k], 4),
            'E0_AKF',
        ])

print(f"[Saved] {csv_filename} (18 columns, {N} rows, scenario=E0_AKF)")
print(f"\nAll outputs: cm_akf_comparison.png, .pdf, {csv_filename}")
