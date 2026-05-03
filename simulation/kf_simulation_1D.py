"""
1D Kalman Filter Simulation — Unified Script
==============================================
역할:
  1. 1D 선형 KF Prediction/Update 루프 구현
  2. 합성 데이터(직선 이동 + 가우시안 노이즈)로 동작 검증
  3. 추정값 vs 실제값 vs 관측값 그래프 생성 (논문 Figure 초안)
  4. 표 2 포맷(16컬럼) CSV 생성

완료 기준: KF 추정값이 노이즈 관측값보다 실제 위치에 더 가까운 그래프 생성

Author: Yubin
Date: 2026-03-30
Scenario: E0 (Python Simulation)
"""

import numpy as np
import matplotlib.pyplot as plt
import csv

# ============================================================
# 1. PARAMETERS
# ============================================================
dt = 0.005              # 5ms sampling period (200Hz, MCU 제어 루프 주기)
N = 2000                # timesteps → 10초 (2000 × 5ms)
v_true = 200.0          # 로봇 이동 속도 (mm/s), 실제 스펙에 맞춰 튜닝 필요

# KF model parameters
A = 1.0                 # 상태 전이: x(k) = 1·x(k-1) + B·u  (위치 유지)
B = dt                  # 입력 게인: 위치 += 속도 × dt
H = 1.0                 # 관측 모델: 센서가 위치를 직접 측정 (변환 없음)

# KF noise parameters
Q = 1.0                 # 프로세스 노이즈 분산 (엔코더 불확실성, 튜닝 필요)
R = 400.0               # 측정 노이즈 분산 (VL53L0X σ≈20mm → 20²=400)

# 합성 데이터 생성용 노이즈
sigma_process = 0.5     # 매 스텝 위치 흔들림 (mm)
sigma_measurement = 20.0  # VL53L0X 측정 노이즈 std (mm)

# 잔차 윈도우 크기
W = 20                  # 20스텝 × 5ms = 100ms (시뮬레이션 검증 결과 채택)

np.random.seed(42)      # 재현성 보장

# ============================================================
# 2. SYNTHETIC DATA GENERATION (합성 데이터)
# ============================================================
# Ground truth: 100mm에서 출발, 200mm/s로 직선 이동
x_true = np.zeros(N)
x_true[0] = 100.0

# 엔코더 속도 (true + noise)
u_encoder = np.zeros(N)

# VL53L0X 센서 측정값
z_tof = np.zeros(N)
z_tof[0] = x_true[0] + np.random.randn() * sigma_measurement

for k in range(1, N):
    # 실제 위치: 이전 + 속도×시간 + 프로세스 노이즈
    x_true[k] = x_true[k-1] + v_true * dt + np.random.randn() * sigma_process

    # 엔코더가 읽는 속도 (약간의 노이즈 포함)
    u_encoder[k] = v_true + np.random.randn() * (sigma_process / dt)

    # VL53L0X 측정: 실제 위치 + 센서 노이즈
    z_tof[k] = x_true[k] + np.random.randn() * sigma_measurement

# 엔코더 누적 거리
encoder_distance = np.zeros(N)
encoder_distance[0] = x_true[0]
for k in range(1, N):
    encoder_distance[k] = encoder_distance[k-1] + u_encoder[k] * dt

# ============================================================
# 3. KALMAN FILTER (Predict → Update 루프)
# ============================================================
x_est = np.zeros(N)       # 후험 추정값 (최종 출력)
x_pred_arr = np.zeros(N)  # 사전 추정값 (예측)
P_est = np.zeros(N)       # 후험 오차 공분산
P_pred_arr = np.zeros(N)  # 사전 오차 공분산
K_gain = np.zeros(N)      # 칼만 게인
residual = np.zeros(N)    # 잔차 (z - x_pred)
innov_cov = np.zeros(N)   # innovation covariance S = P_pred + R (NIS 계산용)

# 초기 조건
x_est[0] = z_tof[0]       # 첫 번째 측정값으로 초기화
P_est[0] = R              # 초기 불확실성 = 측정 불확실성

for k in range(1, N):
    # ---- PREDICT ----
    x_pred = A * x_est[k-1] + B * u_encoder[k]
    P_pred = A * P_est[k-1] * A + Q

    x_pred_arr[k] = x_pred
    P_pred_arr[k] = P_pred

    # ---- UPDATE ----
    innov_cov[k] = P_pred + R                   # innovation covariance S
    K_gain[k] = P_pred / (P_pred + R)           # 칼만 게인
    residual[k] = z_tof[k] - x_pred             # 잔차
    x_est[k] = x_pred + K_gain[k] * residual[k] # 후험 추정
    P_est[k] = (1 - K_gain[k]) * P_pred         # 후험 공분산

# ============================================================
# 4. DERIVED FIELDS (표 2 계산 필드)
# ============================================================
# tof_residual_var / tof_residual_mean: 슬라이딩 윈도우 W
tof_residual_var = np.full(N, np.nan)
tof_residual_mean = np.full(N, np.nan)
for k in range(W, N):
    window = residual[k - W + 1 : k + 1]
    tof_residual_var[k] = np.var(window)
    tof_residual_mean[k] = np.mean(window)

# tof_meas_rate: z(k) - z(k-1)
tof_meas_rate = np.full(N, np.nan)
for k in range(1, N):
    tof_meas_rate[k] = z_tof[k] - z_tof[k - 1]

# R_label: Covariance Matching pseudo label (잔차 분산 기반, 실데이터로 교체 예정)
R_label = np.full(N, np.nan)
for k in range(W, N):
    window = residual[k - W + 1 : k + 1]
    R_label[k] = np.var(window)

# ============================================================
# 5. PERFORMANCE METRICS (성능 지표)
# ============================================================
rmse_sensor = np.sqrt(np.mean((z_tof - x_true) ** 2))
rmse_kf = np.sqrt(np.mean((x_est - x_true) ** 2))
mae_sensor = np.mean(np.abs(z_tof - x_true))
mae_kf = np.mean(np.abs(x_est - x_true))

print("=" * 55)
print("  1D Kalman Filter Simulation Results (E0)")
print("=" * 55)
print(f"  Total time:     {N * dt:.1f}s ({N} steps @ {1/dt:.0f}Hz)")
print(f"  True velocity:  {v_true} mm/s")
print(f"  Parameters:     Q={Q}, R={R}, W={W}")
print("-" * 55)
print(f"  Sensor RMSE:    {rmse_sensor:.2f} mm")
print(f"  KF RMSE:        {rmse_kf:.2f} mm")
print(f"  RMSE improvement: {(1 - rmse_kf / rmse_sensor) * 100:.1f}%")
print("-" * 55)
print(f"  Sensor MAE:     {mae_sensor:.2f} mm")
print(f"  KF MAE:         {mae_kf:.2f} mm")
print(f"  MAE improvement:  {(1 - mae_kf / mae_sensor) * 100:.1f}%")
print("-" * 55)
print(f"  Steady-state K: {K_gain[-1]:.4f}")
print(f"  Steady-state P: {P_est[-1]:.4f}")
print("=" * 55)

# ============================================================
# 6. VISUALIZATION (논문 Figure 초안)
# ============================================================
time = np.arange(N) * dt

fig, axes = plt.subplots(3, 1, figsize=(12, 10),
                         gridspec_kw={'height_ratios': [3, 1, 1]})
fig.suptitle('1D Kalman Filter: Encoder + VL53L0X Sensor Fusion (E0)',
             fontsize=14, fontweight='bold')

# --- Plot 1: 추정값 vs 실제값 vs 관측값 ---
ax1 = axes[0]
ax1.plot(time, x_true, 'k-', linewidth=2, label='Ground truth', zorder=3)
ax1.scatter(time[::20], z_tof[::20], c='red', s=8, alpha=0.4,
            label='VL53L0X measurement', zorder=1)
ax1.plot(time, x_est, 'b-', linewidth=1.5, label='KF estimate', zorder=2)

# 신뢰 구간 (±2σ)
upper = x_est + 2 * np.sqrt(P_est)
lower = x_est - 2 * np.sqrt(P_est)
ax1.fill_between(time, lower, upper, color='blue', alpha=0.1, label='KF ±2σ band')

ax1.set_ylabel('Position (mm)')
ax1.set_title(
    f'RMSE: Sensor={rmse_sensor:.1f}mm → KF={rmse_kf:.1f}mm '
    f'(↓{(1 - rmse_kf / rmse_sensor) * 100:.0f}%)')
ax1.legend(loc='upper left', fontsize=9)
ax1.grid(True, alpha=0.3)

# --- Plot 2: 칼만 게인 수렴 ---
ax2 = axes[1]
ax2.plot(time, K_gain, 'g-', linewidth=1)
ax2.set_ylabel('Kalman Gain K')
ax2.set_ylim(0, 1)
ax2.axhline(y=K_gain[-1], color='g', linestyle='--', alpha=0.5,
            label=f'Steady-state K={K_gain[-1]:.3f}')
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

# --- Plot 3: 잔차 ---
ax3 = axes[2]
ax3.plot(time, residual, 'orange', linewidth=0.8, alpha=0.7)
ax3.axhline(y=0, color='k', linewidth=0.5)
ax3.axhline(y=2 * np.sqrt(R), color='r', linestyle='--', alpha=0.4,
            label=f'±2√R = ±{2 * np.sqrt(R):.0f}mm')
ax3.axhline(y=-2 * np.sqrt(R), color='r', linestyle='--', alpha=0.4)
ax3.set_ylabel('Residual (mm)')
ax3.set_xlabel('Time (s)')
ax3.legend(fontsize=9)
ax3.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('kf_simulation_result.png', dpi=150, bbox_inches='tight')
plt.savefig('kf_simulation_result.pdf', bbox_inches='tight')
print("\n[Saved] kf_simulation_result.png / .pdf")

# ============================================================
# 7. CSV EXPORT (표 2 포맷, 16컬럼)
# ============================================================
csv_filename = 'kf_simulation_data.csv'
with open(csv_filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        'timestamp_ms',        # 1
        'tof_distance_mm',     # 2
        'tof_signal_rate',     # 3  (빈 값 — 하드웨어 전용)
        'tof_range_status',    # 4  (빈 값 — 하드웨어 전용)
        'us_distance_mm',      # 5  (빈 값 — HC-SR04 없음)
        'encoder_distance_mm', # 6
        'encoder_speed_mms',   # 7
        'kf_estimate_mm',      # 8
        'tof_residual',        # 9
        'tof_residual_var',    # 10
        'tof_residual_mean',   # 11
        'sensor_disagree',     # 12 (빈 값 — us_distance 없음)
        'tof_meas_rate',       # 13
        'gt_distance_mm',      # 14
        'R_label',             # 15
        'kalman_gain',         # 16 (추가: KF update에서 계산되는 K)
        'innovation_cov',      # 17 (추가: S = P_pred + R, NIS 계산용)
        'scenario_id',         # 18
    ])
    for k in range(N):
        writer.writerow([
            k * 5,                                                                      # 1
            round(z_tof[k], 2),                                                         # 2
            '',                                                                         # 3
            '',                                                                         # 4
            '',                                                                         # 5
            round(encoder_distance[k], 4),                                              # 6
            round(u_encoder[k], 4),                                                     # 7
            round(x_est[k], 4),                                                         # 8
            round(residual[k], 4),                                                      # 9
            round(tof_residual_var[k], 4) if not np.isnan(tof_residual_var[k]) else '',  # 10
            round(tof_residual_mean[k], 4) if not np.isnan(tof_residual_mean[k]) else '',# 11
            '',                                                                         # 12
            round(tof_meas_rate[k], 4) if not np.isnan(tof_meas_rate[k]) else '',       # 13
            round(x_true[k], 4),                                                        # 14
            round(R_label[k], 4) if not np.isnan(R_label[k]) else '',                   # 15
            round(K_gain[k], 6),                                                        # 16
            round(innov_cov[k], 4),                                                     # 17
            'E0',                                                                       # 18
        ])

print(f"[Saved] {csv_filename} (18 columns, {N} rows, scenario=E0)")
print(f"\nAll outputs: kf_simulation_result.png, .pdf, {csv_filename}")
