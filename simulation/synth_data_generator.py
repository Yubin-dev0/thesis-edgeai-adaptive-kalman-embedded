"""
Synthetic Data Generator + Filter Comparison
==============================================
역할:
  1. E1~E5 시나리오별 합성 센서 데이터 생성
  2. 각 시나리오에서 Fixed KF vs CM-AKF 자동 실행
  3. 비교 그래프 + 요약 테이블 생성
  4. 시나리오별 CSV 출력 (18컬럼 포맷)

시나리오:
  E1 Normal       — 일정 가우시안 노이즈 (baseline)
  E2 Noise Spike  — 특정 구간 노이즈 급증 (σ 20→100→20)
  E3 Sensor Fault — VL53L0X 3초 차단 (측정값 고정)
  E4 Long-term    — 60초, σ 서서히 증가 (20→40mm drift)
  E5 Surface Change — 표면 전환으로 σ 급변 (20→60mm)

완료 기준: E2(스파이크)에서 CM-AKF가 Fixed KF보다
          더 빠르게 복구하는 그래프 생성

Author: Yubin
Date: 2026-03-31
"""

import numpy as np
import matplotlib.pyplot as plt
import csv
import os

# ============================================================
# 1. COMMON PARAMETERS
# ============================================================
dt = 0.005              # 5ms (200Hz)
v_true = 200.0          # mm/s

# KF model
A = 1.0
B = dt
H = 1.0
Q = 1.0
R_initial = 400.0       # σ=20mm → 20²

# CM-AKF
R_MIN = 1.0
R_MAX = 10000.0
W = 20

# 합성 데이터 기본 노이즈
sigma_process = 0.5     # 프로세스 노이즈

# ============================================================
# 2. SCENARIO DEFINITIONS
# ============================================================
def make_scenario(scenario_id, seed=None):
    """
    시나리오별 합성 데이터를 생성한다.
    
    Returns:
        dict with keys: N, x_true, z_tof, u_encoder, encoder_distance,
                        sigma_meas_profile, scenario_id, description
        sigma_meas_profile: 매 스텝의 실제 측정 노이즈 σ (시각화/분석용)
    """
    if seed is not None:
        np.random.seed(seed)
    
    if scenario_id == 'E1':
        # ---- E1: Normal (baseline) ----
        N = 2000  # 10초
        sigma_meas = np.full(N, 20.0)  # 일정 σ=20mm
        desc = 'E1 Normal: constant σ=20mm'
        
    elif scenario_id == 'E2':
        # ---- E2: Noise Spike ----
        # 3~5초 구간에서 σ=20→100mm 급증 후 복귀
        N = 2000  # 10초
        sigma_meas = np.full(N, 20.0)
        spike_start = int(3.0 / dt)   # k=600
        spike_end = int(5.0 / dt)     # k=1000
        sigma_meas[spike_start:spike_end] = 100.0
        desc = 'E2 Spike: σ=100mm @ 3-5s'
        
    elif scenario_id == 'E3':
        # ---- E3: Sensor Fault ----
        # 4~7초 구간에서 VL53L0X 차단 → 마지막 정상값 고정 (stuck sensor)
        N = 2000  # 10초
        sigma_meas = np.full(N, 20.0)
        fault_start = int(4.0 / dt)   # k=800
        fault_end = int(7.0 / dt)     # k=1400
        # sigma_meas는 정상으로 두되, z_tof 생성 시 별도 처리
        desc = 'E3 Fault: sensor stuck @ 4-7s'
        
    elif scenario_id == 'E4':
        # ---- E4: Long-term Drift ----
        # 60초, σ가 20→40mm로 선형 증가
        N = 12000  # 60초
        sigma_meas = np.linspace(20.0, 40.0, N)
        desc = 'E4 Drift: σ 20→40mm over 60s'
        
    elif scenario_id == 'E5':
        # ---- E5: Surface Change ----
        # 5초 기점으로 σ=20→60mm 급변 (표면 전환)
        N = 2000  # 10초
        sigma_meas = np.full(N, 20.0)
        change_point = int(5.0 / dt)  # k=1000
        sigma_meas[change_point:] = 60.0
        desc = 'E5 Surface: σ 20→60mm @ 5s'
        
    else:
        raise ValueError(f"Unknown scenario: {scenario_id}")
    
    # ---- Ground truth 생성 ----
    x_true = np.zeros(N)
    x_true[0] = 100.0
    for k in range(1, N):
        x_true[k] = x_true[k-1] + v_true * dt + np.random.randn() * sigma_process
    
    # ---- 엔코더 속도 ----
    u_encoder = np.zeros(N)
    for k in range(1, N):
        u_encoder[k] = v_true + np.random.randn() * (sigma_process / dt)
    
    # ---- 엔코더 누적 거리 ----
    encoder_distance = np.zeros(N)
    encoder_distance[0] = x_true[0]
    for k in range(1, N):
        encoder_distance[k] = encoder_distance[k-1] + u_encoder[k] * dt
    
    # ---- VL53L0X 측정값 생성 ----
    z_tof = np.zeros(N)
    z_tof[0] = x_true[0] + np.random.randn() * sigma_meas[0]
    
    if scenario_id == 'E3':
        # E3: 정상 구간은 일반 노이즈, 차단 구간은 마지막 정상값 고정
        for k in range(1, N):
            if k < fault_start or k >= fault_end:
                z_tof[k] = x_true[k] + np.random.randn() * sigma_meas[k]
            elif k == fault_start:
                # 차단 시작: 직전 값으로 고정
                z_tof[k] = z_tof[k-1]
            else:
                # 차단 중: 계속 고정 (stuck sensor)
                z_tof[k] = z_tof[fault_start - 1]
    else:
        for k in range(1, N):
            z_tof[k] = x_true[k] + np.random.randn() * sigma_meas[k]
    
    return {
        'N': N,
        'x_true': x_true,
        'z_tof': z_tof,
        'u_encoder': u_encoder,
        'encoder_distance': encoder_distance,
        'sigma_meas_profile': sigma_meas,
        'scenario_id': scenario_id,
        'description': desc,
    }

# ============================================================
# 3. FILTER FUNCTIONS
# ============================================================
def run_fixed_kf(data):
    """Fixed KF (R=400 고정)"""
    N = data['N']
    z_tof = data['z_tof']
    u_encoder = data['u_encoder']
    
    x_est = np.zeros(N)
    P_est = np.zeros(N)
    K_gain = np.zeros(N)
    residual = np.zeros(N)
    
    x_est[0] = z_tof[0]
    P_est[0] = R_initial
    R = R_initial
    
    for k in range(1, N):
        x_pred = A * x_est[k-1] + B * u_encoder[k]
        P_pred = A * P_est[k-1] * A + Q
        
        K_gain[k] = P_pred / (P_pred + R)
        residual[k] = z_tof[k] - x_pred
        x_est[k] = x_pred + K_gain[k] * residual[k]
        P_est[k] = (1 - K_gain[k]) * P_pred
    
    return {
        'x_est': x_est, 'P_est': P_est,
        'K_gain': K_gain, 'residual': residual,
        'R_values': np.full(N, R_initial),
        'label': 'Fixed KF',
    }

def run_cm_akf(data):
    """Covariance Matching AKF [Mehra, 1970]"""
    N = data['N']
    z_tof = data['z_tof']
    u_encoder = data['u_encoder']
    
    x_est = np.zeros(N)
    P_est = np.zeros(N)
    K_gain = np.zeros(N)
    residual = np.zeros(N)
    R_adaptive = np.zeros(N)
    
    x_est[0] = z_tof[0]
    P_est[0] = R_initial
    R_adaptive[0] = R_initial
    R_current = R_initial
    
    # 링버퍼
    res_buf = np.zeros(W)
    buf_idx = 0
    buf_count = 0
    
    for k in range(1, N):
        # PREDICT
        x_pred = A * x_est[k-1] + B * u_encoder[k]
        P_pred = A * P_est[k-1] * A + Q
        
        # RESIDUAL
        residual[k] = z_tof[k] - x_pred
        
        # 링버퍼 업데이트
        res_buf[buf_idx] = residual[k]
        buf_idx = (buf_idx + 1) % W
        buf_count = min(buf_count + 1, W)
        
        # CM: R = E[r²] − P_pred
        if buf_count >= W:
            res_sq_mean = np.mean(res_buf ** 2)
            R_current = np.clip(res_sq_mean - P_pred, R_MIN, R_MAX)
        
        R_adaptive[k] = R_current
        
        # UPDATE
        K_gain[k] = P_pred / (P_pred + R_current)
        x_est[k] = x_pred + K_gain[k] * residual[k]
        P_est[k] = (1 - K_gain[k]) * P_pred
    
    return {
        'x_est': x_est, 'P_est': P_est,
        'K_gain': K_gain, 'residual': residual,
        'R_values': R_adaptive,
        'label': 'CM-AKF',
    }

# ============================================================
# 4. METRICS
# ============================================================
def compute_metrics(data, result):
    """RMSE, MAE, Max Error 계산"""
    x_true = data['x_true']
    x_est = result['x_est']
    err = x_est - x_true
    
    rmse = np.sqrt(np.mean(err ** 2))
    mae = np.mean(np.abs(err))
    max_err = np.max(np.abs(err))
    return {'rmse': rmse, 'mae': mae, 'max_err': max_err}

def compute_recovery_time(data, result, event_end_k, threshold_mm=10.0, window=20):
    """
    이벤트 종료 후 오차가 threshold 이하로 안정되는 데 걸리는 스텝 수.
    E2, E3, E5에서 복구 속도 비교용.
    """
    x_true = data['x_true']
    x_est = result['x_est']
    N = data['N']
    
    for k in range(event_end_k, N - window):
        errors = np.abs(x_est[k:k+window] - x_true[k:k+window])
        if np.all(errors < threshold_mm):
            return (k - event_end_k) * dt  # 초 단위
    return np.nan

# ============================================================
# 5. VISUALIZATION
# ============================================================
def plot_comparison(data, fixed_result, akf_result, metrics_fixed, metrics_akf,
                    save_prefix):
    """4패널 비교 그래프"""
    N = data['N']
    time = np.arange(N) * dt
    x_true = data['x_true']
    z_tof = data['z_tof']
    sigma_prof = data['sigma_meas_profile']
    
    fig, axes = plt.subplots(4, 1, figsize=(14, 14),
                             gridspec_kw={'height_ratios': [3, 1.2, 1.2, 1.2]})
    fig.suptitle(f'{data["description"]}: Fixed KF vs CM-AKF',
                 fontsize=14, fontweight='bold')
    
    # Plot 1: 위치 추정
    ax1 = axes[0]
    ax1.plot(time, x_true, 'k-', lw=2, label='Ground truth', zorder=3)
    step = max(1, N // 100)
    ax1.scatter(time[::step], z_tof[::step], c='red', s=8, alpha=0.3,
                label='VL53L0X', zorder=1)
    ax1.plot(time, fixed_result['x_est'], 'b-', lw=1.2, alpha=0.7,
             label=f'Fixed KF (RMSE={metrics_fixed["rmse"]:.2f}mm)')
    ax1.plot(time, akf_result['x_est'], 'g-', lw=1.2, alpha=0.8,
             label=f'CM-AKF (RMSE={metrics_akf["rmse"]:.2f}mm)')
    
    # 이벤트 구간 표시
    _shade_events(ax1, data)
    
    ax1.set_ylabel('Position (mm)')
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: 칼만 게인
    ax2 = axes[1]
    ax2.plot(time, fixed_result['K_gain'], 'b-', lw=0.8, alpha=0.7, label='Fixed KF K')
    ax2.plot(time, akf_result['K_gain'], 'g-', lw=0.8, alpha=0.8, label='CM-AKF K')
    _shade_events(ax2, data)
    ax2.set_ylabel('Kalman Gain K')
    ax2.set_ylim(0, max(0.6, akf_result['K_gain'].max() * 1.2))
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: R 변화 + 실제 σ² 프로파일
    ax3 = axes[2]
    ax3.axhline(y=R_initial, color='b', ls='--', alpha=0.5, label='Fixed R=400')
    ax3.plot(time, akf_result['R_values'], 'g-', lw=1, label='CM-AKF R(k)')
    ax3.plot(time, sigma_prof**2, 'r--', lw=1, alpha=0.6, label='True σ²(k)')
    _shade_events(ax3, data)
    ax3.set_ylabel('R value')
    ax3.set_yscale('log')
    ax3.set_ylim(R_MIN * 0.5, R_MAX * 2)
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: 추정 오차 (|x_est - x_true|)
    ax4 = axes[3]
    err_fixed = np.abs(fixed_result['x_est'] - x_true)
    err_akf = np.abs(akf_result['x_est'] - x_true)
    ax4.plot(time, err_fixed, 'b-', lw=0.8, alpha=0.5, label='Fixed KF |error|')
    ax4.plot(time, err_akf, 'g-', lw=0.8, alpha=0.6, label='CM-AKF |error|')
    _shade_events(ax4, data)
    ax4.set_ylabel('|Error| (mm)')
    ax4.set_xlabel('Time (s)')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{save_prefix}.png', dpi=150, bbox_inches='tight')
    plt.close()

def _shade_events(ax, data):
    """시나리오별 이벤트 구간을 회색 음영으로 표시"""
    sid = data['scenario_id']
    if sid == 'E2':
        ax.axvspan(3.0, 5.0, alpha=0.15, color='orange', label='Spike zone')
    elif sid == 'E3':
        ax.axvspan(4.0, 7.0, alpha=0.15, color='red', label='Fault zone')
    elif sid == 'E5':
        ax.axvspan(5.0, data['N'] * dt, alpha=0.15, color='purple',
                   label='Surface change')

# ============================================================
# 6. CSV EXPORT
# ============================================================
def export_csv(data, akf_result, filename):
    """18컬럼 CSV 출력 (CM-AKF 결과 기준)"""
    N = data['N']
    z_tof = data['z_tof']
    u_encoder = data['u_encoder']
    encoder_distance = data['encoder_distance']
    x_true = data['x_true']
    x_est = akf_result['x_est']
    residual = akf_result['residual']
    K_gain = akf_result['K_gain']
    P_est = akf_result['P_est']
    R_values = akf_result['R_values']
    
    # 윈도우 통계
    tof_residual_var = np.full(N, np.nan)
    tof_residual_mean = np.full(N, np.nan)
    R_label = np.full(N, np.nan)
    for k in range(W, N):
        win = residual[k - W + 1 : k + 1]
        tof_residual_var[k] = np.var(win)
        tof_residual_mean[k] = np.mean(win)
        R_label[k] = np.var(win)
    
    tof_meas_rate = np.full(N, np.nan)
    for k in range(1, N):
        tof_meas_rate[k] = z_tof[k] - z_tof[k - 1]
    
    # innovation covariance
    innov_cov = np.zeros(N)
    for k in range(1, N):
        P_pred_k = A * (P_est[k-1] if k > 0 else R_initial) * A + Q
        innov_cov[k] = P_pred_k + R_values[k]
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'timestamp_ms', 'tof_distance_mm', 'tof_signal_rate',
            'tof_range_status', 'us_distance_mm', 'encoder_distance_mm',
            'encoder_speed_mms', 'kf_estimate_mm', 'tof_residual',
            'tof_residual_var', 'tof_residual_mean', 'sensor_disagree',
            'tof_meas_rate', 'gt_distance_mm', 'R_label',
            'kalman_gain', 'innovation_cov', 'scenario_id',
        ])
        for k in range(N):
            writer.writerow([
                k * 5,
                round(z_tof[k], 2),
                '', '', '',
                round(encoder_distance[k], 4),
                round(u_encoder[k], 4),
                round(x_est[k], 4),
                round(residual[k], 4),
                round(tof_residual_var[k], 4) if not np.isnan(tof_residual_var[k]) else '',
                round(tof_residual_mean[k], 4) if not np.isnan(tof_residual_mean[k]) else '',
                '',
                round(tof_meas_rate[k], 4) if not np.isnan(tof_meas_rate[k]) else '',
                round(x_true[k], 4),
                round(R_label[k], 4) if not np.isnan(R_label[k]) else '',
                round(K_gain[k], 6),
                round(innov_cov[k], 4),
                data['scenario_id'],
            ])

# ============================================================
# 7. MAIN: 전체 시나리오 실행
# ============================================================
if __name__ == '__main__':
    scenarios = ['E1', 'E2', 'E3', 'E4', 'E5']
    seeds = {'E1': 100, 'E2': 200, 'E3': 300, 'E4': 400, 'E5': 500}
    
    # 이벤트 종료 시점 (복구 시간 계산용)
    event_end_k = {
        'E1': None,
        'E2': int(5.0 / dt),    # k=1000
        'E3': int(7.0 / dt),    # k=1400
        'E4': None,             # drift는 복구 개념 없음
        'E5': int(5.0 / dt),    # k=1000 (전환 시점)
    }
    
    # 결과 수집
    summary = []
    
    print("=" * 75)
    print("  Synthetic Data Generator + Filter Comparison")
    print("  Scenarios: E1(Normal), E2(Spike), E3(Fault), E4(Drift), E5(Surface)")
    print("=" * 75)
    
    for sid in scenarios:
        print(f"\n--- {sid} ---")
        
        # 데이터 생성
        data = make_scenario(sid, seed=seeds[sid])
        print(f"  {data['description']}")
        print(f"  N={data['N']} steps ({data['N']*dt:.0f}s)")
        
        # 필터 실행
        fixed = run_fixed_kf(data)
        akf = run_cm_akf(data)
        
        # 메트릭
        m_fixed = compute_metrics(data, fixed)
        m_akf = compute_metrics(data, akf)
        
        # 복구 시간
        rec_fixed = np.nan
        rec_akf = np.nan
        if event_end_k[sid] is not None:
            rec_fixed = compute_recovery_time(data, fixed, event_end_k[sid])
            rec_akf = compute_recovery_time(data, akf, event_end_k[sid])
        
        # 출력
        print(f"  Fixed KF:  RMSE={m_fixed['rmse']:.2f}mm, "
              f"MAE={m_fixed['mae']:.2f}mm, MaxErr={m_fixed['max_err']:.1f}mm")
        print(f"  CM-AKF:    RMSE={m_akf['rmse']:.2f}mm, "
              f"MAE={m_akf['mae']:.2f}mm, MaxErr={m_akf['max_err']:.1f}mm")
        print(f"  RMSE change: {(1 - m_akf['rmse'] / m_fixed['rmse']) * 100:+.1f}%")
        if not np.isnan(rec_fixed):
            print(f"  Recovery:  Fixed={rec_fixed:.3f}s, CM-AKF={rec_akf:.3f}s")
        
        # 그래프
        plot_comparison(data, fixed, akf, m_fixed, m_akf, f'synth_{sid}')
        print(f"  [Saved] synth_{sid}.png")
        
        # CSV
        csv_name = f'synth_{sid}_data.csv'
        export_csv(data, akf, csv_name)
        print(f"  [Saved] {csv_name}")
        
        summary.append({
            'scenario': sid,
            'desc': data['description'],
            'rmse_fixed': m_fixed['rmse'],
            'rmse_akf': m_akf['rmse'],
            'mae_fixed': m_fixed['mae'],
            'mae_akf': m_akf['mae'],
            'max_err_fixed': m_fixed['max_err'],
            'max_err_akf': m_akf['max_err'],
            'recovery_fixed': rec_fixed,
            'recovery_akf': rec_akf,
        })
    
    # ============================================================
    # 8. SUMMARY TABLE
    # ============================================================
    print("\n" + "=" * 95)
    print("  SUMMARY: Fixed KF vs CM-AKF across all scenarios")
    print("=" * 95)
    print(f"  {'Scenario':<12s} {'RMSE Fixed':>11s} {'RMSE AKF':>10s} "
          f"{'Δ RMSE':>8s} {'MaxErr F':>9s} {'MaxErr A':>9s} "
          f"{'Rec F':>8s} {'Rec A':>8s}")
    print("-" * 95)
    for s in summary:
        rec_f = f"{s['recovery_fixed']:.3f}" if not np.isnan(s['recovery_fixed']) else "N/A"
        rec_a = f"{s['recovery_akf']:.3f}" if not np.isnan(s['recovery_akf']) else "N/A"
        delta = (1 - s['rmse_akf'] / s['rmse_fixed']) * 100
        print(f"  {s['scenario']:<12s} {s['rmse_fixed']:>10.2f}mm {s['rmse_akf']:>9.2f}mm "
              f"{delta:>+7.1f}% {s['max_err_fixed']:>8.1f}mm {s['max_err_akf']:>8.1f}mm "
              f"{rec_f:>8s} {rec_a:>8s}")
    print("=" * 95)
    print("\nAll outputs saved. Done.")
