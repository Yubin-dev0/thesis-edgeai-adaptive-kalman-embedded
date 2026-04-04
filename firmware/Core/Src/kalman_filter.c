/**
 * @file    kalman_filter.c
 * @brief   1D Kalman Filter + Covariance Matching AKF implementation
 *
 * This file implements the core KF predict/update cycle and the
 * CM-AKF adaptive R estimation.  All logic is a direct C translation
 * of the verified Python code (kf_simulation_1D.py, rule_akf_1D.py).
 *
 * Python-to-C correspondence:
 *   Python                          C
 *   ──────────────────────────────  ──────────────────────────────
 *   x_pred = A*x + B*u             kf->x_pred = KF_A*kf->x + KF_B*u
 *   P_pred = A*P*A + Q             kf->P_pred = KF_A*kf->P*KF_A + KF_Q
 *   K = P_pred / (P_pred + R)      kf->K = P_pred / fmaxf(denom, GUARD)
 *   np.clip(R, R_MIN, R_MAX)       fmaxf(fminf(R_new, R_MAX), R_MIN)
 *   residual_buffer[buf_idx]        rb->buf[rb->idx]
 *   np.mean(buf**2)                 rb->sq_sum / W  (O(1) running sum)
 *
 * @author  Yubin
 * @date    2026-04-02
 */

#include "kalman_filter.h"
#include <math.h>
#include <string.h>

/* ================================================================
 * Internal helpers
 * ================================================================ */

/**
 * @brief  Push a new residual into the circular buffer.
 *
 * If the buffer is already full, the oldest entry is subtracted
 * from the running sums before the new entry is added.  This keeps
 * mean / squared-mean computation at O(1) per step.
 *
 * Corresponds to Python:
 *   residual_buffer[buf_idx] = residual
 *   buf_idx = (buf_idx + 1) % W
 *   buf_count = min(buf_count + 1, W)
 */
static void resbuf_push(ResidualBuffer *rb, float r)
{
    if (rb->count >= KF_WINDOW_SIZE) {
        /* Buffer full — subtract the oldest entry being overwritten */
        float old = rb->buf[rb->idx];
        rb->sum    -= old;
        rb->sq_sum -= old * old;
    }

    /* Write new entry */
    rb->buf[rb->idx] = r;
    rb->sum    += r;
    rb->sq_sum += r * r;

    /* Advance index (wraps around) */
    rb->idx = (rb->idx + 1) % KF_WINDOW_SIZE;

    if (rb->count < KF_WINDOW_SIZE) {
        rb->count++;
    }
}

/**
 * @brief  Compute mean of squared residuals: (1/W) * sum(r_i^2)
 *
 * Corresponds to Python:
 *   res_sq_mean = np.mean(residual_buffer ** 2)
 */
static float resbuf_sq_mean(const ResidualBuffer *rb)
{
    if (rb->count == 0) {
        return 0.0f;
    }
    return rb->sq_sum / (float)rb->count;
}

/* ================================================================
 * Public API implementation
 * ================================================================ */

/* ----------------------------------------------------------------
 * kf_init
 * ----------------------------------------------------------------
 * Corresponds to Python:
 *   x_est[0] = z_tof[0]
 *   P_est[0] = R_initial
 *   R_current = R_initial
 *   residual_buffer = zeros(W);  buf_idx=0; buf_count=0
 * ---------------------------------------------------------------- */
void kf_init(KalmanFilter *kf, float x0, float P0, float R0)
{
    /* Zero-fill everything including the circular buffer */
    memset(kf, 0, sizeof(KalmanFilter));

    kf->x = x0;
    kf->P = P0;
    kf->R = R0;

    /* x_pred / P_pred are undefined until the first predict call */
    kf->x_pred = x0;
    kf->P_pred = P0;
}

/* ----------------------------------------------------------------
 * kf_predict
 * ----------------------------------------------------------------
 * Corresponds to Python (both fixed KF and AKF share this):
 *   x_pred = A * x_est[k-1] + B * u_encoder[k]
 *   P_pred = A * P_est[k-1] * A + Q
 *
 * P floor guard: prevents P from collapsing to zero, which would
 * freeze the Kalman gain at 0 and make the filter ignore all
 * future measurements.
 * ---------------------------------------------------------------- */
void kf_predict(KalmanFilter *kf, float u)
{
    kf->x_pred = KF_A * kf->x + KF_B * u;
    kf->P_pred = KF_A * kf->P * KF_A + KF_Q;

    /* P floor guard */
    if (kf->P_pred < KF_P_FLOOR) {
        kf->P_pred = KF_P_FLOOR;
    }
}

/* ----------------------------------------------------------------
 * kf_update
 * ----------------------------------------------------------------
 * Corresponds to Python:
 *
 *   # --- Residual ---
 *   residual[k] = z_tof[k] - x_pred
 *
 *   # --- Buffer push (rule_akf_1D.py) ---
 *   residual_buffer[buf_idx] = residual[k]
 *   buf_idx = (buf_idx + 1) % W
 *   buf_count = min(buf_count + 1, W)
 *
 *   # --- CM adaptive R (rule_akf_1D.py, only when buf full) ---
 *   if buf_count >= W:
 *       res_sq_mean = np.mean(residual_buffer ** 2)
 *       R_current = np.clip(res_sq_mean - P_pred, R_MIN, R_MAX)
 *
 *   # --- Standard KF update ---
 *   innov_cov[k] = P_pred + R
 *   K_gain[k]    = P_pred / (P_pred + R)
 *   x_est[k]     = x_pred + K * residual[k]
 *   P_est[k]     = (1 - K) * P_pred
 *
 * Note: R is updated BEFORE the KF update equations, matching
 * the Python implementation order.  Updating after would introduce
 * a one-step delay in the adaptive response.
 * ---------------------------------------------------------------- */
void kf_update(KalmanFilter *kf, float z, bool use_akf)
{
    float P_pred = kf->P_pred;
    float denom;

    /* ---- Residual ---- */
    kf->residual = z - kf->x_pred;

    /* ---- Push residual into circular buffer ---- */
    resbuf_push(&kf->res_buf, kf->residual);

    /* ---- CM Adaptive R [Mehra, 1970] ---- */
    if (use_akf && kf->res_buf.count >= KF_WINDOW_SIZE) {
        /*
         * R(k) = (1/W) * sum(r_i^2)  -  P_pred(k)
         *
         * Derivation:  S = H*P_pred*H' + R  =>  R = E[r^2] - P_pred
         * (H = 1 in our 1D case)
         */
        float res_sq_mean = resbuf_sq_mean(&kf->res_buf);
        float R_new = res_sq_mean - P_pred;

        /* Clamp: np.clip(R_new, R_MIN, R_MAX) */
        if (R_new < KF_R_MIN)  R_new = KF_R_MIN;
        if (R_new > KF_R_MAX)  R_new = KF_R_MAX;

        kf->R = R_new;
    }
    /* else: keep kf->R unchanged (initial fixed value) */

    /* ---- Standard KF Update ---- */
    denom = P_pred + kf->R;
    if (denom < KF_DENOM_GUARD) {
        denom = KF_DENOM_GUARD;
    }

    kf->S = denom;                                  /* innovation covariance */
    kf->K = P_pred / denom;                         /* Kalman gain          */
    kf->x = kf->x_pred + kf->K * kf->residual;     /* posterior estimate   */
    kf->P = (1.0f - kf->K) * P_pred;                /* posterior covariance */

    /* P floor guard (same reason as in predict) */
    if (kf->P < KF_P_FLOOR) {
        kf->P = KF_P_FLOOR;
    }
}

/* ----------------------------------------------------------------
 * kf_get_residual_stats
 * ----------------------------------------------------------------
 * Utility for CSV logging fields:
 *   tof_residual_mean  = mean(r)   over window
 *   tof_residual_var   = var(r)    over window
 *
 * Variance is computed as:  E[r^2] - (E[r])^2
 *
 * Corresponds to Python:
 *   window = residual[k-W+1 : k+1]
 *   tof_residual_var[k]  = np.var(window)
 *   tof_residual_mean[k] = np.mean(window)
 * ---------------------------------------------------------------- */
bool kf_get_residual_stats(const KalmanFilter *kf,
                           float *mean_out, float *var_out)
{
    const ResidualBuffer *rb = &kf->res_buf;

    if (rb->count < KF_WINDOW_SIZE) {
        if (mean_out) *mean_out = 0.0f;
        if (var_out)  *var_out  = 0.0f;
        return false;
    }

    float mean = rb->sum / (float)KF_WINDOW_SIZE;

    if (mean_out) {
        *mean_out = mean;
    }
    if (var_out) {
        /* Var = E[r^2] - (E[r])^2 */
        float sq_mean = rb->sq_sum / (float)KF_WINDOW_SIZE;
        *var_out = sq_mean - mean * mean;
    }
    return true;
}
