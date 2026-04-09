/**
 * @file    kalman_filter.h
 * @brief   1D Kalman Filter with Covariance Matching Adaptive R
 *
 * Implements a scalar (1D) Kalman Filter for fusing encoder odometry
 * with VL53L0X ToF distance measurements.  Optionally enables
 * Covariance Matching (CM) adaptive R estimation [Mehra, 1970]
 * using a sliding-window circular buffer of innovation residuals.
 *
 * State model (1D linear):
 *   x(k) = A * x(k-1) + B * u(k)      (Prediction)
 *   z(k) = H * x(k) + v(k)             (Measurement)
 *
 * CM-AKF adaptive R:
 *   R(k) = (1/W) * sum(r_i^2) - P_pred(k)   , clamped to [R_MIN, R_MAX]
 *
 * Reference:
 *   Mehra, R. K. (1970). "On the identification of variances and
 *   adaptive Kalman filtering." IEEE Trans. Automatic Control.
 *
 * @author  Yubin
 * @date    2026-04-02
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 * Model Parameters
 * ================================================================
 * Kept as macros for easy tuning.  Values match the Python
 * simulation (kf_simulation_1D.py, cm_akf_1D.py).
 * ================================================================ */

#define KF_A        1.0f        /**< State transition (position preserving)   */
#define KF_B        0.005f      /**< Input gain = dt (5 ms at 200 Hz)         */
#define KF_H        1.0f        /**< Observation model (direct measurement)   */
#define KF_Q        1.0f        /**< Process noise variance                   */
#define KF_R_INIT   400.0f      /**< Initial measurement noise (20mm^2)       */

/* ================================================================
 * CM-AKF Parameters
 * ================================================================ */

#define KF_WINDOW_SIZE  20      /**< Sliding window W for CM residual stats   */
#define KF_R_MIN        1.0f    /**< R lower clamp  (prevents R <= 0)         */
#define KF_R_MAX        10000.0f/**< R upper clamp  (prevents filter lockout) */

/* ================================================================
 * Safety Guards
 * ================================================================ */

#define KF_P_FLOOR      1e-6f   /**< P minimum to prevent filter stall        */
#define KF_DENOM_GUARD  1e-6f   /**< Denominator guard against division by 0  */

/* ================================================================
 * Circular Buffer  (stores residuals for CM-AKF)
 * ================================================================
 * Maintains running sum and squared-sum for O(1) mean computation.
 * When count < W the buffer is not yet full; the CM update is
 * skipped and fixed R is used instead.
 * ================================================================ */

typedef struct {
    float buf[KF_WINDOW_SIZE];  /**< Ring buffer of residuals                 */
    float sum;                  /**< Running sum:    Sigma r_i                */
    float sq_sum;               /**< Running sq sum: Sigma r_i^2             */
    uint16_t idx;               /**< Next write position                      */
    uint16_t count;             /**< Number of valid entries (max = W)        */
} ResidualBuffer;

/* ================================================================
 * Kalman Filter State
 * ================================================================
 * All intermediate values needed for CSV logging (18-column format)
 * are stored here so that the caller can read them after each step.
 * ================================================================ */

typedef struct {
    /* --- Core state --- */
    float x;            /**< Posterior state estimate       (kf_estimate_mm)  */
    float P;            /**< Posterior error covariance      (P_covariance)   */

    /* --- Per-step outputs (updated every kf_update call) --- */
    float K;            /**< Kalman gain                    (kalman_gain)     */
    float S;            /**< Innovation covariance P_pred+R (innovation_cov)  */
    float residual;     /**< Innovation residual z - x_pred (tof_residual)    */
    float R;            /**< Current measurement noise R    (adaptive or fixed)*/

    /* --- Internal (not typically logged) --- */
    float x_pred;       /**< Prior state estimate  (after predict)            */
    float P_pred;       /**< Prior error covariance (after predict)           */

    /* --- Residual circular buffer for CM-AKF --- */
    ResidualBuffer res_buf;
} KalmanFilter;

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief  Initialise the Kalman Filter state.
 * @param  kf   Pointer to KalmanFilter instance
 * @param  x0   Initial state estimate  (typically first sensor reading)
 * @param  P0   Initial error covariance (typically KF_R_INIT)
 * @param  R0   Initial measurement noise variance
 */
void kf_init(KalmanFilter *kf, float x0, float P0, float R0);

/**
 * @brief  Prediction step:  x_pred = A*x + B*u,  P_pred = A*P*A + Q
 * @param  kf   Pointer to KalmanFilter instance
 * @param  u    Control input (encoder speed in mm/s)
 */
void kf_predict(KalmanFilter *kf, float u);

/**
 * @brief  Update step with new measurement.
 *
 * If use_akf is true and the residual buffer is full (count >= W),
 * the measurement noise R is adapted via Covariance Matching before
 * the standard KF update equations are applied.
 *
 * After this call the following fields are updated:
 *   kf->x, kf->P, kf->K, kf->S, kf->residual, kf->R
 *
 * @param  kf       Pointer to KalmanFilter instance
 * @param  z        Measurement value (VL53L0X distance in mm)
 * @param  use_akf  true = CM-AKF adaptive R,  false = fixed R
 */
void kf_update(KalmanFilter *kf, float z, bool use_akf);

/**
 * @brief  Query residual buffer statistics.
 * @param  kf       Pointer to KalmanFilter instance
 * @param  mean_out Pointer to receive residual mean  (NULL to skip)
 * @param  var_out  Pointer to receive residual variance (NULL to skip)
 * @retval true     if buffer is full (count >= W), outputs are valid
 * @retval false    if buffer not yet full, outputs are set to 0.0f
 */
bool kf_get_residual_stats(const KalmanFilter *kf,
                           float *mean_out, float *var_out);

#endif /* KALMAN_FILTER_H */
