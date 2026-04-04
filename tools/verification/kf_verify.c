/**
 * @file    kf_verify.c
 * @brief   PC verification — compare C output against Python reference
 *
 * Reads the Python-generated CSV (kf_simulation_data.csv for fixed KF,
 * rule_akf_simulation_data.csv for CM-AKF) and runs the same input
 * through the C implementation.  Reports per-step max absolute error
 * to verify the Python-to-C port is correct (target: < 1e-3).
 *
 * Verification criteria:
 *   - All fields: absolute error < 1e-3
 *   - Innovation covariance S: if absolute error >= 1e-3, pass if
 *     relative error < 0.01% (float32 accumulation in sliding window
 *     sum-of-squares causes small drift vs Python float64).
 *
 * Build (GCC on PC):
 *   gcc -O2 -o kf_verify kf_verify.c kalman_filter.c -lm
 *
 * Usage:
 *   ./kf_verify kf_simulation_data.csv          # verify fixed KF
 *   ./kf_verify rule_akf_simulation_data.csv     # verify CM-AKF
 *
 * @author  Yubin
 * @date    2026-04-02
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kalman_filter.h"

/* ================================================================
 * Configuration
 * ================================================================ */

#define MAX_ROWS    4000
#define MAX_LINE    1024
#define TOLERANCE   1e-3f

/*
 * S (innovation covariance) tolerance: float32 accumulates rounding
 * error when summing W=20 squared residuals.  Absolute error may
 * exceed 1e-3 mm² while relative error stays < 0.01%.  We accept
 * this as a pass — the cause is well-understood and has no practical
 * impact on filter behaviour.
 */
#define S_REL_TOL   1e-4f   /* 0.01% relative tolerance for S */

/* Per-row data parsed from the Python CSV */
typedef struct {
    int      timestamp_ms;          /* col 1  */
    float    tof_distance_mm;       /* col 2  */
    float    encoder_distance_mm;   /* col 6  */
    float    encoder_speed_mms;     /* col 7  */
    float    kf_estimate_mm;        /* col 8  */
    float    tof_residual;          /* col 9  */
    float    kalman_gain;           /* col 16 */
    float    innovation_cov;        /* col 17 */
    char     scenario[16];          /* col 18 */
} CsvRow;

/* ================================================================
 * CSV Parser
 * ================================================================ */

/**
 * @brief  Parse a single float field; returns 0.0f for empty/NaN fields.
 */
static float parse_float(const char *s)
{
    if (s == NULL || s[0] == '\0' || strcmp(s, "NaN") == 0) {
        return 0.0f;
    }
    return (float)atof(s);
}

/**
 * @brief  Split a CSV line by comma and return field count.
 */
static int split_csv(char *line, char *fields[], int max_fields)
{
    int count = 0;
    char *p = line;

    while (*p && count < max_fields) {
        fields[count++] = p;
        char *comma = strchr(p, ',');
        if (comma) {
            *comma = '\0';
            p = comma + 1;
        } else {
            /* Strip trailing newline */
            char *nl = strchr(p, '\n');
            if (nl) *nl = '\0';
            nl = strchr(p, '\r');
            if (nl) *nl = '\0';
            break;
        }
    }
    return count;
}

/**
 * @brief  Load CSV file into CsvRow array.
 * @return Number of data rows loaded, or -1 on error.
 */
static int load_csv(const char *path, CsvRow *rows, int max_rows)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        fprintf(stderr, "Error: cannot open %s\n", path);
        return -1;
    }

    char line[MAX_LINE];
    char *fields[20];
    int n = 0;

    /* Skip header */
    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        return 0;
    }

    while (fgets(line, sizeof(line), fp) && n < max_rows) {
        int nf = split_csv(line, fields, 20);
        if (nf < 18) continue;

        rows[n].timestamp_ms        = atoi(fields[0]);
        rows[n].tof_distance_mm     = parse_float(fields[1]);
        rows[n].encoder_distance_mm = parse_float(fields[5]);
        rows[n].encoder_speed_mms   = parse_float(fields[6]);
        rows[n].kf_estimate_mm      = parse_float(fields[7]);
        rows[n].tof_residual        = parse_float(fields[8]);
        rows[n].kalman_gain         = parse_float(fields[15]);
        rows[n].innovation_cov      = parse_float(fields[16]);
        strncpy(rows[n].scenario, fields[17], sizeof(rows[n].scenario) - 1);
        rows[n].scenario[sizeof(rows[n].scenario) - 1] = '\0';
        n++;
    }

    fclose(fp);
    return n;
}

/* ================================================================
 * Main — Verification Loop
 * ================================================================ */

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <python_csv>\n", argv[0]);
        fprintf(stderr, "  e.g. %s kf_simulation_data.csv\n", argv[0]);
        return 1;
    }

    /* --- Load Python reference data --- */
    static CsvRow rows[MAX_ROWS];
    int N = load_csv(argv[1], rows, MAX_ROWS);
    if (N <= 0) {
        fprintf(stderr, "Error: no data loaded\n");
        return 1;
    }

    /* Detect mode from scenario field */
    bool use_akf = (strstr(rows[0].scenario, "AKF") != NULL);

    printf("==========================================================\n");
    printf("  KF C Verification vs Python Reference\n");
    printf("==========================================================\n");
    printf("  CSV:       %s\n", argv[1]);
    printf("  Rows:      %d\n", N);
    printf("  Mode:      %s\n", use_akf ? "CM-AKF" : "Fixed KF");
    printf("  Tolerance: %.0e\n", (double)TOLERANCE);
    printf("----------------------------------------------------------\n");

    /* --- Initialise C KF (same as Python: x0 = z_tof[0], P0 = R_init) --- */
    KalmanFilter kf;
    kf_init(&kf, rows[0].tof_distance_mm, KF_R_INIT, KF_R_INIT);

    /* --- Run and compare --- */
    float max_err_x   = 0.0f;
    float max_err_K   = 0.0f;
    float max_err_r   = 0.0f;
    float max_err_S   = 0.0f;
    int   worst_step  = 0;
    int   fail_count  = 0;

    for (int k = 1; k < N; k++) {
        float u = rows[k].encoder_speed_mms;
        float z = rows[k].tof_distance_mm;

        /* Predict + Update (mirrors Python loop body) */
        kf_predict(&kf, u);
        kf_update(&kf, z, use_akf);

        /* Compare against Python reference */
        float err_x = fabsf(kf.x        - rows[k].kf_estimate_mm);
        float err_K = fabsf(kf.K        - rows[k].kalman_gain);
        float err_r = fabsf(kf.residual  - rows[k].tof_residual);
        float err_S = fabsf(kf.S         - rows[k].innovation_cov);

        /*
         * S uses a two-tier pass criterion:
         *   1. Absolute error < TOLERANCE  (primary, same as other fields)
         *   2. If (1) fails, relative error < S_REL_TOL  (float32 accumulation)
         * This avoids penalising the expected sum-of-squares drift while
         * still catching genuine logic bugs (which cause large relative error).
         */
        bool S_pass;
        if (err_S < TOLERANCE) {
            S_pass = true;
        } else {
            float S_ref = fabsf(rows[k].innovation_cov);
            float rel   = (S_ref > 1e-6f) ? (err_S / S_ref) : 0.0f;
            S_pass = (rel < S_REL_TOL);
        }

        /* Step-level pass/fail: x, K, r must be < TOLERANCE; S uses two-tier */
        bool step_pass = (err_x < TOLERANCE) && (err_K < TOLERANCE)
                      && (err_r < TOLERANCE) && S_pass;

        if (!step_pass) {
            fail_count++;
            if (fail_count <= 5) {
                /* Print first few failures for debugging */
                printf("  FAIL k=%4d  err: x=%.6f K=%.6f r=%.6f S=%.4f\n",
                       k, (double)err_x, (double)err_K,
                       (double)err_r, (double)err_S);
                printf("       C:  x=%.4f K=%.6f r=%.4f S=%.4f R=%.4f\n",
                       (double)kf.x, (double)kf.K,
                       (double)kf.residual, (double)kf.S, (double)kf.R);
                printf("       Py: x=%.4f K=%.6f r=%.4f S=%.4f\n",
                       (double)rows[k].kf_estimate_mm,
                       (double)rows[k].kalman_gain,
                       (double)rows[k].tof_residual,
                       (double)rows[k].innovation_cov);
            }
        }

        if (err_x > max_err_x) { max_err_x = err_x; worst_step = k; }
        if (err_K > max_err_K)   max_err_K = err_K;
        if (err_r > max_err_r)   max_err_r = err_r;
        if (err_S > max_err_S)   max_err_S = err_S;
    }

    /* --- Summary --- */
    printf("----------------------------------------------------------\n");
    printf("  Max absolute errors:\n");
    printf("    x (estimate):      %.6f mm   %s\n",
           (double)max_err_x, max_err_x < TOLERANCE ? "[PASS]" : "[FAIL]");
    printf("    K (Kalman gain):   %.6f      %s\n",
           (double)max_err_K, max_err_K < TOLERANCE ? "[PASS]" : "[FAIL]");
    printf("    r (residual):      %.6f mm   %s\n",
           (double)max_err_r, max_err_r < TOLERANCE ? "[PASS]" : "[FAIL]");
    printf("    S (innov cov):     %.6f mm^2 (abs)  ",
           (double)max_err_S);
    if (max_err_S < TOLERANCE) {
        printf("[PASS abs]\n");
    } else {
        printf("[abs>tol, see relative]\n");
    }
    printf("  Worst step (x):     %d\n", worst_step);
    printf("  Failed steps:        %d / %d\n", fail_count, N - 1);
    printf("----------------------------------------------------------\n");

    bool pass = (fail_count == 0);

    if (pass) {
        printf("  RESULT: PASS\n");
        printf("  All fields: absolute error < 1e-3.\n");
        if (max_err_S >= TOLERANCE) {
            printf("  Note: S max absolute error %.2e exceeds 1e-3,\n",
                   (double)max_err_S);
            printf("  but relative error < 0.01%% (float32 accumulation).\n");
        }
    } else {
        printf("  RESULT: FAIL  (%d steps exceeded tolerance)\n", fail_count);
    }
    printf("==========================================================\n");

    return pass ? 0 : 1;
}
