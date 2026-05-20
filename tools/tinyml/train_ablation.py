"""
train_ablation.py — TinyML-AKF 6-feature 모델 학습 + 3-feature ablation
                    + INT8 양자화 + 평가

논문 [표 3-2]/[표 3-3]/[표 4-6] 그대로 구현.
2026-05-20 작성 (E1+E2+E3 실 데이터 17 run 확보 후).

[데이터 분할]
  학습   : E1 run01~03(f5fixed) + E2 {white,black,acryl} run01~02 + E3 run01~03  (12 runs)
  Test   : E2 {white,black,acryl} run03 + E3 run04~05                            ( 5 runs)
  Val    : 학습 12 run 내부에서 run 단위 split (EarlyStopping monitor)
            E2_acryl_run02 + E3_run03  ( 2 runs)
  warm-up: seq < 20 drop (CM 윈도우 미충전)

[입력 feature]
  6-feat (메인): F1 cm_residual, F2 cm_residual_var, F3 cm_residual_mean,
                F4 sensor_disagree, F5 tof_meas_rate, F6 tof_signal_rate
  3-feat (ablation): F1, F2, F3 (잔차 통계만)

[출력]
  log1p(cm_R) — log 공간 회귀 (논문 3.5.3절)

[모델 구조 — 논문 [표 3-2]]
  N → 16 (ReLU) → 8 (ReLU) → 1 (Linear)
  R 양수 보장은 추론 후 expm1 + clamp([1, 10000])로 처리 (모델 아키텍처 아님)

[정규화 — 논문 [표 3-3]]
  StandardScaler (μ, σ), E1 Run 1-3 (f5fixed) 데이터만으로 fit
  transform 시 clip 안 함 (OOD 신호 통과)

[학습 — 논문 [표 3-3]]
  Adam (lr=3e-3), MSE on log1p, batch=32, max_epoch=500
  EarlyStopping patience=20 on val_loss, restore_best_weights=True

[양자화 — 논문 [표 3-3]]
  TFLite post-training INT8 (full integer)
  Representative dataset: 학습 세트 임의 200 샘플
  목표: float32 대비 R 추정 ±5% 이내
"""

import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers


# ─────────────────────────────────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────────────────────────────────

FEATURES_6 = [
    'cm_residual',       # F1
    'cm_residual_var',   # F2
    'cm_residual_mean',  # F3
    'sensor_disagree',   # F4
    'tof_meas_rate',     # F5  — 학습 데이터에선 상수 1.0
    'tof_signal_rate',   # F6
]
FEATURES_3 = FEATURES_6[:3]
R_LABEL_COL = 'cm_R'
WARMUP_SEQ  = 20

TRAIN_FILES = [
    'E1_run01_f5fixed.csv', 'E1_run02_f5fixed.csv', 'E1_run03_f5fixed.csv',
    'E2_white_run01.csv', 'E2_white_run02.csv',
    'E2_black_run01.csv', 'E2_black_run02.csv',
    'E2_acryl_run01.csv', 'E2_acryl_run02.csv',
    'E3_run01.csv', 'E3_run02.csv', 'E3_run03.csv',
]
TEST_FILES = [
    'E2_white_run03.csv', 'E2_black_run03.csv', 'E2_acryl_run03.csv',
    'E3_run04.csv', 'E3_run05.csv',
]
NORM_FIT_FILES = [
    'E1_run01_f5fixed.csv', 'E1_run02_f5fixed.csv', 'E1_run03_f5fixed.csv',
]
VAL_RUNS = {'E2_acryl_run02.csv', 'E3_run03.csv'}


def load_csv(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df = df[df['seq'] >= WARMUP_SEQ].copy()
    df['source_file'] = path.name
    return df


def load_files(data_dir: Path, filenames: list) -> pd.DataFrame:
    dfs = []
    for fn in filenames:
        p = data_dir / fn
        if not p.exists():
            raise FileNotFoundError(f"missing: {p}")
        d = load_csv(p)
        dfs.append(d)
        print(f"  {fn:35s}  {len(d):4d} rows kept")
    return pd.concat(dfs, ignore_index=True)


def extract_xy(df: pd.DataFrame, feature_cols: list):
    for c in feature_cols + [R_LABEL_COL]:
        if df[c].isna().any():
            raise RuntimeError(f"NaN in column {c}")
    X = df[feature_cols].values.astype(np.float32)
    y = np.log1p(df[R_LABEL_COL].values.astype(np.float32))
    return X, y


def fit_standard(X: np.ndarray, feature_names: list) -> dict:
    mean = X.mean(axis=0)
    std  = X.std(axis=0)
    std_safe = np.where(std < 1e-8, 1.0, std)
    return {
        'method':      'standard',
        'fit_on':      'E1 Run 1-3 (f5fixed) — paper [표 3-3]',
        'features':    feature_names,
        'mean':        mean.tolist(),
        'std':         std_safe.tolist(),
        'std_is_safe': (std < 1e-8).tolist(),
    }


def apply_standard(X: np.ndarray, params: dict) -> np.ndarray:
    mean = np.array(params['mean'], dtype=np.float32)
    std  = np.array(params['std'],  dtype=np.float32)
    return ((X - mean) / std).astype(np.float32)


def build_mlp(input_dim: int, name: str) -> keras.Model:
    """N → 16 (ReLU) → 8 (ReLU) → 1 (Linear). 논문 [표 3-2]."""
    inp = layers.Input(shape=(input_dim,), name='features')
    x = layers.Dense(16, activation='relu', name='hidden1')(inp)
    x = layers.Dense( 8, activation='relu', name='hidden2')(x)
    out = layers.Dense(1, activation='linear', name='log_R_estimate')(x)
    return keras.Model(inputs=inp, outputs=out, name=name)


def train_model(model, X_tr, y_tr, X_val, y_val, max_epoch=500, patience=20):
    model.compile(
        optimizer=keras.optimizers.Adam(learning_rate=3e-3),
        loss='mse',
    )
    es = keras.callbacks.EarlyStopping(
        monitor='val_loss',
        patience=patience,
        restore_best_weights=True,
        verbose=0,
    )
    hist = model.fit(
        X_tr, y_tr,
        validation_data=(X_val, y_val),
        epochs=max_epoch,
        batch_size=32,
        callbacks=[es],
        verbose=0,
    )
    return hist


def quantize_int8(model, X_calib_subset, out_path: Path):
    def representative_data_gen():
        for i in range(len(X_calib_subset)):
            yield [X_calib_subset[i:i+1].astype(np.float32)]

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_data_gen
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type  = tf.int8
    converter.inference_output_type = tf.int8
    tflite_bytes = converter.convert()
    out_path.write_bytes(tflite_bytes)
    return len(tflite_bytes)


def tflite_predict(tflite_path: Path, X: np.ndarray) -> np.ndarray:
    interp = tf.lite.Interpreter(model_path=str(tflite_path))
    interp.allocate_tensors()
    in_d  = interp.get_input_details()[0]
    out_d = interp.get_output_details()[0]
    in_scale,  in_zp  = in_d['quantization']
    out_scale, out_zp = out_d['quantization']

    preds = np.zeros(len(X), dtype=np.float32)
    for i in range(len(X)):
        x_q = np.round(X[i] / in_scale + in_zp).clip(-128, 127).astype(np.int8)
        interp.set_tensor(in_d['index'], x_q.reshape(1, -1))
        interp.invoke()
        y_q = interp.get_tensor(out_d['index'])[0, 0]
        preds[i] = (float(y_q) - out_zp) * out_scale
    return preds


def evaluate(y_true_log, y_pred_log, label=""):
    mae_log = float(np.mean(np.abs(y_true_log - y_pred_log)))
    R_true = np.clip(np.expm1(y_true_log), 1.0, 10000.0)
    R_pred = np.clip(np.expm1(y_pred_log), 1.0, 10000.0)
    mae_R  = float(np.mean(np.abs(R_true - R_pred)))
    mape_R = float(np.mean(np.abs(R_true - R_pred) / np.maximum(R_true, 1.0)) * 100)
    return {
        'label':  label, 'n': int(len(y_true_log)),
        'mae_log': mae_log, 'mae_R': mae_R, 'mape_R': mape_R,
        'R_mean': float(R_true.mean()), 'R_max': float(R_true.max()),
    }


def run_pipeline(data_dir: Path, out_dir: Path, seed=42):
    np.random.seed(seed)
    tf.random.set_seed(seed)
    tf.keras.utils.set_random_seed(seed)

    models_dir  = out_dir / 'models'
    results_dir = out_dir / 'results'
    models_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)

    print("="*70)
    print("[1] Loading data")
    print("="*70)
    print(f"\n  Training set ({len(TRAIN_FILES)} runs):")
    train_df = load_files(data_dir, TRAIN_FILES)
    print(f"\n  Test set ({len(TEST_FILES)} runs):")
    test_df  = load_files(data_dir, TEST_FILES)
    print(f"\n  total train: {len(train_df)} rows  |  total test: {len(test_df)} rows")

    print("\n" + "="*70)
    print("[2] Train/Val split (run-based)")
    print("="*70)
    val_mask  = train_df['source_file'].isin(VAL_RUNS)
    tr_df     = train_df[~val_mask].copy()
    val_df    = train_df[val_mask].copy()
    print(f"  train: {len(tr_df)} rows from {tr_df['source_file'].nunique()} runs")
    print(f"  val  : {len(val_df)} rows from {val_df['source_file'].nunique()} runs ({sorted(VAL_RUNS)})")

    print("\n" + "="*70)
    print("[3] Normalization (StandardScaler, fit on E1 Run 1-3)")
    print("="*70)
    norm_df = load_files(data_dir, NORM_FIT_FILES)
    X_norm_fit, _ = extract_xy(norm_df, FEATURES_6)
    norm_params = fit_standard(X_norm_fit, FEATURES_6)
    print(f"\n  μ: {[f'{m:7.3f}' for m in norm_params['mean']]}")
    print(f"  σ: {[f'{s:7.3f}' for s in norm_params['std']]}")
    safe_feats = [FEATURES_6[i] for i, b in enumerate(norm_params['std_is_safe']) if b]
    if safe_feats:
        print(f"  σ-safe applied (std=0 features): {safe_feats}")

    norm_params_3 = {
        'method':      'standard',
        'fit_on':      'E1 Run 1-3 (f5fixed) — paper [표 3-3]',
        'features':    FEATURES_3,
        'mean':        norm_params['mean'][:3],
        'std':         norm_params['std'][:3],
        'std_is_safe': norm_params['std_is_safe'][:3],
    }

    nparams_path = models_dir / 'normalization_params.json'
    with open(nparams_path, 'w') as f:
        json.dump({
            '6feat': norm_params,
            '3feat': norm_params_3,
            'note': ('Firmware should hardcode mean[i] and std[i] arrays as float32. '
                     'For each feature: x_norm = (x - mean[i]) / std[i]. No clipping. '
                     'F5 (tof_meas_rate) has std=0 in training data → mapped to std_safe=1.0; '
                     'normalized value is always (1.0 - 1.0) / 1.0 = 0.'),
        }, f, indent=2)
    print(f"\n  saved: {nparams_path}")

    print("\n" + "="*70)
    print("[4] Training 6-feature MAIN model")
    print("="*70)
    X_tr_6,  y_tr  = extract_xy(tr_df,   FEATURES_6)
    X_val_6, y_val = extract_xy(val_df,  FEATURES_6)
    X_te_6,  y_te  = extract_xy(test_df, FEATURES_6)
    X_tr_6n  = apply_standard(X_tr_6,  norm_params)
    X_val_6n = apply_standard(X_val_6, norm_params)
    X_te_6n  = apply_standard(X_te_6,  norm_params)
    print(f"  X_train: {X_tr_6n.shape}  X_val: {X_val_6n.shape}  X_test: {X_te_6n.shape}")
    print(f"  y range (log) train=[{y_tr.min():.3f},{y_tr.max():.3f}]  "
          f"val=[{y_val.min():.3f},{y_val.max():.3f}]  "
          f"test=[{y_te.min():.3f},{y_te.max():.3f}]")

    model_6 = build_mlp(input_dim=6, name='tinyml_akf_6feat')
    n_params_6 = model_6.count_params()
    print(f"\n  params: {n_params_6}  (paper [표 3-2]: 257)")

    hist_6 = train_model(model_6, X_tr_6n, y_tr, X_val_6n, y_val)
    final_epoch_6 = len(hist_6.history['loss'])
    print(f"  trained {final_epoch_6} epochs")
    print(f"  best val_loss: {min(hist_6.history['val_loss']):.4f}")

    y_pred_tr_6  = model_6.predict(X_tr_6n,  verbose=0).flatten()
    y_pred_val_6 = model_6.predict(X_val_6n, verbose=0).flatten()
    y_pred_te_6  = model_6.predict(X_te_6n,  verbose=0).flatten()
    eval_f32_6 = {
        'train': evaluate(y_tr,  y_pred_tr_6,  '6feat-train-f32'),
        'val':   evaluate(y_val, y_pred_val_6, '6feat-val-f32'),
        'test':  evaluate(y_te,  y_pred_te_6,  '6feat-test-f32'),
    }
    for k, v in eval_f32_6.items():
        print(f"  [f32 {k:5s}] n={v['n']:4d}  MAE_log={v['mae_log']:.4f}  "
              f"MAE_R={v['mae_R']:7.2f}  MAPE_R={v['mape_R']:5.1f}%  "
              f"R̄={v['R_mean']:7.1f}")
    model_6.save(models_dir / '6feat_main.keras')

    print("\n" + "="*70)
    print("[5] Training 3-feature ABLATION model")
    print("="*70)
    X_tr_3,  _ = extract_xy(tr_df,   FEATURES_3)
    X_val_3, _ = extract_xy(val_df,  FEATURES_3)
    X_te_3,  _ = extract_xy(test_df, FEATURES_3)
    X_tr_3n  = apply_standard(X_tr_3,  norm_params_3)
    X_val_3n = apply_standard(X_val_3, norm_params_3)
    X_te_3n  = apply_standard(X_te_3,  norm_params_3)

    model_3 = build_mlp(input_dim=3, name='tinyml_akf_3feat')
    n_params_3 = model_3.count_params()
    print(f"  params: {n_params_3}  (paper note: 209)")
    hist_3 = train_model(model_3, X_tr_3n, y_tr, X_val_3n, y_val)
    final_epoch_3 = len(hist_3.history['loss'])
    print(f"  trained {final_epoch_3} epochs")
    print(f"  best val_loss: {min(hist_3.history['val_loss']):.4f}")

    y_pred_tr_3  = model_3.predict(X_tr_3n,  verbose=0).flatten()
    y_pred_val_3 = model_3.predict(X_val_3n, verbose=0).flatten()
    y_pred_te_3  = model_3.predict(X_te_3n,  verbose=0).flatten()
    eval_f32_3 = {
        'train': evaluate(y_tr,  y_pred_tr_3,  '3feat-train-f32'),
        'val':   evaluate(y_val, y_pred_val_3, '3feat-val-f32'),
        'test':  evaluate(y_te,  y_pred_te_3,  '3feat-test-f32'),
    }
    for k, v in eval_f32_3.items():
        print(f"  [f32 {k:5s}] n={v['n']:4d}  MAE_log={v['mae_log']:.4f}  "
              f"MAE_R={v['mae_R']:7.2f}  MAPE_R={v['mape_R']:5.1f}%")
    model_3.save(models_dir / '3feat_ablation.keras')

    print("\n" + "="*70)
    print("[6] INT8 quantization (representative = 200 samples)")
    print("="*70)
    n_calib = min(200, len(X_tr_6n))
    calib_idx = np.random.choice(len(X_tr_6n), size=n_calib, replace=False)
    X_calib_6 = X_tr_6n[calib_idx]
    X_calib_3 = X_tr_3n[calib_idx]

    tflite_6_path = models_dir / '6feat_main.tflite'
    tflite_3_path = models_dir / '3feat_ablation.tflite'
    size_6 = quantize_int8(model_6, X_calib_6, tflite_6_path)
    size_3 = quantize_int8(model_3, X_calib_3, tflite_3_path)
    print(f"  6feat_main.tflite     : {size_6:5d} bytes ({size_6/1024:.2f} KB)  paper: ~3.16 KB")
    print(f"  3feat_ablation.tflite : {size_3:5d} bytes ({size_3/1024:.2f} KB)")

    print("\n" + "="*70)
    print("[7] INT8 vs float32 — quantization degradation")
    print("="*70)
    y_int8_te_6 = tflite_predict(tflite_6_path, X_te_6n)
    y_int8_te_3 = tflite_predict(tflite_3_path, X_te_3n)
    eval_int8_6 = evaluate(y_te, y_int8_te_6, '6feat-test-int8')
    eval_int8_3 = evaluate(y_te, y_int8_te_3, '3feat-test-int8')

    def deg(int8_v, f32_v):
        if f32_v['mae_R'] < 1e-6:
            return 0.0
        return (int8_v['mae_R'] - f32_v['mae_R']) / f32_v['mae_R'] * 100

    print(f"  6feat: f32 MAE_R={eval_f32_6['test']['mae_R']:6.2f}  "
          f"INT8 MAE_R={eval_int8_6['mae_R']:6.2f}  "
          f"deg={deg(eval_int8_6, eval_f32_6['test']):+.2f}%  (target: ±5%)")
    print(f"  3feat: f32 MAE_R={eval_f32_3['test']['mae_R']:6.2f}  "
          f"INT8 MAE_R={eval_int8_3['mae_R']:6.2f}  "
          f"deg={deg(eval_int8_3, eval_f32_3['test']):+.2f}%")

    print("\n" + "="*70)
    print("[8] Saving outputs")
    print("="*70)
    pred_df = test_df[['source_file', 'seq', 'scenario_id', 'tof_distance_mm',
                       'cm_R'] + FEATURES_6].copy()
    pred_df['log1p_cm_R_true']      = y_te
    pred_df['log1p_cm_R_pred_6f32'] = y_pred_te_6
    pred_df['log1p_cm_R_pred_6int'] = y_int8_te_6
    pred_df['log1p_cm_R_pred_3f32'] = y_pred_te_3
    pred_df['log1p_cm_R_pred_3int'] = y_int8_te_3
    pred_df['R_pred_6f32'] = np.clip(np.expm1(y_pred_te_6), 1.0, 10000.0)
    pred_df['R_pred_6int'] = np.clip(np.expm1(y_int8_te_6), 1.0, 10000.0)
    pred_df['R_pred_3f32'] = np.clip(np.expm1(y_pred_te_3), 1.0, 10000.0)
    pred_df['R_pred_3int'] = np.clip(np.expm1(y_int8_te_3), 1.0, 10000.0)
    pred_path = results_dir / 'test_predictions.csv'
    pred_df.to_csv(pred_path, index=False)
    print(f"  saved: {pred_path}")

    report_path = results_dir / 'training_report.md'
    with open(report_path, 'w') as f:
        f.write("# TinyML-AKF Training Report\n\n")
        f.write("Generated: 2026-05-20  \n")
        f.write("Data: 17 runs (E1×3 f5fixed + E2×9 + E3×5), warm-up seq<20 dropped  \n")
        f.write("Paper reference: [표 3-2], [표 3-3], [표 4-6]\n\n")

        f.write("## Data split\n\n")
        f.write(f"- **Train**: {len(tr_df)} rows from {tr_df['source_file'].nunique()} runs\n")
        f.write(f"- **Val**  : {len(val_df)} rows from {val_df['source_file'].nunique()} runs ({', '.join(sorted(VAL_RUNS))})\n")
        f.write(f"- **Test** : {len(test_df)} rows from {test_df['source_file'].nunique()} runs ({', '.join(TEST_FILES)})\n\n")

        f.write("## Normalization (StandardScaler, fit on E1 Run 1-3)\n\n")
        f.write("| Feature | μ | σ |\n|---|---|---|\n")
        for i, fn in enumerate(FEATURES_6):
            f.write(f"| {fn} | {norm_params['mean'][i]:.4f} | {norm_params['std'][i]:.4f} |\n")
        f.write("\n")

        f.write("## 6-feature MAIN model\n\n")
        f.write(f"- Params: {n_params_6} (paper: 257)\n")
        f.write(f"- Epochs trained: {final_epoch_6}\n")
        f.write(f"- Best val_loss: {min(hist_6.history['val_loss']):.4f}\n")
        f.write(f"- TFLite size: {size_6} bytes ({size_6/1024:.2f} KB; paper: ~3.16 KB)\n\n")
        f.write("| Split | n | MAE_log32 | MAE_R32 | MAPE_R32 | MAE_Rint8 | quant deg |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for k in ['train', 'val', 'test']:
            v = eval_f32_6[k]
            if k == 'test':
                f.write(f"| {k} | {v['n']} | {v['mae_log']:.4f} | {v['mae_R']:.2f} | {v['mape_R']:.1f}% | {eval_int8_6['mae_R']:.2f} | {deg(eval_int8_6, eval_f32_6['test']):+.2f}% |\n")
            else:
                f.write(f"| {k} | {v['n']} | {v['mae_log']:.4f} | {v['mae_R']:.2f} | {v['mape_R']:.1f}% | — | — |\n")
        f.write("\n")

        f.write("## 3-feature ABLATION model\n\n")
        f.write(f"- Params: {n_params_3} (paper: 209)\n")
        f.write(f"- Epochs trained: {final_epoch_3}\n")
        f.write(f"- Best val_loss: {min(hist_3.history['val_loss']):.4f}\n")
        f.write(f"- TFLite size: {size_3} bytes ({size_3/1024:.2f} KB)\n\n")
        f.write("| Split | n | MAE_log32 | MAE_R32 | MAPE_R32 | MAE_Rint8 | quant deg |\n")
        f.write("|---|---|---|---|---|---|---|\n")
        for k in ['train', 'val', 'test']:
            v = eval_f32_3[k]
            if k == 'test':
                f.write(f"| {k} | {v['n']} | {v['mae_log']:.4f} | {v['mae_R']:.2f} | {v['mape_R']:.1f}% | {eval_int8_3['mae_R']:.2f} | {deg(eval_int8_3, eval_f32_3['test']):+.2f}% |\n")
            else:
                f.write(f"| {k} | {v['n']} | {v['mae_log']:.4f} | {v['mae_R']:.2f} | {v['mape_R']:.1f}% | — | — |\n")
        f.write("\n")

        f.write("## Ablation comparison (test set)\n\n")
        f.write(f"- 6-feature MAE_R (float32): **{eval_f32_6['test']['mae_R']:.2f}** mm²\n")
        f.write(f"- 3-feature MAE_R (float32): **{eval_f32_3['test']['mae_R']:.2f}** mm²\n")
        diff = eval_f32_3['test']['mae_R'] - eval_f32_6['test']['mae_R']
        f.write(f"- Difference: {diff:+.2f} mm² ({'6feat better' if diff > 0 else '3feat better'})\n\n")

        f.write("## Notes for paper Ch.5\n\n")
        f.write("- F5 (`tof_meas_rate`) was constant 1.0 across all 17 training runs ")
        f.write("(status≠0 = 0 in all runs, including E3 dynamic occlusion which produced outlier distance values but kept status=0). ")
        f.write("Normalized to constant 0 via std-safe (σ→1). 6feat vs 3feat ablation effectively isolates the contribution of F4 (sensor_disagree) and F6 (tof_signal_rate).\n")
        f.write("- F5 is retained in the 6-feature input as a reserve channel for environments where it can vary.\n")
        f.write("- Normalization fit data: E1 Run 1-3 only (paper [표 3-3]). E2/E3 features go outside [μ-3σ, μ+3σ] — used as out-of-distribution signal by the model, not clipped.\n")
        f.write("- R label clamp [1, 10000] is already applied in firmware (`kalman_filter.c` CM-AKF update). Training applies log1p only.\n")
        f.write("- E2 data split: paper [표 4-6] specifies Run 4-5 evaluation, but only 3 runs per surface were measured. Adjusted to Run 01-02 train / Run 03 test (run-unit, preserves data-leakage isolation principle).\n")

    print(f"  saved: {report_path}")

    print("\n" + "="*70)
    print("DONE")
    print("="*70)
    print(f"\n  6feat test MAE_R (f32):  {eval_f32_6['test']['mae_R']:.2f}")
    print(f"  6feat test MAE_R (int8): {eval_int8_6['mae_R']:.2f}")
    print(f"  6feat quant deg:         {deg(eval_int8_6, eval_f32_6['test']):+.2f}%  (target: ±5%)\n")
    print(f"  3feat test MAE_R (f32):  {eval_f32_3['test']['mae_R']:.2f}")
    print(f"  3feat test MAE_R (int8): {eval_int8_3['mae_R']:.2f}")
    print(f"  3feat quant deg:         {deg(eval_int8_3, eval_f32_3['test']):+.2f}%\n")
    print(f"  Artifacts:")
    print(f"    {models_dir}/normalization_params.json")
    print(f"    {models_dir}/6feat_main.tflite     ← deploy to MCU")
    print(f"    {models_dir}/3feat_ablation.tflite ← PC comparison only")
    print(f"    {models_dir}/*.keras")
    print(f"    {results_dir}/training_report.md")
    print(f"    {results_dir}/test_predictions.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='.', help='directory containing CSV files')
    ap.add_argument('--out-dir',  default='.', help='output root (creates models/ and results/)')
    ap.add_argument('--seed',     type=int, default=42)
    args = ap.parse_args()
    run_pipeline(Path(args.data_dir), Path(args.out_dir), seed=args.seed)


if __name__ == '__main__':
    main()
