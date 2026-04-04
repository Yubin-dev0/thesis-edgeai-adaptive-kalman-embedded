"""
KF Evaluation Metrics Module
졸업논문: Edge AI 기반 적응형 칼만 필터의 임베디드 실시간 적용 연구
저자: 임다영, 신유빈

사용법:
  python kf_eval_metrics.py E0_data.csv
  python kf_eval_metrics.py E0.csv E1.csv E2.csv
  python kf_eval_metrics.py E0.csv --threshold 10
  python kf_eval_metrics.py E1_run01.csv E1_run02.csv --repeat
"""

import sys, argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, List

plt.rcParams.update({
    "font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10,
    "legend.fontsize": 8, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "figure.dpi": 150, "savefig.dpi": 300,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linewidth": 0.5,
    "axes.axisbelow": True, "axes.spines.top": False, "axes.spines.right": False,
    "font.family": "serif", "mathtext.fontset": "dejavuserif",
})

C = {"gt": "#1a1a1a", "meas": "#d4735e", "kf": "#2b5ea7", "band": "#b8d4f0",
     "residual": "#d4a017", "resvar": "#2d8c4e", "threshold": "#c0392b",
     "bar1": "#2b5ea7", "bar2": "#d4735e"}

# ================================================================
# Core Metrics
# ================================================================

def rmse(a, b):
    return float(np.sqrt(np.nanmean((a - b) ** 2)))

def mae(a, b):
    return float(np.nanmean(np.abs(a - b)))

def convergence_time(time_s, estimate, truth, threshold=5.0, window=20):
    errors = (estimate - truth) ** 2
    n = len(errors)
    if n < window:
        return None
    cumsum = np.concatenate([[0], np.cumsum(errors)])
    w_rmse = np.sqrt((cumsum[window:] - cumsum[:-window]) / window)
    for i, val in enumerate(w_rmse):
        if val <= threshold:
            return float(time_s[i + window - 1])
    return None

def residual_bias(residual):
    return float(np.nanmean(residual))

def nis_pass_rate(residual, innov_cov, chi2_lo=0.0039, chi2_hi=5.024):
    valid = ~np.isnan(residual) & ~np.isnan(innov_cov) & (innov_cov > 0)
    if valid.sum() < 10:
        return None
    r, s = residual[valid], innov_cov[valid]
    nis = (r ** 2) / s
    return float(((nis >= chi2_lo) & (nis <= chi2_hi)).sum() / len(nis) * 100)


@dataclass
class Metrics:
    sensor_rmse: float; kf_rmse: float; improvement_pct: float
    sensor_mae: float; kf_mae: float
    convergence_s: Optional[float]; residual_mean: float
    nis_pass_pct: Optional[float]; r_est_rmse: Optional[float]
    n_samples: int; duration_s: float; scenario: str; threshold: float


# ================================================================
# KFEvaluator
# ================================================================

class KFEvaluator:
    def __init__(self, df, name=""):
        self.df = df
        self.name = name
        self._metrics = None

    @classmethod
    def from_csv(cls, path):
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"CSV not found: {path}")
        df = pd.read_csv(p)
        required = ["timestamp_ms","tof_distance_mm","kf_estimate_mm","gt_distance_mm","tof_residual"]
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise ValueError(f"Missing columns: {missing}")
        df["time_s"] = df["timestamp_ms"] / 1000.0
        name = p.stem
        if "scenario_id" in df.columns:
            sc = df["scenario_id"].dropna().unique()
            if len(sc) == 1:
                name = str(sc[0])
        return cls(df, name)

    def compute_metrics(self, threshold=5.0):
        df = self.df
        gt   = df["gt_distance_mm"].values
        meas = df["tof_distance_mm"].values
        kf   = df["kf_estimate_mm"].values
        res  = df["tof_residual"].values
        ts   = df["time_s"].values
        sr = rmse(meas, gt); kr = rmse(kf, gt)
        imp = (1 - kr / sr) * 100 if sr > 0 else 0.0
        conv = convergence_time(ts, kf, gt, threshold=threshold)
        nis = None
        if "innovation_cov" in df.columns:
            ic = pd.to_numeric(df["innovation_cov"], errors="coerce").values
            nis = nis_pass_rate(res, ic)
        elif "tof_residual_var" in df.columns:
            rv = pd.to_numeric(df["tof_residual_var"], errors="coerce").values
            nis = nis_pass_rate(res, rv)
        self._metrics = Metrics(
            sensor_rmse=sr, kf_rmse=kr, improvement_pct=imp,
            sensor_mae=mae(meas, gt), kf_mae=mae(kf, gt),
            convergence_s=conv, residual_mean=residual_bias(res),
            nis_pass_pct=nis, r_est_rmse=None, n_samples=len(df),
            duration_s=float(ts[-1]-ts[0]) if len(ts)>1 else 0,
            scenario=self.name, threshold=threshold)
        return self._metrics

    @property
    def metrics(self):
        if self._metrics is None: self.compute_metrics()
        return self._metrics

    def summary(self):
        m = self.metrics
        hz = m.n_samples / m.duration_s if m.duration_s > 0 else 0
        cs = f"{m.convergence_s:.3f} s" if m.convergence_s is not None else "    N/A"
        lines = ["",
            f"  +{'='*48}+",
            f"  | {'Scenario: '+m.scenario:^47s}|",
            f"  +{'='*48}+",
            f"  |  Sensor RMSE : {m.sensor_rmse:>8.2f} mm                  |",
            f"  |  KF RMSE     : {m.kf_rmse:>8.2f} mm   (down {m.improvement_pct:.1f}%)       |",
            f"  |  Sensor MAE  : {m.sensor_mae:>8.2f} mm                  |",
            f"  |  KF MAE      : {m.kf_mae:>8.2f} mm                  |",
            f"  |  Convergence : {cs:>9s}   (eps={m.threshold:.0f}mm)       |",
            f"  |  Resid. Mean : {m.residual_mean:>+8.2f} mm   (bias ~ 0?)      |"]
        if m.nis_pass_pct is not None:
            lines.append(
            f"  |  NIS Pass    : {m.nis_pass_pct:>8.1f} %   (target ~95%)    |")
        lines.extend([
            f"  +{'-'*48}+",
            f"  |  Samples: {m.n_samples}  Duration: {m.duration_s:.1f}s  Rate: {hz:.0f}Hz    |",
            f"  +{'='*48}+", ""])
        return "\n".join(lines)

    def plot_paper(self, out_path=None, show=False):
        m = self.metrics; df = self.df
        t    = df["time_s"].values
        gt   = df["gt_distance_mm"].values
        meas = df["tof_distance_mm"].values
        kf   = df["kf_estimate_mm"].values
        res  = df["tof_residual"].values

        fig, axes = plt.subplots(3, 1, figsize=(8, 7.5), sharex=True,
            gridspec_kw={"height_ratios": [3, 1.2, 1.2], "hspace": 0.08})

        # Panel 1: Position
        ax = axes[0]
        ax.plot(t, gt, color=C["gt"], lw=1.5, label="Ground truth", zorder=3)
        ax.scatter(t, meas, color=C["meas"], s=6, alpha=0.5,
                   edgecolors="none", label="VL53L0X measurement", zorder=2)
        ax.plot(t, kf, color=C["kf"], lw=1.2, label="KF estimate", zorder=4)
        ax.set_ylabel("Position (mm)")
        ax.legend(loc="upper left", framealpha=0.9, edgecolor="none")
        ax.set_title(
            f"1D Kalman Filter: Encoder + VL53L0X Sensor Fusion ({m.scenario})\n"
            f"RMSE: Sensor={m.sensor_rmse:.1f}mm -> KF={m.kf_rmse:.1f}mm "
            f"(down {m.improvement_pct:.0f}%)", fontsize=11)

        # Panel 2: Residual
        ax = axes[1]
        ax.plot(t, res, color=C["residual"], lw=0.6, alpha=0.8)
        ax.axhline(0, color="#666", lw=0.5, ls="-")
        if "R_label" in df.columns:
            rl = pd.to_numeric(df["R_label"], errors="coerce").values
            vr = rl[~np.isnan(rl)]
            if len(vr) > 0:
                med = np.median(vr); bd = 2*np.sqrt(med)
                ax.axhline(bd, color=C["threshold"], lw=0.8, ls="--",
                           label=f"+/-2*sqrt(R) = +/-{bd:.0f}mm")
                ax.axhline(-bd, color=C["threshold"], lw=0.8, ls="--")
                ax.legend(loc="upper left", framealpha=0.9, edgecolor="none")
        ax.set_ylabel("Residual (mm)")

        # Panel 3: Variance
        ax = axes[2]
        hd = False
        if "tof_residual_var" in df.columns:
            rv = pd.to_numeric(df["tof_residual_var"], errors="coerce").values
            ax.plot(t, rv, color=C["resvar"], lw=0.8, alpha=0.8, label="Residual var (W=20)")
            hd = True
        if "R_label" in df.columns:
            rl = pd.to_numeric(df["R_label"], errors="coerce").values
            ax.plot(t, rl, color=C["kf"], lw=0.8, alpha=0.6, ls="--", label="R_label (Cov.Matching)")
            hd = True
        ax.set_ylabel("Variance (mm^2)"); ax.set_xlabel("Time (s)")
        if hd: ax.legend(loc="upper left", framealpha=0.9, edgecolor="none")
        for a in axes: a.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))
        if out_path: fig.savefig(out_path, bbox_inches="tight"); print(f"[saved] {out_path}")
        if show: plt.show()
        else: plt.close(fig)


# ================================================================
# Compare Scenarios
# ================================================================

def compare_scenarios(evaluators, out_path=None, show=False):
    rows = []
    for ev in evaluators:
        m = ev.metrics
        rows.append({"Scenario":m.scenario,"Sensor RMSE":m.sensor_rmse,
            "KF RMSE":m.kf_rmse,"Improve(%)":m.improvement_pct,
            "Sensor MAE":m.sensor_mae,"KF MAE":m.kf_mae,
            "Conv.(s)":m.convergence_s if m.convergence_s else float("nan"),
            "Resid.Mean":m.residual_mean,"Samples":m.n_samples})
    table = pd.DataFrame(rows)
    print("\n"+"="*72+"\n  SCENARIO COMPARISON\n"+"="*72)
    fmt={"Sensor RMSE":"{:.2f}","KF RMSE":"{:.2f}","Improve(%)":"{:.1f}",
         "Sensor MAE":"{:.2f}","KF MAE":"{:.2f}","Conv.(s)":"{:.3f}","Resid.Mean":"{:+.2f}"}
    d=table.copy()
    for col,f in fmt.items():
        if col in d.columns:
            d[col]=d[col].apply(lambda x:f.format(x) if pd.notna(x) else "N/A")
    print(d.to_string(index=False)); print("="*72)

    if len(evaluators)>=2:
        fig,axes=plt.subplots(1,2,figsize=(10,4))
        sc=[ev.metrics.scenario for ev in evaluators]; x=np.arange(len(sc)); w=0.35
        ax=axes[0]
        ax.bar(x-w/2,[ev.metrics.sensor_rmse for ev in evaluators],w,label="Sensor",color=C["bar2"],alpha=0.8)
        ax.bar(x+w/2,[ev.metrics.kf_rmse for ev in evaluators],w,label="KF",color=C["bar1"],alpha=0.8)
        ax.set_xticks(x);ax.set_xticklabels(sc);ax.set_ylabel("RMSE (mm)");ax.set_title("RMSE Comparison");ax.legend()
        ax=axes[1]
        ax.bar(x-w/2,[ev.metrics.sensor_mae for ev in evaluators],w,label="Sensor",color=C["bar2"],alpha=0.8)
        ax.bar(x+w/2,[ev.metrics.kf_mae for ev in evaluators],w,label="KF",color=C["bar1"],alpha=0.8)
        ax.set_xticks(x);ax.set_xticklabels(sc);ax.set_ylabel("MAE (mm)");ax.set_title("MAE Comparison");ax.legend()
        plt.tight_layout()
        if out_path: fig.savefig(out_path,bbox_inches="tight"); print(f"[saved] {out_path}")
        if show: plt.show()
        else: plt.close(fig)
    return table


def compare_runs(evaluators, out_path=None):
    data={"Run":[],"KF RMSE":[],"KF MAE":[],"Sensor RMSE":[],"Conv.(s)":[],"Resid.Mean":[]}
    for i,ev in enumerate(evaluators):
        m=ev.metrics; data["Run"].append(ev.name or f"run{i+1:02d}")
        data["KF RMSE"].append(m.kf_rmse); data["KF MAE"].append(m.kf_mae)
        data["Sensor RMSE"].append(m.sensor_rmse)
        data["Conv.(s)"].append(m.convergence_s if m.convergence_s else float("nan"))
        data["Resid.Mean"].append(m.residual_mean)
    df=pd.DataFrame(data)
    print("\n"+"="*60+"\n  REPEATED RUNS SUMMARY\n"+"="*60)
    print(df.to_string(index=False,float_format="{:.3f}".format)); print("-"*60)
    for col in ["KF RMSE","KF MAE","Sensor RMSE","Conv.(s)","Resid.Mean"]:
        v=df[col].dropna()
        if len(v)>0: print(f"  {col:15s}: {v.mean():.2f} +/- {v.std():.2f}")
    print("="*60)
    if out_path: df.to_csv(out_path,index=False); print(f"[saved] {out_path}")
    return df


def main():
    parser=argparse.ArgumentParser(description="KF Evaluation Metrics")
    parser.add_argument("csv_files",nargs="+",help="CSV file paths")
    parser.add_argument("--threshold","-t",type=float,default=5.0)
    parser.add_argument("--repeat",action="store_true")
    parser.add_argument("--out","-o",type=str,default=None)
    parser.add_argument("--show",action="store_true")
    args=parser.parse_args()
    evs=[]
    for p in args.csv_files:
        ev=KFEvaluator.from_csv(p); ev.compute_metrics(threshold=args.threshold); evs.append(ev)
    od=Path(args.out) if args.out else Path(args.csv_files[0]).parent
    od.mkdir(parents=True,exist_ok=True)
    if len(evs)==1:
        print(evs[0].summary()); evs[0].plot_paper(str(od/f"{evs[0].name}_eval.png"),show=args.show)
    elif args.repeat:
        for ev in evs: print(ev.summary())
        compare_runs(evs,str(od/"runs_summary.csv"))
    else:
        for ev in evs:
            print(ev.summary()); ev.plot_paper(str(od/f"{ev.name}_eval.png"),show=args.show)
        compare_scenarios(evs,str(od/"scenario_comparison.png"),show=args.show)

if __name__=="__main__":
    main()
