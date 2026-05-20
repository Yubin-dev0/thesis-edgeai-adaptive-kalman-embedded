"""
F5 후처리 복원 스크립트

목적
----
어제 측정한 E1 데이터의 `tof_meas_rate` 컬럼은 펌웨어 F5 수정 전 버전이라
*ToF 변화율* 값으로 찍혀 있다. 논문 표 3-1 정의는 W=20 윈도우 내
`range_status==0` 비율이다. 본 스크립트는 CSV의 `tof_range_status`
컬럼을 이용해 F5를 논문 정의대로 후처리 복원하고, 별도 파일로 저장한다.

설계 결정
---------
- 원본 보존: `*_f5fixed.csv`로 새 파일 생성 (원본 덮어쓰지 않음).
- 윈도우 단위: CSV 행 단위 (50Hz CSV ≈ ToF 50Hz라 행 1:1 대응).
- 정의: F5(k) = (max(0, k-W+1) .. k 범위에서 status==0인 행 수) / (윈도우 내 행 수).
        윈도우가 아직 안 찬 초반(k<W-1)은 부분 비율.
- 컬럼 위치/이름: 25컬럼 그대로, `tof_meas_rate` 값만 교체.

검증
----
E1은 status가 100% 0이므로 결과는:
  - k=0   → 1/1 = 1.0
  - k=1   → 2/2 = 1.0
  ...
  - k>=19 → 20/20 = 1.0
즉 *전 행 1.0*이 나와야 정상. 만약 0.9 같은 값이 보이면 status에 0이 아닌
값이 섞여 있다는 뜻 (예외/누락 흔적).

사용법
------
    python postprocess_f5.py E1_run01.csv [E1_run02.csv ...]
    python postprocess_f5.py *.csv

출력은 같은 폴더에 `_f5fixed.csv` 접미사로 생성.
"""

import sys
import os
import pandas as pd

# 논문 표 3-1 정의: W=20 윈도우
WINDOW_SIZE = 20


def restore_f5(csv_path: str) -> str:
    """
    한 CSV 파일을 읽어 tof_meas_rate를 F5 (status==0 비율) 로 재계산한 뒤
    새 파일로 저장한다. 출력 경로를 반환한다.
    """
    df = pd.read_csv(csv_path)

    # 필수 컬럼 확인
    required = {"tof_range_status", "tof_meas_rate"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{csv_path}: 컬럼 누락 {missing}")

    # status==0 여부를 0/1로 (True/False → 1/0)
    is_status0 = (df["tof_range_status"] == 0).astype(int)

    # 슬라이딩 윈도우 평균 (min_periods=1: 초반 부분 윈도우도 계산)
    # rolling().mean()이 정확히 "윈도우 내 status==0 행 수 / 윈도우 내 행 수"
    f5_new = is_status0.rolling(window=WINDOW_SIZE, min_periods=1).mean()

    # 원본 컬럼 위치·다른 모든 값은 보존, tof_meas_rate만 교체
    df["tof_meas_rate"] = f5_new

    # 출력 경로: 원본_f5fixed.csv (같은 폴더)
    root, ext = os.path.splitext(csv_path)
    out_path = f"{root}_f5fixed{ext}"

    df.to_csv(out_path, index=False)
    return out_path


def verify(csv_path: str, out_path: str) -> dict:
    """
    검증용 요약 통계. 어제 확인한 E1 특성(status 100% 0)을 만족하면
    전 행 F5가 점진적으로 0→1로 증가하다 20행 이후 1.0이어야 한다.
    """
    df = pd.read_csv(csv_path)
    out = pd.read_csv(out_path)

    n = len(out)
    status_zero_ratio = (df["tof_range_status"] == 0).mean()
    f5 = out["tof_meas_rate"]

    return {
        "rows": n,
        "status0_ratio": status_zero_ratio,
        "f5_first": f5.iloc[0],
        "f5_at_19": f5.iloc[19] if n > 19 else None,  # 윈도우 처음 가득 차는 시점
        "f5_last": f5.iloc[-1],
        "f5_min": f5.min(),
        "f5_max": f5.max(),
        "f5_mean": f5.mean(),
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    print(f"F5 후처리 복원 (W={WINDOW_SIZE}, 정의: 윈도우 내 status==0 비율)")
    print("=" * 72)

    for csv_path in sys.argv[1:]:
        if not os.path.exists(csv_path):
            print(f"SKIP {csv_path} — 파일 없음")
            continue

        try:
            out_path = restore_f5(csv_path)
            stats = verify(csv_path, out_path)
        except Exception as e:
            print(f"FAIL {csv_path}: {e}")
            continue

        print(f"\n{os.path.basename(csv_path)} → {os.path.basename(out_path)}")
        print(f"  행 수            : {stats['rows']}")
        print(f"  status==0 비율   : {stats['status0_ratio']:.4f}")
        print(f"  F5 첫 행 (k=0)   : {stats['f5_first']:.4f}")
        print(f"  F5 k=19 (윈도우 가득): "
              f"{stats['f5_at_19']:.4f}" if stats['f5_at_19'] is not None else "  F5 k=19: N/A")
        print(f"  F5 마지막 행     : {stats['f5_last']:.4f}")
        print(f"  F5 범위          : [{stats['f5_min']:.4f}, {stats['f5_max']:.4f}]")
        print(f"  F5 평균          : {stats['f5_mean']:.4f}")

    print("\n" + "=" * 72)
    print("검증 기준 (E1 - status 100% 0):")
    print("  - status==0 비율 = 1.0")
    print("  - F5 첫 행 = 1.0  (1/1)")
    print("  - F5 k=19  = 1.0  (20/20)")
    print("  - F5 마지막 행 = 1.0")
    print("  - F5 범위 = [1.0, 1.0]")
    print("위 다섯 개가 다 1.0이면 후처리 정상.")


if __name__ == "__main__":
    main()
