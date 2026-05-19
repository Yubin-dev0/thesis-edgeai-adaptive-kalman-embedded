#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC Serial Logger (CLI, auto-header) for KF/AKF Robot Experiment
================================================================
Command-line CSV logger for the Bluetooth data stream from the MCU.
No GUI - this is a deliberate design choice. The tkinter version froze
because the GUI event loop, the receive thread and periodic stat
updates contended with each other. A CLI tool has none of that: there
is exactly one thread doing one thing, so it cannot dead-lock or hang.

The data path (auto-header parsing + integrity validation) is the same
logic as the GUI version and was unit-tested separately.

Usage
-----
    python logger_cli.py <scenario> <run> [port]

    scenario : E0 / E1 / E2 / E3 / E4 / E5
    run      : run number (integer, e.g. 1)
    port     : optional COM port (e.g. COM6). If omitted, the logger
               auto-detects; if exactly one port exists it is used,
               otherwise you are asked to pass it explicitly.

Examples
    python logger_cli.py E1 1            # auto-detect port
    python logger_cli.py E1 2 COM6       # explicit port

Behaviour
---------
1. Opens the port, then prompts you to reset the MCU.
2. Waits for the firmware '# CSV_HEADER:' line, parses the column
   layout from it, and writes that header to the CSV.
3. Streams rows to the CSV, printing a live count to the console.
4. The firmware stops after ~5 s. When no data has arrived for
   IDLE_TIMEOUT_S seconds, the logger finalises the file, prints a
   summary and exits. Ctrl+C also stops cleanly at any time.

Output file: <scenario>_run<NN>_<date>_<time>.csv  in OUTPUT_DIR.

Requirements:
    pip install pyserial
"""

import sys
import csv
import time
import signal
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial이 설치되어 있지 않습니다.")
    print("설치:  py -m pip install pyserial")
    sys.exit(1)

# ──────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────
BAUD_RATE = 115200
SCENARIOS = ["E0", "E1", "E2", "E3", "E4", "E5"]
OUTPUT_DIR = Path.home() / "KF_Experiment_Data"

HEADER_PREFIX = "# CSV_HEADER:"

# 데이터가 이 시간(초) 동안 안 들어오면 측정이 끝난 것으로 보고 종료.
# 펌웨어는 5초(1000루프) 동안만 전송하므로, 그보다 넉넉히 잡되
# 사람이 굴림을 준비하는 시간보다는 짧게.
IDLE_TIMEOUT_S = 3.0

# 측정 시작 후 첫 데이터가 이 시간 안에 안 오면 경고.
FIRST_DATA_WARN_S = 15.0

# 타임스탬프 간격 검증 (ms). 50Hz 로깅 = 20ms.
TIMESTAMP_INTERVAL_MS = 20
TIMESTAMP_TOLERANCE = 0.5


def is_firmware_noise(line: str) -> bool:
    """데이터가 아닌 펌웨어 출력 줄(배너/결과/PuTTY 배너)인지."""
    return line.startswith("#") or line.startswith("=~=~=")


# 헤더(# CSV_HEADER:) 줄이 유실됐을 때만 발동하는 폴백 임계값.
# '데이터처럼 보이는' 줄이 이 개수 이상 연속으로 쌓이면, 헤더가
# 정말 안 온 것으로 보고 폴백 헤더를 만든다. 부팅 배너 몇 줄에
# 잘못 발동하지 않도록 충분히 크게 잡는다.
FALLBACK_TRIGGER = 10


def _looks_like_data_row(line: str) -> bool:
    """줄이 CSV 데이터 행처럼 보이는지 판단한다.

    펌웨어 데이터 행은 쉼표로 구분된 십수 개의 숫자 필드다. 부팅
    배너('==== ', 'KF: Q=1.00, ...')와 구분하기 위해, 쉼표가 충분히
    많고(>=8) 첫 토큰이 숫자로 파싱되는 줄만 데이터로 본다.
    'KF: Q=1.00, R_INIT=400.0, W=20' 같은 배너는 첫 토큰('KF: Q=1.00')
    이 숫자가 아니므로 걸러진다.
    """
    parts = line.split(",")
    if len(parts) < 8:
        return False
    try:
        float(parts[0])
        return True
    except ValueError:
        return False


# ──────────────────────────────────────────────
# 데이터 검증 (헤더 자동 적응) - GUI 버전과 동일 로직
# ──────────────────────────────────────────────
class DataValidator:
    """수신 데이터의 무결성을 검증한다.

    컬럼 레이아웃을 고정하지 않는다. set_header()로 펌웨어 헤더를
    전달받으면 'timestamp_ms'와 'seq' 컬럼 위치를 이름으로 찾아 두고,
    그 위치 기준으로 검증한다.
    """

    def __init__(self):
        self.header = None
        self.n_fields = None
        self.idx_ts = None
        self.idx_seq = None

        self.last_timestamp = None
        self.last_seq = None

        self.total_lines = 0
        self.valid_lines = 0
        self.field_errors = 0
        self.timestamp_gaps = 0
        self.timestamp_duplicates = 0
        self.seq_drops = 0

    def set_header(self, columns):
        self.header = [c.strip() for c in columns]
        self.n_fields = len(self.header)
        lookup = {c.lower(): i for i, c in enumerate(self.header)}
        self.idx_ts = lookup.get("timestamp_ms")
        self.idx_seq = lookup.get("seq")

        msg = f"  헤더 인식: {self.n_fields}개 컬럼"
        if self.idx_ts is not None:
            msg += f", timestamp_ms=#{self.idx_ts}"
        else:
            msg += ", timestamp 컬럼 없음(시간검증 생략)"
        if self.idx_seq is not None:
            msg += f", seq=#{self.idx_seq}"
        print(msg)

    def set_header_fallback(self, n_fields):
        self.header = [f"col{i}" for i in range(n_fields)]
        self.n_fields = n_fields
        self.idx_ts = None
        self.idx_seq = None
        print(f"  [경고] 펌웨어 헤더(# CSV_HEADER:)를 못 받음 - "
              f"{n_fields}개 필드 기준 진행(시간검증 생략)")

    def validate_line(self, fields):
        self.total_lines += 1

        if self.n_fields is not None and len(fields) != self.n_fields:
            self.field_errors += 1
            return False, f"필드 수 불일치: {len(fields)} (기대 {self.n_fields})"

        warn = ""

        if self.idx_seq is not None and self.idx_seq < len(fields):
            try:
                seq = int(float(fields[self.idx_seq]))
            except ValueError:
                self.field_errors += 1
                return False, f"seq 파싱 실패"
            if self.last_seq is not None:
                step = seq - self.last_seq
                if step <= 0:
                    warn = f"seq 비정상 {self.last_seq}->{seq}"
                elif step > 1:
                    self.seq_drops += (step - 1)
                    warn = f"seq 점프 {self.last_seq}->{seq} ({step-1} 누락)"
            self.last_seq = seq

        if self.idx_ts is not None and self.idx_ts < len(fields):
            try:
                ts = int(float(fields[self.idx_ts]))
            except ValueError:
                self.field_errors += 1
                return False, "타임스탬프 파싱 실패"
            if self.last_timestamp is not None:
                gap = ts - self.last_timestamp
                tol = TIMESTAMP_INTERVAL_MS * TIMESTAMP_TOLERANCE
                if gap == 0:
                    self.timestamp_duplicates += 1
                    return False, f"타임스탬프 중복 {ts}ms"
                elif gap < 0:
                    self.timestamp_gaps += 1
                    return False, f"타임스탬프 역전 {self.last_timestamp}->{ts}"
                elif abs(gap - TIMESTAMP_INTERVAL_MS) > tol:
                    self.timestamp_gaps += 1
                    self.last_timestamp = ts
                    self.valid_lines += 1
                    return True, (warn + " | " if warn else "") + \
                        f"간격 이상 {gap}ms"
            self.last_timestamp = ts

        self.valid_lines += 1
        return True, warn

    def summary(self):
        return {
            "total": self.total_lines,
            "valid": self.valid_lines,
            "field_errors": self.field_errors,
            "timestamp_gaps": self.timestamp_gaps,
            "timestamp_duplicates": self.timestamp_duplicates,
            "seq_drops": self.seq_drops,
        }


# ──────────────────────────────────────────────
# 포트 선택
# ──────────────────────────────────────────────
def pick_port(explicit):
    """사용할 COM 포트를 결정한다.

    explicit이 주어지면 그대로 쓴다. 아니면 자동 탐색: 포트가 정확히
    1개면 그것을, 여러 개면 목록을 보여주고 명시 지정을 요청한다.
    """
    if explicit:
        return explicit

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[오류] COM 포트가 하나도 감지되지 않았습니다.")
        print("       HC-06 페어링 상태와 블루투스 연결을 확인하세요.")
        return None

    if len(ports) == 1:
        p = ports[0]
        print(f"  COM 포트 자동 선택: {p.device} ({p.description})")
        return p.device

    # 여러 개: 블루투스로 보이는 것을 추천하되, 명시 지정을 권한다.
    print("  COM 포트가 여러 개 감지되었습니다:")
    for p in ports:
        print(f"    {p.device} - {p.description}")
    bt = [p.device for p in ports
          if "bluetooth" in p.description.lower()]
    if bt:
        print(f"  추천: {bt[0]} (블루투스)")
    print("  명령에 포트를 직접 지정하세요. 예:  python logger_cli.py E1 1 COM6")
    return None


# ──────────────────────────────────────────────
# 로깅 본체
# ──────────────────────────────────────────────
class CliLogger:
    def __init__(self, scenario, run, port):
        self.scenario = scenario
        self.run = run
        self.port = port
        self.ser = None
        self.validator = DataValidator()
        self.filepath = None
        self._stop = False

    def _make_filepath(self):
        now = datetime.now()
        name = (f"{self.scenario}_run{self.run:02d}_"
                f"{now.strftime('%Y%m%d')}_{now.strftime('%H%M%S')}.csv")
        return OUTPUT_DIR / name

    def request_stop(self):
        """Ctrl+C 등으로 외부에서 종료를 요청."""
        self._stop = True

    def run_logging(self):
        # 1) 포트 열기
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0.1,
            )
        except serial.SerialException as e:
            print(f"[오류] 포트 열기 실패: {e}")
            print("       다른 프로그램(PuTTY 등)이 포트를 쓰고 있지 않은지 확인.")
            return False

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        self.filepath = self._make_filepath()

        print()
        print("=" * 60)
        print(f" 시나리오 {self.scenario}  run {self.run}")
        print(f" 포트 {self.port} @ {BAUD_RATE} baud")
        print(f" 저장 파일: {self.filepath}")
        print("=" * 60)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

        print()
        print("  >> 지금 MCU를 리셋하세요. 데이터 수신을 기다립니다...")
        print("     (굴림이 끝나고 데이터가 멈추면 자동 저장됩니다.")
        print("      수동 종료는 Ctrl+C)")
        print()

        # 2) 수신 루프
        ok = self._receive_to_file()

        # 3) 정리
        try:
            self.ser.close()
        except Exception:
            pass
        return ok

    def _receive_to_file(self):
        """시리얼에서 줄을 받아 CSV로 쓴다. 단일 스레드, 블로킹 없음.

        timeout=0.1로 열린 포트라 read()는 길어야 0.1초 만에 반환한다.
        따라서 이 루프는 멈추지 않으며, Ctrl+C도 즉시 반응한다.
        """
        buffer = ""
        header_done = False
        header_written = False
        csv_file = None
        csv_writer = None
        pre_header_data = []   # 헤더 전에 도착한 '데이터처럼 보이는' 줄 보관

        start_time = time.time()
        last_data_time = None
        first_data_seen = False
        warned_no_data = False
        printed_count = 0

        try:
            csv_file = open(self.filepath, "w", newline="", encoding="utf-8")
            csv_writer = csv.writer(csv_file)

            while not self._stop:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                now = time.time()

                if chunk:
                    buffer += chunk.decode("utf-8", errors="replace")
                    if not first_data_seen:
                        first_data_seen = True
                        print("  [수신 시작] 데이터가 들어오기 시작했습니다.")
                    last_data_time = now
                else:
                    # 데이터 없음: 종료 조건 검사
                    if first_data_seen and last_data_time is not None:
                        if (now - last_data_time) > IDLE_TIMEOUT_S:
                            print(f"\n  [종료] {IDLE_TIMEOUT_S:.0f}초간 "
                                  f"데이터 없음 - 측정 완료로 판단.")
                            break
                    else:
                        # 아직 첫 데이터 전: 너무 오래면 안내
                        if (not warned_no_data
                                and (now - start_time) > FIRST_DATA_WARN_S):
                            warned_no_data = True
                            print(f"  [안내] {FIRST_DATA_WARN_S:.0f}초간 데이터가 "
                                  f"없습니다. MCU 리셋을 했는지 확인하세요.")
                    continue

                # 버퍼를 줄 단위로 처리
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    # 헤더 줄
                    if line.startswith(HEADER_PREFIX):
                        cols = line[len(HEADER_PREFIX):].strip().split(",")
                        cols = [c.strip() for c in cols if c.strip()]
                        if cols and not header_done:
                            header_done = True
                            self.validator.set_header(cols)
                            csv_writer.writerow(cols)
                            csv_file.flush()
                            header_written = True
                        continue

                    # --- 헤더를 아직 못 받은 상태 ---
                    # 펌웨어는 부팅 시 '# CSV_HEADER:' 줄을 보낸 뒤에야
                    # 데이터를 보낸다. 따라서 헤더 전에 오는 모든 줄은
                    # 부팅 배너('====', 'Phase 6 - ...', 'KF: Q=...' 등)
                    # 이므로 데이터로 처리하지 않고 건너뛴다.
                    #
                    # 다만 헤더 줄이 유실됐을 가능성에 대비한 폴백이
                    # 필요하다. '데이터처럼 생긴 줄'(쉼표가 충분히 많고
                    # 첫 칸이 숫자)이 일정 개수 이상 연속으로 쌓이면,
                    # 그제서야 헤더 유실로 보고 폴백 헤더를 만든다.
                    if not header_done:
                        if _looks_like_data_row(line):
                            pre_header_data.append(line)
                            if len(pre_header_data) >= FALLBACK_TRIGGER:
                                # 헤더가 정말 안 온다 - 폴백
                                n = len(pre_header_data[0].split(","))
                                self.validator.set_header_fallback(n)
                                fb = [f"col{i}" for i in range(n)]
                                csv_writer.writerow(fb)
                                csv_file.flush()
                                header_written = True
                                header_done = True
                                # 모아둔 줄들을 마저 처리
                                for held in pre_header_data:
                                    hv, hmsg = self.validator.validate_line(
                                        held.split(","))
                                    if hv:
                                        csv_writer.writerow(held.split(","))
                                pre_header_data.clear()
                        # 배너 줄이거나 아직 폴백 임계 미달 - 건너뜀
                        continue

                    # 펌웨어 잡음 줄 (헤더 이후의 '#' 결과 배너 등)
                    if is_firmware_noise(line):
                        continue

                    # --- 데이터 줄 (헤더 확정 이후) ---
                    fields = line.split(",")
                    is_valid, msg = self.validator.validate_line(fields)
                    if is_valid:
                        csv_writer.writerow(fields)
                        v = self.validator.valid_lines
                        if v % 100 == 0:
                            csv_file.flush()
                        # 50줄마다 진행 표시 (한 줄 갱신)
                        if v - printed_count >= 50:
                            printed_count = v
                            print(f"\r  수신 중... 유효 {v}줄", end="", flush=True)
                    else:
                        # 경고는 줄바꿈해서 분리 출력
                        print(f"\n  [무효] line {self.validator.total_lines}: {msg}")
                    if msg and is_valid:
                        print(f"\n  [경고] {msg}")

        except KeyboardInterrupt:
            print("\n  [종료] Ctrl+C - 사용자 중단.")
        except serial.SerialException as e:
            print(f"\n[오류] 시리얼 수신 오류: {e}")
            return self._finalize(csv_file, header_written, aborted=True)
        except Exception as e:
            import traceback
            print(f"\n[오류] 예외: {e}")
            print(traceback.format_exc())
            return self._finalize(csv_file, header_written, aborted=True)

        return self._finalize(csv_file, header_written, aborted=False)

    def _finalize(self, csv_file, header_written, aborted):
        """파일을 닫고 요약을 출력한다."""
        if csv_file:
            try:
                csv_file.flush()
                csv_file.close()
            except Exception as e:
                print(f"  [경고] 파일 닫기 중 오류: {e}")

        s = self.validator.summary()
        print()
        print("-" * 60)
        print(f"  저장 완료: {self.filepath}")
        print(f"  수신 줄        : {s['total']}")
        print(f"  유효 줄        : {s['valid']}")
        print(f"  필드 수 오류   : {s['field_errors']}")
        print(f"  타임스탬프 이상: {s['timestamp_gaps']}")
        print(f"  seq 누락 추정  : {s['seq_drops']} 프레임")
        if s["total"] > 0:
            integ = 100.0 * s["valid"] / s["total"]
            print(f"  무결성         : {integ:.2f}%")
        print("-" * 60)

        if not header_written:
            print("  [주의] 헤더를 못 받아 파일이 비정상일 수 있습니다.")
            return False
        if s["valid"] == 0:
            print("  [주의] 유효 데이터가 0줄입니다. MCU 리셋/연결을 확인하세요.")
            return False
        return not aborted


# ──────────────────────────────────────────────
# 엔트리 포인트
# ──────────────────────────────────────────────
def main():
    args = sys.argv[1:]
    if len(args) < 2:
        print("사용법:  python logger_cli.py <scenario> <run> [port]")
        print("  예:    python logger_cli.py E1 1")
        print("         python logger_cli.py E1 2 COM6")
        print(f"  scenario : {' / '.join(SCENARIOS)}")
        return 1

    scenario = args[0].upper()
    if scenario not in SCENARIOS:
        print(f"[오류] 시나리오는 {SCENARIOS} 중 하나여야 합니다. 입력: {scenario}")
        return 1

    try:
        run = int(args[1])
    except ValueError:
        print(f"[오류] run 번호는 정수여야 합니다. 입력: {args[1]}")
        return 1

    explicit_port = args[2] if len(args) >= 3 else None
    port = pick_port(explicit_port)
    if not port:
        return 1

    logger = CliLogger(scenario, run, port)

    # Ctrl+C를 깔끔하게 처리: 플래그만 세팅하고 루프가 스스로 빠지게.
    def _sigint(signum, frame):
        logger.request_stop()
    signal.signal(signal.SIGINT, _sigint)

    ok = logger.run_logging()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
