#!/usr/bin/env python3
"""
PC Serial Logger for KF/AKF Robot Experiment
=============================================
Bluetooth 가상 COM 포트를 통해 MCU에서 전송하는 18컬럼 CSV 데이터를
실시간으로 수신하여 파일로 저장하는 GUI 기반 로깅 스크립트.

파일명 규칙: E{시나리오}_run{번호}_{날짜}_{시각}.csv
예: E1_run03_20260412_143022.csv

Requirements:
    pip install pyserial
"""

import os
import sys
import csv
import time
import threading
import logging
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial이 설치되어 있지 않습니다.")
    print("설치: pip install pyserial")
    sys.exit(1)

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

# ──────────────────────────────────────────────
# 상수 정의
# ──────────────────────────────────────────────
BAUD_RATE = 115200
SCENARIOS = ["E0", "E1", "E2", "E3", "E4", "E5"]
EXPECTED_FIELDS = 18
DEFAULT_OUTPUT_DIR = Path.home() / "KF_Experiment_Data"

# CSV 헤더 (MCU에서도 동일한 헤더를 전송)
CSV_HEADER = [
    "timestamp_ms",        # uint32, ms
    "tof_distance_mm",     # float, mm
    "tof_signal_rate",     # float, MCPS
    "tof_range_status",    # uint8
    "us_distance_mm",      # float, mm
    "encoder_distance_mm", # float, mm
    "encoder_speed_mms",   # float, mm/s
    "kf_estimate_mm",      # float, mm
    "tof_residual",        # float, mm
    "tof_residual_var",    # float, mm²
    "tof_residual_mean",   # float, mm
    "sensor_disagree",     # float, mm
    "tof_meas_rate",       # float, mm
    "gt_distance_mm",      # float, mm
    "R_label",             # float, mm²
    "kalman_gain",         # float
    "innovation_cov",      # float, mm²
    "scenario_id",         # string
]

# 타임스탬프 간격 허용 범위 (ms)
# 50Hz 로깅 = 20ms 간격, ±50% 허용
TIMESTAMP_INTERVAL_MS = 20
TIMESTAMP_TOLERANCE = 0.5  # 50%


# ──────────────────────────────────────────────
# 로깅 설정
# ──────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("SerialLogger")


# ──────────────────────────────────────────────
# 데이터 검증 클래스
# ──────────────────────────────────────────────
class DataValidator:
    """수신 데이터의 무결성을 실시간으로 검증한다."""

    def __init__(self):
        self.last_timestamp = None
        self.total_lines = 0
        self.valid_lines = 0
        self.field_errors = 0
        self.timestamp_gaps = 0
        self.timestamp_duplicates = 0

    def validate_line(self, fields: list[str]) -> tuple[bool, str]:
        """
        한 줄의 CSV 데이터를 검증한다.

        Returns:
            (is_valid, error_message)
        """
        self.total_lines += 1

        # 1. 필드 수 검증
        if len(fields) != EXPECTED_FIELDS:
            self.field_errors += 1
            return False, f"필드 수 불일치: {len(fields)}개 (기대: {EXPECTED_FIELDS})"

        # 2. 타임스탬프 파싱 및 연속성 검증
        try:
            ts = int(fields[0])
        except ValueError:
            self.field_errors += 1
            return False, f"타임스탬프 파싱 실패: '{fields[0]}'"

        if self.last_timestamp is not None:
            gap = ts - self.last_timestamp
            expected = TIMESTAMP_INTERVAL_MS
            tolerance = expected * TIMESTAMP_TOLERANCE

            if gap == 0:
                self.timestamp_duplicates += 1
                return False, f"타임스탬프 중복: {ts}ms"
            elif gap < 0:
                self.timestamp_gaps += 1
                return False, f"타임스탬프 역전: {self.last_timestamp}ms → {ts}ms"
            elif abs(gap - expected) > tolerance:
                self.timestamp_gaps += 1
                # 경고만 하고 유효로 처리 (MCU 타이밍 지터 허용)
                self.valid_lines += 1
                self.last_timestamp = ts
                return True, f"타임스탬프 간격 이상: {gap}ms (기대: {expected}±{tolerance:.0f}ms)"

        self.last_timestamp = ts
        self.valid_lines += 1
        return True, ""

    def get_stats(self) -> dict:
        return {
            "total": self.total_lines,
            "valid": self.valid_lines,
            "field_errors": self.field_errors,
            "timestamp_gaps": self.timestamp_gaps,
            "timestamp_duplicates": self.timestamp_duplicates,
        }


# ──────────────────────────────────────────────
# 시리얼 로거 클래스
# ──────────────────────────────────────────────
class SerialLogger:
    """시리얼 포트에서 데이터를 수신하여 CSV 파일로 저장한다."""

    def __init__(self):
        self.ser = None
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False
        self.validator = DataValidator()
        self.filepath = None
        self._lock = threading.Lock()
        self._thread = None
        self.header_received = False

    def generate_filename(self, scenario: str, run_number: int, output_dir: Path) -> Path:
        """파일명을 자동 생성한다."""
        now = datetime.now()
        date_str = now.strftime("%Y%m%d")
        time_str = now.strftime("%H%M%S")
        filename = f"{scenario}_run{run_number:02d}_{date_str}_{time_str}.csv"
        return output_dir / filename

    def connect(self, port: str) -> bool:
        """시리얼 포트에 연결한다."""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 100ms 읽기 타임아웃
            )
            # 버퍼 클리어 (Bluetooth 연결 시 잔여 데이터 제거)
            self.ser.reset_input_buffer()
            logger.info(f"연결 완료: {port} @ {BAUD_RATE} baud")
            return True
        except serial.SerialException as e:
            logger.error(f"연결 실패: {e}")
            return False

    def disconnect(self):
        """시리얼 포트 연결을 해제한다."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("시리얼 포트 연결 해제")

    def start_logging(self, scenario: str, run_number: int, output_dir: Path) -> bool:
        """로깅을 시작한다."""
        if not self.ser or not self.ser.is_open:
            logger.error("시리얼 포트가 연결되어 있지 않습니다.")
            return False

        # 출력 디렉토리 생성
        output_dir.mkdir(parents=True, exist_ok=True)

        # 파일 생성
        self.filepath = self.generate_filename(scenario, run_number, output_dir)
        try:
            self.csv_file = open(self.filepath, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            # PC 측에서 헤더 먼저 기록
            self.csv_writer.writerow(CSV_HEADER)
            self.csv_file.flush()
        except IOError as e:
            logger.error(f"파일 생성 실패: {e}")
            return False

        # 상태 초기화
        self.validator = DataValidator()
        self.header_received = False
        self.is_logging = True

        # 수신 버퍼 클리어
        self.ser.reset_input_buffer()

        # 수신 스레드 시작
        self._thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._thread.start()

        logger.info(f"로깅 시작: {self.filepath.name}")
        return True

    def stop_logging(self) -> dict:
        """로깅을 중지하고 파일을 안전하게 닫는다."""
        self.is_logging = False

        # 스레드 종료 대기
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

        # 파일 안전 종료
        stats = self.validator.get_stats()
        if self.csv_file:
            try:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.csv_file.close()
                logger.info(f"파일 저장 완료: {self.filepath}")
            except Exception as e:
                logger.error(f"파일 종료 중 오류: {e}")
            finally:
                self.csv_file = None
                self.csv_writer = None

        return stats

    def _receive_loop(self):
        """시리얼 데이터 수신 루프 (별도 스레드에서 실행)."""
        buffer = ""

        while self.is_logging:
            try:
                if self.ser.in_waiting > 0:
                    raw = self.ser.read(self.ser.in_waiting)
                    try:
                        buffer += raw.decode("utf-8", errors="replace")
                    except Exception:
                        continue

                    # 줄 단위 처리
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if not line:
                            continue

                        # MCU 헤더 행 감지 (timestamp_ms로 시작)
                        if line.startswith("timestamp_ms"):
                            self.header_received = True
                            logger.info("MCU 헤더 수신 확인")
                            continue  # PC가 이미 헤더를 기록했으므로 스킵

                        # 데이터 파싱
                        fields = line.split(",")

                        # 검증
                        is_valid, msg = self.validator.validate_line(fields)

                        if is_valid:
                            with self._lock:
                                if self.csv_writer:
                                    self.csv_writer.writerow(fields)
                                    # 100줄마다 flush (실시간성과 I/O 효율 균형)
                                    if self.validator.valid_lines % 100 == 0:
                                        self.csv_file.flush()

                            if msg:  # 경고 메시지 (유효하지만 이상 감지)
                                logger.warning(msg)
                        else:
                            logger.warning(f"무효 데이터 (line {self.validator.total_lines}): {msg}")

                else:
                    time.sleep(0.001)  # 1ms 대기 (CPU 사용률 절감)

            except serial.SerialException as e:
                logger.error(f"시리얼 수신 오류: {e}")
                self.is_logging = False
                break
            except Exception as e:
                logger.error(f"예외 발생: {e}")
                continue


# ──────────────────────────────────────────────
# GUI 애플리케이션
# ──────────────────────────────────────────────
class LoggerApp:
    """tkinter 기반 로깅 GUI."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("KF/AKF Serial Logger")
        self.root.resizable(False, False)

        self.logger_core = SerialLogger()
        self.is_connected = False
        self.is_logging = False
        self.status_update_id = None

        self._build_ui()
        self._refresh_ports()

        # 창 닫기 이벤트 처리
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        """UI를 구성한다."""
        # 메인 프레임
        main = ttk.Frame(self.root, padding=15)
        main.grid(row=0, column=0, sticky="nsew")

        # ── 연결 설정 ──
        conn_frame = ttk.LabelFrame(main, text="연결 설정", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        ttk.Label(conn_frame, text="COM 포트:").grid(row=0, column=0, sticky="w", padx=(0, 5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(0, 5))

        self.refresh_btn = ttk.Button(conn_frame, text="새로고침", command=self._refresh_ports, width=8)
        self.refresh_btn.grid(row=0, column=2, padx=(0, 5))

        self.connect_btn = ttk.Button(conn_frame, text="연결", command=self._toggle_connect, width=8)
        self.connect_btn.grid(row=0, column=3)

        # ── 실험 설정 ──
        exp_frame = ttk.LabelFrame(main, text="실험 설정", padding=10)
        exp_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 10))

        ttk.Label(exp_frame, text="시나리오:").grid(row=0, column=0, sticky="w", padx=(0, 5))
        self.scenario_var = tk.StringVar(value="E1")
        self.scenario_combo = ttk.Combobox(
            exp_frame, textvariable=self.scenario_var,
            values=SCENARIOS, width=8, state="readonly"
        )
        self.scenario_combo.grid(row=0, column=1, sticky="w", padx=(0, 15))

        ttk.Label(exp_frame, text="Run 번호:").grid(row=0, column=2, sticky="w", padx=(0, 5))
        self.run_var = tk.IntVar(value=1)
        self.run_spin = ttk.Spinbox(
            exp_frame, textvariable=self.run_var,
            from_=1, to=99, width=5
        )
        self.run_spin.grid(row=0, column=3, sticky="w")

        ttk.Label(exp_frame, text="저장 폴더:").grid(row=1, column=0, sticky="w", padx=(0, 5), pady=(8, 0))
        self.dir_var = tk.StringVar(value=str(DEFAULT_OUTPUT_DIR))
        dir_entry = ttk.Entry(exp_frame, textvariable=self.dir_var, width=30)
        dir_entry.grid(row=1, column=1, columnspan=2, sticky="ew", pady=(8, 0))

        self.dir_btn = ttk.Button(exp_frame, text="찾아보기", command=self._browse_dir, width=8)
        self.dir_btn.grid(row=1, column=3, padx=(5, 0), pady=(8, 0))

        # ── 시작/종료 버튼 ──
        btn_frame = ttk.Frame(main)
        btn_frame.grid(row=2, column=0, columnspan=2, pady=(0, 10))

        self.start_btn = ttk.Button(
            btn_frame, text="▶  로깅 시작", command=self._start_logging, width=18
        )
        self.start_btn.grid(row=0, column=0, padx=(0, 10))
        self.start_btn.state(["disabled"])

        self.stop_btn = ttk.Button(
            btn_frame, text="■  로깅 종료", command=self._stop_logging, width=18
        )
        self.stop_btn.grid(row=0, column=1)
        self.stop_btn.state(["disabled"])

        # ── 상태 표시 ──
        status_frame = ttk.LabelFrame(main, text="상태", padding=10)
        status_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(0, 5))

        self.status_label = ttk.Label(status_frame, text="대기 중", foreground="gray")
        self.status_label.grid(row=0, column=0, sticky="w")

        stats_grid = ttk.Frame(status_frame)
        stats_grid.grid(row=1, column=0, sticky="ew", pady=(5, 0))

        ttk.Label(stats_grid, text="수신 행:").grid(row=0, column=0, sticky="w")
        self.total_label = ttk.Label(stats_grid, text="0")
        self.total_label.grid(row=0, column=1, sticky="w", padx=(5, 20))

        ttk.Label(stats_grid, text="유효:").grid(row=0, column=2, sticky="w")
        self.valid_label = ttk.Label(stats_grid, text="0")
        self.valid_label.grid(row=0, column=3, sticky="w", padx=(5, 20))

        ttk.Label(stats_grid, text="오류:").grid(row=0, column=4, sticky="w")
        self.error_label = ttk.Label(stats_grid, text="0", foreground="red")
        self.error_label.grid(row=0, column=5, sticky="w", padx=(5, 0))

        self.file_label = ttk.Label(status_frame, text="", foreground="blue")
        self.file_label.grid(row=2, column=0, sticky="w", pady=(5, 0))

        # ── 로그 출력 ──
        log_frame = ttk.LabelFrame(main, text="로그", padding=5)
        log_frame.grid(row=4, column=0, columnspan=2, sticky="nsew")

        self.log_text = tk.Text(log_frame, height=8, width=60, state="disabled", font=("Consolas", 9))
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        log_frame.grid_columnconfigure(0, weight=1)
        log_frame.grid_rowconfigure(0, weight=1)

        # 로그 핸들러 연결
        self._setup_log_handler()

    def _setup_log_handler(self):
        """로거 출력을 GUI 텍스트 위젯에 연결한다."""

        class TextHandler(logging.Handler):
            def __init__(self, text_widget, root):
                super().__init__()
                self.text_widget = text_widget
                self.root = root

            def emit(self, record):
                msg = self.format(record) + "\n"
                try:
                    self.root.after(0, self._append, msg)
                except Exception:
                    pass

            def _append(self, msg):
                self.text_widget.configure(state="normal")
                self.text_widget.insert("end", msg)
                self.text_widget.see("end")
                # 최대 500줄 유지
                line_count = int(self.text_widget.index("end-1c").split(".")[0])
                if line_count > 500:
                    self.text_widget.delete("1.0", f"{line_count - 500}.0")
                self.text_widget.configure(state="disabled")

        handler = TextHandler(self.log_text, self.root)
        handler.setFormatter(logging.Formatter("%(asctime)s %(message)s", datefmt="%H:%M:%S"))
        logger.addHandler(handler)

    def _refresh_ports(self):
        """사용 가능한 COM 포트 목록을 갱신한다."""
        ports = serial.tools.list_ports.comports()
        port_list = []
        for p in ports:
            # Bluetooth 포트 우선 표시
            desc = f"{p.device} - {p.description}"
            port_list.append(desc)

        self.port_combo["values"] = port_list
        if port_list:
            # Bluetooth 관련 포트를 자동 선택
            bt_idx = 0
            for i, desc in enumerate(port_list):
                if "bluetooth" in desc.lower() or "bt" in desc.lower():
                    bt_idx = i
                    break
            self.port_combo.current(bt_idx)
        logger.info(f"COM 포트 {len(port_list)}개 감지")

    def _toggle_connect(self):
        """연결/해제를 토글한다."""
        if not self.is_connected:
            port_desc = self.port_var.get()
            if not port_desc:
                messagebox.showwarning("경고", "COM 포트를 선택해주세요.")
                return
            port = port_desc.split(" - ")[0]  # "COM3 - Bluetooth" → "COM3"

            if self.logger_core.connect(port):
                self.is_connected = True
                self.connect_btn.config(text="연결 해제")
                self.start_btn.state(["!disabled"])
                self.port_combo.state(["disabled"])
                self.refresh_btn.state(["disabled"])
                self.status_label.config(text=f"연결됨: {port}", foreground="green")
        else:
            if self.is_logging:
                self._stop_logging()
            self.logger_core.disconnect()
            self.is_connected = False
            self.connect_btn.config(text="연결")
            self.start_btn.state(["disabled"])
            self.port_combo.state(["!disabled"])
            self.refresh_btn.state(["!disabled"])
            self.status_label.config(text="연결 해제됨", foreground="gray")

    def _browse_dir(self):
        """저장 폴더를 선택한다."""
        d = filedialog.askdirectory(initialdir=self.dir_var.get())
        if d:
            self.dir_var.set(d)

    def _start_logging(self):
        """로깅을 시작한다."""
        scenario = self.scenario_var.get()
        run_num = self.run_var.get()
        output_dir = Path(self.dir_var.get())

        if self.logger_core.start_logging(scenario, run_num, output_dir):
            self.is_logging = True
            self.start_btn.state(["disabled"])
            self.stop_btn.state(["!disabled"])
            self.scenario_combo.state(["disabled"])
            self.run_spin.state(["disabled"])
            self.dir_btn.state(["disabled"])

            self.file_label.config(text=f"파일: {self.logger_core.filepath.name}")
            self.status_label.config(text="● 로깅 중...", foreground="red")

            # 상태 업데이트 시작
            self._update_stats()

    def _stop_logging(self):
        """로깅을 종료한다."""
        stats = self.logger_core.stop_logging()

        self.is_logging = False
        self.start_btn.state(["!disabled"])
        self.stop_btn.state(["disabled"])
        self.scenario_combo.state(["!disabled"])
        self.run_spin.state(["!disabled"])
        self.dir_btn.state(["!disabled"])

        # 상태 업데이트 중지
        if self.status_update_id:
            self.root.after_cancel(self.status_update_id)
            self.status_update_id = None

        # 최종 통계 표시
        self._display_stats(stats)
        self.status_label.config(text="로깅 완료", foreground="blue")

        # run 번호 자동 증가
        self.run_var.set(self.run_var.get() + 1)

        # 완료 알림
        msg = (
            f"로깅 완료!\n\n"
            f"파일: {self.logger_core.filepath.name}\n"
            f"총 수신: {stats['total']}행\n"
            f"유효: {stats['valid']}행\n"
            f"필드 오류: {stats['field_errors']}건\n"
            f"타임스탬프 이상: {stats['timestamp_gaps']}건"
        )
        messagebox.showinfo("로깅 완료", msg)

    def _update_stats(self):
        """GUI 상태 표시를 주기적으로 업데이트한다."""
        if not self.is_logging:
            return

        stats = self.logger_core.validator.get_stats()
        self._display_stats(stats)

        # 500ms마다 업데이트
        self.status_update_id = self.root.after(500, self._update_stats)

    def _display_stats(self, stats: dict):
        """통계를 GUI에 표시한다."""
        self.total_label.config(text=str(stats["total"]))
        self.valid_label.config(text=str(stats["valid"]))
        total_errors = stats["field_errors"] + stats["timestamp_gaps"] + stats["timestamp_duplicates"]
        self.error_label.config(text=str(total_errors))

    def _on_close(self):
        """창 닫기 시 안전 종료 처리."""
        if self.is_logging:
            if messagebox.askyesno("확인", "로깅 중입니다. 종료하시겠습니까?\n(파일이 안전하게 저장됩니다)"):
                self._stop_logging()
                self.logger_core.disconnect()
                self.root.destroy()
        else:
            if self.is_connected:
                self.logger_core.disconnect()
            self.root.destroy()

    def run(self):
        """GUI 메인 루프를 시작한다."""
        logger.info("KF/AKF Serial Logger 시작")
        logger.info(f"기대 필드 수: {EXPECTED_FIELDS}, baud rate: {BAUD_RATE}")
        self.root.mainloop()


# ──────────────────────────────────────────────
# 엔트리 포인트
# ──────────────────────────────────────────────
if __name__ == "__main__":
    app = LoggerApp()
    app.run()
