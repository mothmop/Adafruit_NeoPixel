#!/usr/bin/env python3
"""
Sony α7C Focus Stacking Prototype - Host Tethered Controller
"""

import os
import csv
import sys
import time
import logging
import threading

CALIBRATION_FILE = "macro_calibration.csv"

DEFAULT_CALIBRATION_DATA = [
    [2.0, 10.0, 24.5],
    [3.0, 15.0, 18.2],
    [4.0, 20.0, 14.1],
    [6.0, 30.0, 9.4],
    [8.0, 40.0, 6.8],
    [10.0, 50.0, 5.1],
]

def ensure_calibration_file(file_path: str = CALIBRATION_FILE) -> None:
    if not os.path.exists(file_path):
        headers = ["Ruler_Distance_cm", "True_Depth_mm", "Steps_Per_mm"]
        with open(file_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(DEFAULT_CALIBRATION_DATA)

def load_calibration_table(file_path: str = CALIBRATION_FILE) -> list[tuple[float, float, float]]:
    ensure_calibration_file(file_path)
    table = []
    with open(file_path, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            table.append((
                float(row["Ruler_Distance_cm"]),
                float(row["True_Depth_mm"]),
                float(row["Steps_Per_mm"])
            ))
    table.sort(key=lambda x: x[0])
    return table

def interpolate_steps_per_mm(ruler_cm: float, table: list[tuple[float, float, float]]) -> float:
    if not table:
        return 10.0
    if ruler_cm <= table[0][0]:
        return table[0][2]
    if ruler_cm >= table[-1][0]:
        return table[-1][2]

    for i in range(len(table) - 1):
        x0, _, y0 = table[i]
        x1, _, y1 = table[i + 1]
        if x0 <= ruler_cm <= x1:
            t = (ruler_cm - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)
    return table[-1][2]

class ThreadSafeStackLogger:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, log_file: str = "stacking_debug.log"):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._init_logger(log_file)
            return cls._instance

    def _init_logger(self, log_file: str) -> None:
        self.logger = logging.getLogger("SonyFocusStack")
        self.logger.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            fmt="[%(asctime)s.%(msecs)03d] [%(threadName)s] [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        file_handler = logging.FileHandler(log_file, mode="a", encoding="utf-8")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def log_stack_step(self, frame_idx: int, total_frames: int, steps: int, target_depth_mm: float) -> None:
        with self._lock:
            self.logger.info(
                f"Frame {frame_idx + 1}/{total_frames} | Drive Steps: {steps} | "
                f"Relative Depth: {target_depth_mm:.3f} mm"
            )

    def log_interpolation(self, ruler_cm: float, depth_mm: float, steps_per_mm: float) -> None:
        with self._lock:
            self.logger.debug(
                f"Interpolation resolved: Ruler={ruler_cm:.2f}cm -> "
                f"TrueDepth={depth_mm:.2f}mm -> Rate={steps_per_mm:.3f} steps/mm"
            )

    def log_backlash_compensation(self, margin_steps: int) -> None:
        with self._lock:
            self.logger.info(
                f"Backlash pre-travel: Reversing {margin_steps} steps Far, then advancing {margin_steps} steps Near to seat gear teeth at Z=0."
            )

if __name__ == "__main__":
    logger = ThreadSafeStackLogger()
    table = load_calibration_table()
    test_ruler = 3.5
    rate = interpolate_steps_per_mm(test_ruler, table)
    true_z = test_ruler * 10.0 * 0.5
    logger.log_interpolation(test_ruler, true_z, rate)
    logger.log_backlash_compensation(50)
    print("Python prototype validation completed.")
