"""Đo latency (CPU) của các bước kiểm tra calibration. Bỏ lần chạy đầu, báo p50/p95.

Chạy từ gốc repo:
    python -m src.latency_topic_a --runs 30
"""
from __future__ import annotations

import argparse
import platform
import subprocess
import time
from pathlib import Path

import numpy as np
import pandas as pd

from src.align_metrics import (KERNELS, box_hit_stats, edge_alignment_score, image_edge_distance,
                               object_point_sets)
from starter.datasets import load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image

CASES = [("kitti", "data/kitti_mini", "000011"), ("nuscenes", "data/nuscenes_mini_subset", "scene-0103_010")]


def cpu_name() -> str:
    try:
        out = subprocess.run(["wmic", "cpu", "get", "name"], capture_output=True, text=True, timeout=10).stdout
        names = [x.strip() for x in out.splitlines() if x.strip() and x.strip() != "Name"]
        if names:
            return names[0]
    except Exception:
        pass
    return platform.processor() or platform.machine()


def timeit(fn, runs: int) -> np.ndarray:
    fn()                                            # lần đầu (khởi tạo, cache): bỏ
    t = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        t.append((time.perf_counter() - t0) * 1000)
    return np.array(t)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=30, help="số lần đo (không tính lần đầu), tối thiểu 20")
    ap.add_argument("--out", default="results/latency_calib_check.csv")
    args = ap.parse_args()
    assert args.runs >= 20, "cần ít nhất 20 lần chạy"
    rows = []
    for name, root, frame in CASES:
        fr = load_frame(root, frame)
        shape = fr["image"].shape
        c = perturb_extrinsic(fr["calib"], yaw_deg=1.0)
        edge_dist = image_edge_distance(fr["image"])
        sets = object_point_sets(fr["points"], fr["calib"], fr["labels"], shape)
        steps = {
            "project_points": lambda: project_velo_to_image(fr["points"], c, shape),
            "image_edge_distance(Canny+DT)": lambda: image_edge_distance(fr["image"]),
            "edge_alignment_score": lambda: edge_alignment_score(fr["points"], c, edge_dist, shape, kernel=KERNELS[name]),
            "box_hit_stats(offline, can label)": lambda: box_hit_stats(sets, c, shape),
        }
        for step, fn in steps.items():
            t = timeit(fn, args.runs)
            rows.append({"dataset": name, "frame": frame, "step": step, "runs": args.runs,
                         "p50_ms": round(float(np.percentile(t, 50)), 2), "p95_ms": round(float(np.percentile(t, 95)), 2),
                         "cpu": cpu_name()})
    df = pd.DataFrame(rows)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.out, index=False)
    print(df.drop(columns="cpu").to_string(index=False))
    print("CPU:", cpu_name(), "->", args.out)


if __name__ == "__main__":
    main()
