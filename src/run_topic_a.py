"""Topic A: LiDAR-camera projection QA. Chạy toàn bộ thí nghiệm của báo cáo.

    python -m src.run_topic_a demo        # Basic:    3 ảnh overlay ở 3 khoảng cách (KITTI)
    python -m src.run_topic_a sweep       # Good:     perturb calibration -> CSV (mọi frame của KITTI + nuScenes)
    python -m src.run_topic_a plots       # Good/Adv: bảng tổng hợp + biểu đồ từ CSV của `sweep`
    python -m src.run_topic_a failures    # ảnh fail_*.png
    python -m src.run_topic_a all         # chạy lần lượt cả 4 bước

Không có thành phần ngẫu nhiên (không random, không train): chạy lại ra đúng cùng số.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from src.align_metrics import (DIST_BINS, KERNELS, box_hit_stats, depth_edge_points, dist_bin,  # noqa: E402
                               edge_alignment_score, ground_free_mask, image_edge_distance,
                               object_point_sets)
from starter.datasets import dataset_type, list_frames, load_frame  # noqa: E402
from starter.projection import (cam_to_image, draw_box2d, overlay_points, perturb_extrinsic,  # noqa: E402
                                project_velo_to_image, velo_to_cam)

RESULTS = Path("results")
FIG = RESULTS / "figures"
DATASETS = {"kitti": "data/kitti_mini", "nuscenes": "data/nuscenes_mini_subset"}
ROT_LEVELS = [0.5, 1.0, 2.0, 3.0]          # độ
TRANS_LEVELS_CM = [2, 5, 10]                # cm
AXES = [("yaw", ROT_LEVELS), ("pitch", ROT_LEVELS), ("roll", ROT_LEVELS),
        ("tx", TRANS_LEVELS_CM), ("ty", TRANS_LEVELS_CM), ("tz", TRANS_LEVELS_CM)]
FA_QUANTILE = 0.05                          # ngưỡng phát hiện: 5% frame sạch bị báo nhầm


def perturbed(calib, axis: str, level: float):
    if axis in ("yaw", "pitch", "roll"):
        return perturb_extrinsic(calib, **{f"{axis}_deg": level})
    t = [0.0, 0.0, 0.0]
    t["xyz".index(axis[1])] = level / 100.0
    return perturb_extrinsic(calib, t_xyz_m=tuple(t))


def all_levels():
    yield "none", 0.0
    for axis, levels in AXES:
        for lv in levels:
            yield axis, lv


def load(root: str, frame: str, **kw):
    kwargs = kw if dataset_type(root) == "nuscenes" else {}
    return load_frame(root, frame, **kwargs)


# ------------------------------------------------------------------------------------------- demo
def stage_demo(_args) -> None:
    """3 ảnh overlay KITTI: vật gần (<10 m), vừa (~20-30 m), xa (>50 m). Vẽ 2D box + khoảng cách của vật chọn."""
    root = DATASETS["kitti"]
    picks = [("near", "000019"), ("mid", "000011"), ("far", "000004")]
    FIG.mkdir(parents=True, exist_ok=True)
    for tag, frame in picks:
        fr = load(root, frame)
        uv, depth, mask = project_velo_to_image(fr["points"], fr["calib"], fr["image"].shape)
        vis = overlay_points(fr["image"], uv, depth, radius=2)
        dists = []
        for o in fr["labels"]:
            d = float(np.linalg.norm(o.location[[0, 2]]))
            dists.append(d)
            vis = draw_box2d(vis, o.bbox, label=f"{o.type} {d:.0f}m")
        cv2.putText(vis, f"{frame}: vat gan nhat {min(dists):.1f}m, xa nhat {max(dists):.1f}m, "
                         f"{mask.mean():.1%} diem trong anh", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)
        cv2.imwrite(str(FIG / f"demo_overlay_{tag}_{frame}.png"), vis)
        print(f"demo {tag} {frame}: nearest={min(dists):.1f}m farthest={max(dists):.1f}m")


# ------------------------------------------------------------------------------------------ sweep
def sweep_dataset(name: str, root: str, use_ego_motion: bool = True) -> tuple[list[dict], list[dict]]:
    frame_rows, box_rows = [], []
    frames = list_frames(root)
    kernel = KERNELS[name]
    for i, f in enumerate(frames):
        kw = {"use_ego_motion": use_ego_motion} if name == "nuscenes" else {}
        fr = load(root, f, **kw)
        img, pts, calib0, labels = fr["image"], fr["points"], fr["calib"], fr["labels"]
        shape = img.shape
        finite = np.isfinite(pts[:, :3]).all(axis=1)
        edge_dist = image_edge_distance(img)
        obj_sets = object_point_sets(pts, calib0, labels, shape)
        for axis, lv in all_levels():
            c = calib0 if axis == "none" else perturbed(calib0, axis, lv)
            _, _, m = project_velo_to_image(pts[finite], c, shape)
            score, n_edge = edge_alignment_score(pts, c, edge_dist, shape, kernel=kernel)
            frame_rows.append({"dataset": name, "frame": f, "axis": axis, "level": lv,
                               "inside_fov_pct": 100 * m.mean(), "score": score, "n_edge": n_edge})
            agg: dict[str, dict] = {}
            for r in box_hit_stats(obj_sets, c, shape):
                a = agg.setdefault(dist_bin(r["dist"]), {"n": 0, "hits": 0, "shift_sum": 0.0, "n_front": 0, "objs": 0})
                a["n"] += r["n"]; a["hits"] += r["hits"]; a["shift_sum"] += r["shift_sum"]
                a["n_front"] += r["n_front"]; a["objs"] += 1
            for b, a in agg.items():
                box_rows.append({"dataset": name, "frame": f, "axis": axis, "level": lv, "dist_bin": b, **a})
        if (i + 1) % 20 == 0:
            print(f"  {name}: {i + 1}/{len(frames)} frames")
    return frame_rows, box_rows


def stage_sweep(_args) -> None:
    RESULTS.mkdir(exist_ok=True)
    fr_all, box_all = [], []
    for name, root in DATASETS.items():
        t0 = time.time()
        print(f"sweep {name} ({len(list_frames(root))} frames)")
        a, b = sweep_dataset(name, root)
        fr_all += a; box_all += b
        print(f"  xong trong {time.time() - t0:.0f}s")
    pd.DataFrame(fr_all).to_csv(RESULTS / "calib_sweep_frames.csv", index=False)
    pd.DataFrame(box_all).to_csv(RESULTS / "calib_sweep_boxes.csv", index=False)
    print("-> results/calib_sweep_frames.csv, results/calib_sweep_boxes.csv")


# ------------------------------------------------------------------------------------------ plots
def summarize() -> pd.DataFrame:
    fr = pd.read_csv(RESULTS / "calib_sweep_frames.csv")
    bx = pd.read_csv(RESULTS / "calib_sweep_boxes.csv")
    rows = []
    for ds, g in fr.groupby("dataset"):
        clean = g[g.axis == "none"].set_index("frame")["score"]
        thr = clean.dropna().quantile(FA_QUANTILE)
        for (axis, lv), h in g.groupby(["axis", "level"], sort=False):
            hv = h.set_index("frame")
            valid = hv["score"].notna()
            paired = (hv["score"] < clean.reindex(hv.index))[valid & clean.reindex(hv.index).notna()]
            row = {"dataset": ds, "axis": axis, "level": lv, "frames": len(h),
                   "inside_fov_pct": h.inside_fov_pct.mean(),
                   "score_mean": h.score.mean(), "n_edge_median": h.n_edge.median(),
                   "score_threshold": thr,
                   "detect_rate_pct": 100 * (hv["score"][valid] < thr).mean(),
                   "score_drop_vs_clean_pct": 100 * paired.mean()}
            b = bx[(bx.dataset == ds) & (bx.axis == axis) & (bx.level == lv)]
            for _, _, name in DIST_BINS:
                s = b[b.dist_bin == name]
                row[f"hit_pct_{name}"] = 100 * s.hits.sum() / s.n.sum() if s.n.sum() else np.nan
                row[f"shift_px_{name}"] = s.shift_sum.sum() / s.n_front.sum() if s.n_front.sum() else np.nan
                row[f"n_obj_{name}"] = int(s.objs.sum() / max(len(h), 1) * len(h)) if len(s) else 0
            rows.append(row)
    out = pd.DataFrame(rows)
    out.to_csv(RESULTS / "calib_sweep_summary.csv", index=False)
    return out


def stage_plots(_args) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    s = summarize()
    print("-> results/calib_sweep_summary.csv")
    bins = [n for *_, n in DIST_BINS]
    colors = {"near<15m": "#1b9e77", "mid15-30m": "#d95f02", "far>30m": "#7570b3"}

    # Hình 1: % điểm còn trong 2D box theo mức perturb, tách theo khoảng cách (hàng: yaw / ty)
    fig, axs = plt.subplots(2, 2, figsize=(10, 7), sharey=True)
    for j, ds in enumerate(["kitti", "nuscenes"]):
        for i, (axis, unit) in enumerate([("yaw", "deg"), ("ty", "cm")]):
            ax = axs[i, j]
            d = s[(s.dataset == ds) & (s.axis.isin([axis, "none"]))].sort_values("level")
            for b in bins:
                ax.plot(d.level, d[f"hit_pct_{b}"], marker="o", label=b, color=colors[b])
            ax.set_title(f"{ds}: {axis}"); ax.set_xlabel(f"{axis} ({unit})"); ax.grid(alpha=.3)
            if j == 0:
                ax.set_ylabel("% điểm object còn trong 2D box")
    axs[0, 0].legend(title="khoảng cách vật")
    fig.suptitle("Điểm LiDAR của object rơi đúng 2D box theo mức calibration drift")
    fig.tight_layout(); fig.savefig(FIG / "calib_box_hit_rate.png", dpi=130); plt.close(fig)

    # Hình 2: edge-alignment score theo mức perturb + ngưỡng phát hiện
    fr = pd.read_csv(RESULTS / "calib_sweep_frames.csv")
    fig, axs = plt.subplots(2, 2, figsize=(10, 7))
    for j, ds in enumerate(["kitti", "nuscenes"]):
        thr = s[s.dataset == ds].score_threshold.iloc[0]
        for i, (axis, unit) in enumerate([("yaw", "deg"), ("ty", "cm")]):
            ax = axs[i, j]
            g = fr[(fr.dataset == ds) & (fr.axis.isin([axis, "none"]))]
            piv = g.pivot_table(index="level", columns="frame", values="score")
            ax.plot(piv.index, piv.values, color="gray", alpha=.25, lw=.8)
            ax.errorbar(piv.index, piv.mean(axis=1), yerr=piv.std(axis=1), color="C3", marker="o", lw=2, label="trung bình ± std")
            ax.axhline(thr, color="k", ls="--", label=f"ngưỡng ({FA_QUANTILE:.0%} FA) = {thr:.3f}")
            ax.set_title(f"{ds}: {axis}"); ax.set_xlabel(f"{axis} ({unit})"); ax.grid(alpha=.3)
            ax.set_ylabel("edge-alignment score")
            if i == 0:
                ax.legend(fontsize=8)
    fig.suptitle("Edge-alignment score (mỗi đường xám = 1 frame)")
    fig.tight_layout(); fig.savefig(FIG / "calib_alignment_score.png", dpi=130); plt.close(fig)

    # In bảng tóm tắt cho báo cáo
    cols = ["axis", "level", "inside_fov_pct"] + [f"hit_pct_{b}" for b in bins] + \
           [f"shift_px_{bins[1]}", "score_mean", "detect_rate_pct"]
    pd.set_option("display.width", 220); pd.set_option("display.max_columns", 30)
    for ds in ["kitti", "nuscenes"]:
        print(f"\n== {ds} ==")
        print(s[s.dataset == ds][cols].round(2).to_string(index=False))


# ---------------------------------------------------------------------------------------- failures
def edge_vis(fr, calib, edge_dist, kernel, title: str) -> np.ndarray:
    """Ảnh xám + điểm biên-độ-sâu: xanh lá nếu cách cạnh ảnh <= 3 px, đỏ nếu xa hơn."""
    img = fr["image"]
    pts = fr["points"][np.isfinite(fr["points"][:, :3]).all(axis=1)]
    keep_ng = ground_free_mask(pts)
    uv, depth, m = cam_to_image(velo_to_cam(pts[:, :3], calib), calib.P2, img.shape)
    keep = keep_ng[m] & (depth < 40)
    uv, depth = uv[keep], depth[keep]
    is_edge = depth_edge_points(uv, depth, img.shape, kernel=kernel)
    base = cv2.cvtColor(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
    base = (base * 0.6).astype(np.uint8)
    h, w = img.shape[:2]
    for u, v in uv[is_edge]:
        d = edge_dist[min(int(round(v)), h - 1), min(int(round(u)), w - 1)]
        cv2.circle(base, (int(u), int(v)), 3, (0, 255, 0) if d <= 3 else (0, 0, 255), -1)
    cv2.putText(base, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return base


def stage_failures(_args) -> None:
    FIG.mkdir(parents=True, exist_ok=True)
    s = pd.read_csv(RESULTS / "calib_sweep_summary.csv")
    fr_csv = pd.read_csv(RESULTS / "calib_sweep_frames.csv")

    # fail_01: score KHÔNG phát hiện được drift. Chọn frame KITTI có score ở yaw 3 deg vẫn >= ngưỡng,
    # ưu tiên frame có nhiều điểm biên (không phải do thiếu dữ liệu).
    ds, root = "kitti", DATASETS["kitti"]
    thr = s[s.dataset == ds].score_threshold.iloc[0]
    y3 = fr_csv[(fr_csv.dataset == ds) & (fr_csv.axis == "yaw") & (fr_csv.level == 3.0)]
    miss = y3[y3.score >= thr].sort_values("n_edge", ascending=False)
    print(f"KITTI: {len(miss)}/{len(y3)} frame không bị phát hiện ở yaw 3 deg (score >= {thr:.3f})")
    if len(miss):
        f = miss.iloc[0]["frame"]
        fr = load(root, f)
        ed = image_edge_distance(fr["image"])
        k = KERNELS[ds]
        c0 = fr["calib"]; c3 = perturbed(c0, "yaw", 3.0)
        s0, n0 = edge_alignment_score(fr["points"], c0, ed, fr["image"].shape, kernel=k)
        s3, n3 = edge_alignment_score(fr["points"], c3, ed, fr["image"].shape, kernel=k)
        a = edge_vis(fr, c0, ed, k, f"{f} calib dung: score={s0:.3f} (n_edge={n0})")
        b = edge_vis(fr, c3, ed, k, f"{f} yaw +3 deg: score={s3:.3f} (n_edge={n3}) - van >= nguong {thr:.3f}")
        cv2.imwrite(str(FIG / f"fail_01_score_miss_yaw3_{f}.png"), np.vstack([a, b]))
        print(f"fail_01: frame {f}: {s0:.3f} -> {s3:.3f}")

    # fail_02: lệch THỜI GIAN (nuScenes bỏ bù ego-motion) giống hệt calibration drift với score & box-hit.
    root = DATASETS["nuscenes"]
    ids = list_frames(root)
    rows = []
    for f in ids:
        for ego in (True, False):
            fr = load(root, f, use_ego_motion=ego)
            sh = fr["image"].shape
            ed = image_edge_distance(fr["image"])
            sc, ne = edge_alignment_score(fr["points"], fr["calib"], ed, sh, kernel=KERNELS["nuscenes"])
            os_ = object_point_sets(fr["points"], fr["calib"], fr["labels"], sh)
            rows.append({"frame": f, "ego_motion": ego, "score": sc, "n_edge": ne,
                         "dt_ms": (fr["timestamp_camera_us"] - fr["timestamp_lidar_us"]) / 1000,
                         "n_objs": len(os_)})
    ego = pd.DataFrame(rows)
    ego.to_csv(RESULTS / "nuscenes_ego_motion_ablation.csv", index=False)
    piv = ego.pivot(index="frame", columns="ego_motion", values="score")
    d = (piv[False] - piv[True]).dropna()
    print(f"nuScenes ego-motion: n={len(d)}, mean score change when comp OFF = {d.mean():+.4f}")
    print(ego.groupby("ego_motion")[["score", "n_edge"]].mean())
    dts = ego[ego.ego_motion].set_index("frame")["dt_ms"]
    print(f"dt camera-lidar: mean {dts.mean():.1f} ms, max {dts.max():.1f} ms")


def stage_failures_fov(_args) -> None:
    """fail_02: '% điểm nằm trong ảnh' TĂNG khi calibration xấu đi, nên không dùng được làm chỉ số sức khoẻ calibration.
    nuScenes: trục x của LiDAR hướng sang phải, nên `roll` của perturb_extrinsic (quanh x) chính là camera pitch."""
    root, f = DATASETS["nuscenes"], "scene-0103_010"
    fr = load(root, f)
    panels = []
    for lv in (0.0, 3.0):
        c = perturb_extrinsic(fr["calib"], roll_deg=lv)
        uv, depth, mask = project_velo_to_image(fr["points"], c, fr["image"].shape)
        vis = overlay_points(fr["image"], uv, depth, radius=3)
        for o in fr["labels"]:
            vis = draw_box2d(vis, o.bbox)
        cv2.rectangle(vis, (0, 0), (1000, 48), (0, 0, 0), -1)
        cv2.putText(vis, f"{f} roll(x-LiDAR) {lv:g} deg: {mask.mean():.2%} diem trong anh ({int(mask.sum())})",
                    (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        panels.append(vis)
    cv2.imwrite(str(FIG / f"fail_02_fov_up_when_miscalibrated_{f}.png"), np.vstack(panels))
    print("fail_02 saved")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["demo", "sweep", "plots", "failures", "all"])
    args = ap.parse_args()
    stages = {"demo": stage_demo, "sweep": stage_sweep, "plots": stage_plots,
              "failures": lambda a: (stage_failures(a), stage_failures_fov(a))}
    for name in (stages if args.stage == "all" else [args.stage]):
        stages[name](args)


if __name__ == "__main__":
    main()
