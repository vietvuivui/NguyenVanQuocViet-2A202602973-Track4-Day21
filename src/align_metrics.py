"""Metric đo độ lệch calibration LiDAR-camera (topic A).

Hai nhóm metric:
1. Có GT (dùng label): % điểm của object còn rơi vào 2D box sau khi perturb calibration.
2. Không cần GT (tự giám sát): edge-alignment score, so cạnh độ sâu của LiDAR với cạnh ảnh (Canny).

Chỉ dùng numpy + OpenCV, không có thành phần ngẫu nhiên: chạy lại ra đúng cùng số.
"""
from __future__ import annotations

import cv2
import numpy as np

from starter.kitti_io import KittiCalib, KittiObject
from starter.projection import cam_to_image, velo_to_cam

DIST_BINS = [(0, 15, "near<15m"), (15, 30, "mid15-30m"), (30, 200, "far>30m")]


def points_in_box3d(points_cam: np.ndarray, obj: KittiObject, margin: float = 0.1) -> np.ndarray:
    """Mask (N,) điểm (camera frame) nằm trong 3D box GT. Ngược lại với `box3d_corners_cam`:
    đưa điểm về hệ toạ độ của box rồi so với nửa kích thước."""
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ R            # = R^T (p - loc) cho từng hàng
    return ((np.abs(local[:, 0]) <= l / 2 + margin) & (np.abs(local[:, 2]) <= w / 2 + margin)
            & (local[:, 1] <= margin) & (local[:, 1] >= -h - margin))


def object_point_sets(points: np.ndarray, calib: KittiCalib, labels: list[KittiObject],
                      image_shape: tuple[int, ...], min_points: int = 10) -> list[dict]:
    """Với calibration GỐC (chưa perturb): điểm LiDAR thuộc từng object, và chỉ giữ điểm mà phép chiếu gốc
    rơi vào 2D box của label. Nhờ vậy baseline = 100% theo định nghĩa, và lệch do box 2D amodal/bị cắt
    (truncation) không lẫn vào kết quả."""
    pts = points[np.isfinite(points[:, :3]).all(axis=1), :3]
    cam = velo_to_cam(pts, calib)
    out = []
    for obj in labels:
        if obj.location[2] <= 0:
            continue
        idx = np.flatnonzero(points_in_box3d(cam, obj))
        if len(idx) == 0:
            continue
        uv, _, m = cam_to_image(cam[idx], calib.P2, image_shape)
        x1, y1, x2, y2 = obj.bbox
        keep = (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
        sel = idx[m][keep]
        if len(sel) >= min_points:
            out.append({"type": obj.type, "dist": float(np.linalg.norm(obj.location[[0, 2]])),
                        "bbox": obj.bbox, "points_velo": pts[sel], "uv0": uv[keep]})
    return out


def box_hit_stats(obj_sets: list[dict], calib_perturbed: KittiCalib, image_shape: tuple[int, ...]) -> list[dict]:
    """Chiếu lại điểm của từng object bằng calibration đã perturb: số điểm còn trong 2D box + dịch chuyển pixel."""
    rows = []
    for o in obj_sets:
        cam = velo_to_cam(o["points_velo"], calib_perturbed)
        h, w = image_shape[:2]
        z = cam[:, 2]
        front = z > 0.1
        homo = np.hstack([cam, np.ones((len(cam), 1))])
        pr = (calib_perturbed.P2 @ homo.T).T
        s = np.where(front, pr[:, 2], 1.0)
        uv = np.stack([pr[:, 0] / s, pr[:, 1] / s], axis=1)
        x1, y1, x2, y2 = o["bbox"]
        hit = front & (uv[:, 0] >= x1) & (uv[:, 0] <= x2) & (uv[:, 1] >= y1) & (uv[:, 1] <= y2)
        shift = np.linalg.norm(uv - o["uv0"], axis=1)
        rows.append({"dist": o["dist"], "n": len(cam), "hits": int(hit.sum()),
                     "shift_sum": float(shift[front].sum()), "n_front": int(front.sum())})
    return rows


def dist_bin(d: float) -> str:
    for lo, hi, name in DIST_BINS:
        if lo <= d < hi:
            return name
    return DIST_BINS[-1][2]


# --------------------------------------------------------------------------- edge-alignment score
def image_edge_distance(image_bgr: np.ndarray) -> np.ndarray:
    """Khoảng cách (pixel) từ mỗi pixel tới cạnh Canny gần nhất. Tính 1 lần cho mỗi frame."""
    gray = cv2.GaussianBlur(cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    return cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 3)


def ground_free_mask(points: np.ndarray, clearance: float = 0.35) -> np.ndarray:
    """Bỏ điểm mặt đất: ước lượng độ cao mặt đất bằng percentile 10 của z trong bán kính 30 m."""
    xyz = points[:, :3]
    near = np.hypot(xyz[:, 0], xyz[:, 1]) < 30
    z_ground = np.percentile(xyz[near, 2], 10) if near.any() else xyz[:, 2].min()
    return xyz[:, 2] > z_ground + clearance


def depth_edge_points(uv: np.ndarray, depth: np.ndarray, image_shape: tuple[int, ...],
                      kernel: tuple[int, int] = (7, 7), min_jump_m: float = 2.0,
                      min_jump_rel: float = 0.15) -> np.ndarray:
    """Mask (M,) điểm nằm sát một bước nhảy độ sâu (biên vật thể): trong cửa sổ (cao, rộng) = kernel quanh
    điểm (trên ảnh), chênh lệch max-min của depth lớn hơn max(min_jump_m, min_jump_rel * depth)."""
    h, w = image_shape[:2]
    iu = np.clip(np.round(uv[:, 0]).astype(int), 0, w - 1)
    iv = np.clip(np.round(uv[:, 1]).astype(int), 0, h - 1)
    d_max = np.zeros((h, w), np.float32)
    d_min = np.full((h, w), 1e6, np.float32)
    d_max[iv, iu] = depth
    d_min[iv, iu] = depth
    k = np.ones(kernel, np.uint8)
    jump = cv2.dilate(d_max, k)[iv, iu] - cv2.erode(d_min, k)[iv, iu]
    return jump > np.maximum(min_jump_m, min_jump_rel * depth)


# Dịch bản đồ cạnh đi các offset CỐ ĐỊNH (px, trục u/v) để ước lượng mức "trúng cạnh do ngẫu nhiên".
CHANCE_SHIFTS = [(-60, 0), (60, 0), (-35, 25), (35, -25), (0, 40), (0, -40)]

# Cửa sổ tìm bước nhảy độ sâu (cao, rộng) theo mật độ beam của LiDAR: 64 beam dày, 32 beam thưa theo chiều dọc.
KERNELS = {"kitti": (7, 7), "nuscenes": (31, 15)}


def edge_alignment_score(points: np.ndarray, calib: KittiCalib, edge_dist: np.ndarray,
                         image_shape: tuple[int, ...], max_depth: float = 40.0,
                         kernel: tuple[int, int] = (7, 7), sigma_px: float = 2.5) -> tuple[float, int]:
    """Edge-alignment score (không cần label/GT). Với các điểm biên-độ-sâu của LiDAR chiếu bằng `calib`:
        aligned = mean exp(-d / sigma),  d = khoảng cách tới cạnh ảnh (Canny) gần nhất
        chance  = cùng công thức, nhưng bản đồ cạnh bị dịch đi các offset cố định (CHANCE_SHIFTS)
        score   = aligned - chance
    Trừ `chance` để loại ảnh nhiều cạnh (cây, toà nhà) làm điểm cao giả. score ~ 0: không hơn ngẫu nhiên.
    Trả về (score, số điểm biên dùng để tính); NaN nếu quá ít điểm biên."""
    valid = np.isfinite(points[:, :3]).all(axis=1)
    pts = points[valid]
    keep_ng = ground_free_mask(pts)
    cam = velo_to_cam(pts[:, :3], calib)
    uv, depth, m = cam_to_image(cam, calib.P2, image_shape)
    keep = keep_ng[m] & (depth < max_depth)
    uv, depth = uv[keep], depth[keep]
    if len(uv) < 50:
        return float("nan"), 0
    is_edge = depth_edge_points(uv, depth, image_shape, kernel=kernel)
    if is_edge.sum() < 20:
        return float("nan"), int(is_edge.sum())
    h, w = image_shape[:2]
    e = uv[is_edge]

    def sample(du: int, dv: int) -> float:
        iu = np.clip(np.round(e[:, 0] + du).astype(int), 0, w - 1)
        iv = np.clip(np.round(e[:, 1] + dv).astype(int), 0, h - 1)
        return float(np.exp(-edge_dist[iv, iu] / sigma_px).mean())

    aligned = sample(0, 0)
    chance = float(np.mean([sample(du, dv) for du, dv in CHANCE_SHIFTS]))
    return aligned - chance, int(is_edge.sum())
