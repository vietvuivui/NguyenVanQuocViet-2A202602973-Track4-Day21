# Báo cáo Day 6: LiDAR-camera projection QA, calibration drift bao nhiêu thì phát hiện được?

- **Họ tên:** Nguyễn Văn Quốc Việt
- **MSSV:** 2A202602973
- **Lớp:** AI20K-T4
- **Link repo:** https://github.com/vietvuivui/NguyenVanQuocViet-2A202602973-Track4-Day21
- **Topic:** A — LiDAR-camera projection QA (mức Basic + Good + Advanced)
- **Dataset:** data/kitti_mini, data/nuscenes_mini_subset (và data/synthetic để debug)
- **Các frame đã dùng:** toàn bộ 20 frame KITTI (000001 … 000061) và 80 frame nuScenes (scene-0103_000 … scene-1094_039). Ảnh demo: 000019, 000011, 000004 (KITTI), scene-0103_010 (nuScenes)

## 1. Claim

*Tóm tắt thí nghiệm: đo % điểm LiDAR của object còn trong 2D box, trên 20 frame KITTI và 80 frame nuScenes, với các mức yaw/pitch/roll 0.5–3° và tịnh tiến 2–10 cm, chia theo khoảng cách vật (<15 m, 15–30 m, >30 m).*

Lệch **yaw 1°** làm điểm LiDAR của vật **xa hơn 30 m** mất 29% khả năng rơi vào 2D box (còn 71%), trong khi vật gần hơn 15 m chỉ mất 4% (KITTI), còn lệch tịnh tiến đến **10 cm** gần như không đổi gì (≥ 97% điểm vẫn trong box). Một edge-alignment score tự giám sát (không cần label) có phát hiện được drift, nhưng **chỉ ở mức yếu**: với ngưỡng cho phép 5% báo nhầm, nó phát hiện 10% frame ở yaw 1° và 40% frame ở yaw 3° trên KITTI, và gần như không phát hiện được gì trên nuScenes (LiDAR 32 beam).

## 2. Evidence

Mọi số liệu dưới đây được sinh bởi `python -m src.run_topic_a all` (xem mục 5), không có phần ngẫu nhiên. Chạy lại 2 lần cho cùng hash CSV. `box hit` là % điểm LiDAR thuộc object (nằm trong 3D box GT và chiếu gốc vào 2D box) vẫn nằm trong 2D box sau khi perturb, nên baseline = 100% theo định nghĩa. KITTI: 108 object (36 xa, 40 vừa, 32 gần). nuScenes: 300 object (35 xa, 180 vừa, 85 gần). Mỗi lần perturb chỉ đổi **một** trục, giữ nguyên các trục khác.

**Bảng 1 — KITTI (20 frame), % điểm còn trong 2D box theo khoảng cách vật** (`results/calib_sweep_summary.csv`)

| Perturb | % điểm trong ảnh | box hit gần <15 m | box hit vừa 15–30 m | box hit xa >30 m | dịch pixel (vừa) | score (TB) | % frame phát hiện (FA 5%) |
|---|---|---|---|---|---|---|---|
| không (baseline) | 15.75 | 100 | 100 | 100 | 0 | 0.119 | 5.0 |
| yaw 0.5° | 15.76 | 98.2 | 95.9 | 88.9 | 7.5 | 0.101 | 5.0 |
| yaw 1° | 15.76 | 95.6 | 88.3 | 71.1 | 14.9 | 0.076 | 10.0 |
| yaw 2° | 15.77 | 88.6 | 75.7 | 43.3 | 29.8 | 0.047 | 20.0 |
| yaw 3° | 15.77 | 81.8 | 63.9 | 25.1 | 44.7 | 0.033 | 40.0 |
| pitch 1° | 15.02 | 90.6 | 85.8 | 62.6 | 12.8 | 0.050 | 25.0 |
| pitch 3° | 13.50 | 69.5 | 52.7 | 14.3 | 38.5 | 0.003 | 55.0 |
| roll 3° | 15.79 | 92.5 | 87.5 | 87.5 | 13.4 | 0.038 | 35.0 |
| tịnh tiến y 2 cm | 15.76 | 99.7 | 99.8 | 99.8 | 0.8 | 0.122 | 10.0 |
| tịnh tiến y 5 cm | 15.76 | 99.1 | 99.4 | 99.4 | 1.9 | 0.123 | 5.0 |
| tịnh tiến y 10 cm | 15.76 | 98.0 | 98.5 | 98.8 | 3.8 | 0.111 | 5.0 |
| tịnh tiến z 10 cm | 16.48 | 97.4 | 98.6 | 97.1 | 3.8 | 0.087 | 0.0 |

Đủ 6 trục (yaw, pitch, roll, tx, ty, tz) × các mức 0.5/1/2/3° và 2/5/10 cm có trong CSV.

**Bảng 2 — nuScenes (80 frame, LiDAR 32 beam, ảnh 1600×900)**

| Perturb | box hit gần | box hit vừa | box hit xa | dịch pixel (vừa) | % frame phát hiện (FA 5.1%) | số điểm biên TB (median) |
|---|---|---|---|---|---|---|
| không | 100 | 100 | 100 | 0 | 5.1 | 81 |
| yaw 1° | 95.6 | 94.4 | 95.0 | 26.0 | 10.1 | 81 |
| yaw 3° | 78.9 | 70.8 | 63.9 | 78.4 | 15.2 | 75 |
| trục x LiDAR 3° (= camera pitch) | 86.6 | 64.4 | 29.0 | 68.2 | 5.1 | — |
| tịnh tiến 10 cm (y) | 99.9 | 100 | 100 | 2.5 | 6.3 | — |

Nhận xét cho B5 (so sánh hai dataset): cùng 1° yaw, nuScenes dịch **26 px**, KITTI dịch **15 px**, vì tiêu cự nuScenes lớn hơn (ảnh 1600 px so với 1242 px), nhưng box hit của nuScenes lại giảm **chậm hơn** ở vật xa (95% so với 71%). Tôi **chưa kiểm chứng** nguyên nhân; giả thuyết hợp lý là 2D box nuScenes được sinh từ phép chiếu 3D box nên rộng hơn box KITTI vẽ tay. Ngoài ra trục `pitch`/`roll` của nuScenes bị hoán đổi so với KITTI, vì trục x của LiDAR nuScenes hướng sang phải (xem `data/README.md`): "roll" quanh x chính là camera pitch (22 px/° ở khoảng cách vừa, tương đương yaw), còn "pitch" quanh y (hướng trước) chỉ xoay ảnh quanh tâm. Score trên nuScenes không dùng được: median chỉ 81.5 điểm biên mỗi frame (KITTI 913.5), nên phát hiện chỉ 10–15%, sát mức báo nhầm 5%.

![demo gần](../results/figures/demo_overlay_near_000019.png)
![demo vừa](../results/figures/demo_overlay_mid_000011.png)
![demo xa](../results/figures/demo_overlay_far_000004.png)
![box hit](../results/figures/calib_box_hit_rate.png)
![score](../results/figures/calib_alignment_score.png)

Ba ảnh demo là overlay chưa perturb ở ba khoảng cách (vật gần nhất 6.3 m, 6.6 m–34 m, và 41–54 m). Điểm LiDAR khớp xe, người, cột và mặt đường, không có điểm trên bầu trời. Chi tiết score: `score = aligned − chance`, với `aligned` là trung bình `exp(−d/2.5px)` của điểm biên-độ-sâu tới cạnh Canny gần nhất, `chance` là cùng đại lượng khi bản đồ cạnh bị dịch 6 offset cố định. Tôi trừ `chance` vì không trừ thì score phẳng (0.54 → 0.53 khi yaw 0 → 3°), do ảnh nhiều cạnh nên điểm lệch vẫn "trúng" cạnh ngẫu nhiên. Ngưỡng phát hiện = percentile 5 của score các frame sạch (KITTI 0.007, nuScenes −0.037).

So từng frame với chính nó khi sạch thì tín hiệu có thật: score giảm ở 85% frame KITTI khi yaw 1° và 100% khi yaw 2°. Điểm yếu là score tuyệt đối khác nhau giữa các cảnh nhiều hơn mức drift gây ra, nên ngưỡng toàn cục yếu. Điều này gợi ý lưu score baseline lúc calibrate cho từng cảnh/thiết bị.

## 3. Failure case

**fail_01 — score không phát hiện được yaw 3° (lớp Metric / Preprocess).** 12/20 frame KITTI có score ở yaw 3° vẫn ≥ ngưỡng. Ví dụ frame 000048 (cây, xe đạp, bụi cây dày): score 0.060 (calib đúng) → 0.034 (yaw 3°), vẫn cao hơn ngưỡng 0.007. Nguyên nhân: cảnh lá cây cho ~3900 điểm "biên độ sâu" nhiễu (không phải biên vật thể), còn ảnh đầy cạnh Canny, nên điểm lệch 45 px vẫn trúng cạnh khác. Cảnh có ít biên rõ ràng (đường thẳng, vài xe) mới cho score nhạy. Cách phát hiện khi chạy thật: chỉ tính score trên cảnh có đủ biên sạch (ví dụ biên từ cột, mép xe), và gộp nhiều frame thay vì quyết định từng frame.

![fail 01](../results/figures/fail_01_score_miss_yaw3_000048.png)

**fail_02 — "% điểm trong ảnh" tăng khi calibration xấu đi (lớp Metric).** Trên nuScenes, xoay LiDAR 3° quanh trục x làm tỉ lệ điểm trong ảnh tăng từ 8.73% lên 9.79% (điểm vẫn trong khung hình nhưng xếp chồng sai lên vật), và KITTI tz 10 cm tăng 15.75% → 16.48%. Vì vậy % inside-FOV không thể làm chỉ số sức khoẻ calibration. Hình dưới (scene-0103_010, trên: calib đúng; dưới: xoay 3°): toàn bộ vòng quét LiDAR bị đẩy lệch lên trên khoảng 68 px (số dịch trung bình ở khoảng cách vừa) so với vật thể, nhưng số điểm trong ảnh lại tăng.

![fail 02](../results/figures/fail_02_fov_up_when_miscalibrated_scene-0103_010.png)

**Kết quả âm (Time):** bỏ bù chuyển động xe giữa lúc LiDAR và camera chụp (camera chụp sớm hơn LiDAR trung bình 35.6 ms) chỉ làm score nuScenes đổi −0.006, nhỏ hơn nhiễu giữa các frame (std 0.049–0.054), nên đồng bộ thời gian không phân biệt được bằng score này (`results/nuscenes_ego_motion_ablation.csv`).

## 4. Khuyến nghị nếu triển khai thật

Use-case: xe ADAS/robot với LiDAR + camera dùng chung để gán nhãn và fusion. Câu hỏi "bracket lệch 1° sau va chạm nhẹ thì có tự phát hiện được không": **không đáng tin nếu chỉ dựa vào một frame**. Với 1° yaw, vật xa 30 m dịch 15 px và mất 29% điểm khỏi box, nên fusion ở tầm xa hỏng trước, trong khi vật gần vẫn ổn nên khó nhận ra bằng mắt. Trade-off: score tự giám sát rẻ (chỉ Canny + vài phép nhân ma trận, chạy trên CPU) nhưng nhạy yếu và phụ thuộc cảnh; box-hit nhạy hơn nhiều nhưng cần label hoặc detector 2D+3D nên chỉ dùng được offline. Đề xuất: (1) lưu baseline score theo từng cảnh/xe lúc calibrate, (2) báo động khi **trung bình trượt** nhiều frame giảm liên tục, không báo theo từng frame, (3) kiểm tra riêng vật >30 m vì chúng nhạy nhất với góc, (4) không dùng % điểm trong ảnh làm chỉ số. Log khi chạy thật: score trung bình trượt + số điểm biên, box hit của các detection ổn định (xe đỗ) theo khoảng cách, nhiệt độ/va chạm từ IMU để đối chiếu thời điểm drift.

## 5. Cách chạy lại

Tạo môi trường (trên Windows cần `PYTHONUTF8=1` vì `requirements.txt` có chữ tiếng Việt), chạy từ thư mục gốc của repo. Toàn bộ khoảng 2–3 phút trên CPU.

```bash
python -m venv .venv
# Windows PowerShell:  .venv\Scripts\activate      macOS/Linux:  source .venv/bin/activate
export PYTHONUTF8=1      # PowerShell: $env:PYTHONUTF8="1"
pip install -r requirements.txt
python tools/verify_data.py --data-root data/kitti_mini
python tools/verify_data.py --data-root data/nuscenes_mini_subset
python -m src.test_projection
python -m starter.projection --data-root data/kitti_mini --frame 000011
python -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010
python -m src.run_topic_a all      # demo + sweep + plots + failures (xem --help để chạy từng bước)
python tools/check_submission.py
```

Kết quả: `results/calib_sweep_summary.csv`, `results/calib_sweep_frames.csv`, `results/calib_sweep_boxes.csv`, `results/nuscenes_ego_motion_ablation.csv` và các ảnh trong `results/figures/`. Code: `src/align_metrics.py` (metric), `src/run_topic_a.py` (thí nghiệm, có tham số dòng lệnh và `--help`), và 2 hàm `velo_to_cam`, `cam_to_image` trong `starter/projection.py`.

## 6. Khai báo sử dụng AI

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| Claude Code (Claude Sonnet 5.5) | Viết 2 hàm TODO trong `starter/projection.py`, viết `src/align_metrics.py` và `src/run_topic_a.py`, chạy thí nghiệm, soạn nháp báo cáo này, chẩn đoán lỗi cài đặt pip (`UnicodeDecodeError` do `requirements.txt`) | Điểm velodyne (10, 0, 0) cho z_cam = 9.73 (đúng kỳ vọng ≈ 10); đã xem ảnh overlay trên synthetic/KITTI/nuScenes, điểm khớp xe/người/mặt đường; chạy lại `sweep` + `plots` cho cùng hash CSV; số trong báo cáo đối chiếu trực tiếp với `results/calib_sweep_summary.csv`; score ban đầu phẳng đã được sửa bằng cách trừ `chance` và ghi lại trong mục 2 |
