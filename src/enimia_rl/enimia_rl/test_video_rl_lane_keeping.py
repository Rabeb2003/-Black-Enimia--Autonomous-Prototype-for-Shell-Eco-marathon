#!/usr/bin/env python3
"""
Offline RL Lane Keeping Test on Real Silesia Ring Video.
Runs the trained RL vision model on the real track video and renders:
  - Lane detection overlay (white line segmentation + corridor)
  - RL steering prediction arrow
  - Classical PD steering comparison
  - Telemetry HUD

Usage:
    python3 test_video_rl_lane_keeping.py \
        --video "/home/rabeb/Downloads/enimia_black/TRACK VIDEO - Silesia Ring ｜ Shell Eco-marathon Europe and Africa 2025 [tAJ-_mDg-W8].mp4" \
        --model  /home/rabeb/Downloads/enimia_black/src/enimia_rl/models/vision_lane_keeping_ppo.zip \
        --output /home/rabeb/Downloads/enimia_black/rl_test_output.mp4
"""

import argparse
import math
import sys
import time

import cv2
import numpy as np

try:
    from stable_baselines3 import PPO
    HAS_SB3 = True
except ImportError:
    HAS_SB3 = False
    print("[WARN] stable-baselines3 not installed. RL predictions will be skipped.")


# ─── Lane Detection (simplified offline version of fused_lane_detector) ────

def detect_lanes_frame(img):
    """Simplified lane detection for offline video analysis."""
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # White paint
    white_mask = (gray >= 170) & (hsv[:, :, 1] <= 55)

    # Grass
    grass_mask = (
        (hsv[:, :, 0] >= 30) & (hsv[:, :, 0] <= 90) &
        (hsv[:, :, 1] >= 40) & (hsv[:, :, 2] >= 40)
    )

    # Kerb red
    kerb_mask = (
        ((hsv[:, :, 0] <= 12) | (hsv[:, :, 0] >= 168)) &
        (hsv[:, :, 1] >= 50) & (hsv[:, :, 2] >= 50)
    )

    clean_white = white_mask & (~grass_mask) & (~kerb_mask)
    clean_u8 = (clean_white.astype(np.uint8) * 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    clean_u8 = cv2.morphologyEx(clean_u8, cv2.MORPH_OPEN, kernel)

    # ROI
    roi_top = int(h * 0.35)
    roi_bottom = int(h * 0.88)
    scan_rows = np.linspace(roi_top, roi_bottom, 9).astype(int)

    half_w = w // 2
    tracked_center = float(half_w)
    slice_centers = []

    for y in reversed(scan_rows):
        row_white = clean_u8[y, :] > 0
        row_grass = (grass_mask[y, :] | kerb_mask[y, :]).astype(np.uint8)
        road_mask = (row_grass == 0).astype(np.uint8)
        diff = np.diff(np.pad(road_mask, (1, 1), constant_values=0).astype(np.int32))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]

        best_left, best_right = 0, w - 1
        min_dist = math.inf
        for s, e in zip(starts, ends):
            if (e - s) > int(w * 0.12):
                mid = (s + e) / 2.0
                dist = abs(mid - tracked_center)
                if dist < min_dist:
                    min_dist = dist
                    best_left, best_right = int(s), int(e)

        white_idx = np.where(row_white)[0]
        left_w, right_w, center_w = None, None, None
        if len(white_idx) > 0:
            dw = np.diff(white_idx)
            sp = np.where(dw > 8)[0] + 1
            clusters = np.split(white_idx, sp)
            for cl in clusters:
                if len(cl) >= 2:
                    cx = float(np.mean(cl))
                    if abs(cx - tracked_center) < 200:
                        center_w = cx
                    elif cx < tracked_center - 200:
                        left_w = cx
                    elif cx > tracked_center + 200:
                        right_w = cx

        eff_l = left_w if left_w else (best_left if best_left > 12 else None)
        eff_r = right_w if right_w else (best_right if best_right < (w - 12) else None)

        row_center = None
        if center_w is not None:
            row_center = center_w
        elif eff_l is not None and eff_r is not None:
            row_center = (eff_l + eff_r) / 2.0
        elif eff_l is not None:
            row_center = eff_l + 400
        elif eff_r is not None:
            row_center = eff_r - 400

        if row_center is not None:
            row_center = float(np.clip(row_center, 30, w - 30))
            slice_centers.append((y, row_center))
            tracked_center = 0.65 * row_center + 0.35 * tracked_center

    valid = len(slice_centers) >= 2
    lat_error = 0.0
    heading_error = 0.0

    if valid:
        near = [c for (y, c) in slice_centers if y >= scan_rows[len(scan_rows) // 2]]
        far = [c for (y, c) in slice_centers if y <= scan_rows[len(scan_rows) // 2]]
        center_near = float(np.median(near)) if near else float(slice_centers[0][1])
        center_far = float(np.median(far)) if far else center_near
        lat_error = (center_near - half_w) / float(half_w)
        heading_error = (center_far - center_near) / float(half_w)

    return valid, lat_error, heading_error, slice_centers, clean_u8


# ─── Classical PD Steering ──────────────────────────────────────────────────

def classical_pd_steer(lat_error, heading_error, kp=1.15, kd=0.0, kh=0.35):
    steer = -(kp * lat_error) - kh * heading_error
    return float(np.clip(steer, -0.45, 0.45))


# ─── HUD Rendering ─────────────────────────────────────────────────────────

def render_hud(frame, valid, lat_error, heading_error,
               classical_steer, rl_steer, slice_centers, white_mask, frame_idx, fps):
    hud = frame.copy()
    h, w = hud.shape[:2]
    half_w = w // 2

    # Overlay white mask
    neon = np.zeros_like(hud)
    neon[:, :] = (0, 255, 128)
    mask3 = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
    hud = np.where(mask3 > 0, cv2.addWeighted(hud, 0.40, neon, 0.60, 0), hud)

    # Draw slice centers
    pts = []
    for y, cx in slice_centers:
        cv2.circle(hud, (int(cx), int(y)), 4, (0, 255, 255), -1)
        pts.append((int(cx), int(y)))
    if len(pts) >= 2:
        pts_sorted = sorted(pts, key=lambda p: p[1])
        for i in range(1, len(pts_sorted)):
            cv2.line(hud, pts_sorted[i - 1], pts_sorted[i], (0, 220, 255), 3)

    # Steering arrows
    arrow_y = int(h * 0.85)
    arrow_len = 120

    # Classical (blue arrow)
    cl_dx = int(arrow_len * math.sin(classical_steer))
    cv2.arrowedLine(hud, (half_w, arrow_y), (half_w + cl_dx, arrow_y - 40),
                    (255, 180, 0), 3, tipLength=0.3)
    cv2.putText(hud, f"PD: {classical_steer:+.3f}", (half_w + cl_dx + 5, arrow_y - 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 180, 0), 1)

    # RL (green arrow)
    rl_dx = int(arrow_len * math.sin(rl_steer))
    cv2.arrowedLine(hud, (half_w, arrow_y + 30), (half_w + rl_dx, arrow_y - 10),
                    (0, 255, 100), 3, tipLength=0.3)
    cv2.putText(hud, f"RL: {rl_steer:+.3f}", (half_w + rl_dx + 5, arrow_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 100), 1)

    # Telemetry card
    overlay = hud.copy()
    cv2.rectangle(overlay, (10, 10), (420, 120), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.80, hud, 0.20, 0, hud)
    cv2.rectangle(hud, (10, 10), (420, 120), (0, 255, 200), 1)

    status = "TRACKING" if valid else "SEARCH"
    color = (0, 255, 128) if valid else (0, 140, 255)
    cv2.putText(hud, f"ENIMIA RL LANE KEEPING | {status}", (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)
    cv2.putText(hud, f"Lat Error: {lat_error:+.3f}  |  Heading: {heading_error:+.3f}",
                (20, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (240, 240, 240), 1)
    cv2.putText(hud, f"Classical PD: {classical_steer:+.3f} rad  |  RL: {rl_steer:+.3f} rad",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1)
    cv2.putText(hud, f"Frame: {frame_idx}  |  FPS: {fps:.1f}",
                (20, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

    return hud


# ─── Main ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Test RL lane keeping on real Silesia Ring video")
    parser.add_argument('--video', type=str,
                        default='/home/rabeb/Downloads/enimia_black/TRACK VIDEO - Silesia Ring ｜ Shell Eco-marathon Europe and Africa 2025 [tAJ-_mDg-W8].mp4')
    parser.add_argument('--model', type=str,
                        default='/home/rabeb/Downloads/enimia_black/src/enimia_rl/models/vision_lane_keeping_ppo.zip')
    parser.add_argument('--output', type=str,
                        default='/home/rabeb/Downloads/enimia_black/rl_test_output.mp4')
    parser.add_argument('--display', action='store_true', help='Show live preview window')
    parser.add_argument('--skip-frames', type=int, default=0, help='Skip N initial frames')
    parser.add_argument('--max-frames', type=int, default=0, help='Process at most N frames (0 = all)')
    args = parser.parse_args()

    # Open video
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video: {args.video}")
        sys.exit(1)

    vid_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    vid_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    vid_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {vid_w}x{vid_h} @ {vid_fps:.1f} FPS, {total_frames} frames ({total_frames/vid_fps:.1f}s)")

    # Load RL model
    rl_model = None
    if HAS_SB3 and args.model:
        import os
        if os.path.exists(args.model):
            rl_model = PPO.load(args.model, device="cpu")
            obs_shape = rl_model.observation_space.shape
            print(f"RL Model loaded: {args.model}")
            print(f"  Observation space: {obs_shape}")
            print(f"  Action space: {rl_model.action_space}")
        else:
            print(f"[WARN] Model not found: {args.model}")

    # Output writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(args.output, fourcc, vid_fps, (vid_w, vid_h))

    # Skip frames
    for _ in range(args.skip_frames):
        cap.read()

    frame_idx = 0
    processed = 0
    t_start = time.time()

    print(f"\nProcessing... (output: {args.output})")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_idx += 1

        if args.max_frames > 0 and processed >= args.max_frames:
            break

        # Lane detection
        valid, lat_error, heading_error, slice_centers, white_mask = detect_lanes_frame(frame)

        # Classical PD steering
        classical_steer = classical_pd_steer(lat_error, heading_error) if valid else 0.0

        # RL steering prediction
        rl_steer = 0.0
        if rl_model is not None:
            try:
                obs_shape = rl_model.observation_space.shape
                if len(obs_shape) == 3:
                    # CNN: (3, 64, 64) uint8
                    resized = cv2.resize(frame, (64, 64))
                    obs = np.transpose(resized, (2, 0, 1)).astype(np.uint8)
                else:
                    # MLP: flattened float image + scalars
                    resized = cv2.resize(frame, (64, 64))
                    img_flat = (resized.astype(np.float32) / 255.0).flatten()
                    scalars = np.array([lat_error, 5.0, classical_steer], dtype=np.float32)
                    obs = np.concatenate([img_flat, scalars])
                action, _ = rl_model.predict(obs, deterministic=True)
                rl_steer = float(action[0])
            except Exception as e:
                if frame_idx % 100 == 1:
                    print(f"[WARN] RL inference error at frame {frame_idx}: {e}")

        # Render HUD
        elapsed = time.time() - t_start
        fps = (processed + 1) / max(0.01, elapsed)
        hud = render_hud(frame, valid, lat_error, heading_error,
                         classical_steer, rl_steer, slice_centers, white_mask, frame_idx, fps)

        out.write(hud)
        processed += 1

        if args.display:
            cv2.imshow("ENIMIA RL Lane Keeping Test", hud)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if processed % 200 == 0:
            print(f"  Frame {frame_idx}/{total_frames} | FPS: {fps:.1f} | "
                  f"LatErr: {lat_error:+.3f} | PD: {classical_steer:+.3f} | RL: {rl_steer:+.3f}")

    cap.release()
    out.release()
    if args.display:
        cv2.destroyAllWindows()

    total_time = time.time() - t_start
    print(f"\n✅ Done! Processed {processed} frames in {total_time:.1f}s ({processed/total_time:.1f} FPS)")
    print(f"   Output: {args.output}")


if __name__ == "__main__":
    main()
