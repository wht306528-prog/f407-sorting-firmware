#!/usr/bin/env python3
"""分拣设备 Calibration 一体化入口。

子命令：solve-board | merge-patch | fit-tray | fit-arm | verify | one-table | apply-f407

通用参数在所有子命令末尾附加：例如
  python calibrate_device.py --config calibration_config.json \\
      solve-board --image images/extrinsic/tray1_pose_001.jpg

一表流示例：
  python calibrate_device.py one-table --points data/calibration_points.csv \\
      --images images/extrinsic --results calibration_results.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Tuple

import numpy as np

CAL_DIR = Path(__file__).resolve().parent

DEFAULT_SERVO_PULSE_PER_REV = 10000.0
DEFAULT_MOTOR_TO_JOINT_RATIO = 30.0


try:
    from apply_calibration_to_f407 import resolve_arm_geometry

    _HAVE_APPLY_CALIB_MOD = True
except ImportError:
    _HAVE_APPLY_CALIB_MOD = False

try:
    from scipy.optimize import least_squares  # type: ignore[import-untyped]

    _HAVE_SCIPY = True
except Exception:
    _HAVE_SCIPY = False

CAL_EPS_FB = 1e-4


def _sep(title: str = "") -> None:
    bar = "=" * 56
    if title:
        print(f"\n{bar}\n  {title}\n{bar}")
    else:
        print(f"\n{bar}")


def _hint_next(msg: str) -> None:
    print(f"\n>>> 下一步建议: {msg}")


def _abs_or_rel(p: Path) -> str:
    try:
        return str(p.resolve().relative_to(Path.cwd()))
    except ValueError:
        return str(p.resolve())


def _load_cfg(path: Path | None) -> dict[str, Any]:
    if path is None or not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_results_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open("r", encoding="utf-8") as f:
        d = json.load(f)
    return d if isinstance(d, dict) else {}


def save_results_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def rigid_transform_3d(
    a: np.ndarray, b: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    assert a.shape == b.shape and a.shape[1] == 3
    ca = a.mean(axis=0)
    cb = b.mean(axis=0)
    aa = a - ca
    bb = b - cb
    h_mx = aa.T @ bb
    u, _, vt = np.linalg.svd(h_mx)
    r_m = vt.T @ u.T
    if np.linalg.det(r_m) < 0:
        vt[-1, :] *= -1.0
        r_m = vt.T @ u.T
    t_v = cb - r_m @ ca
    return r_m, t_v


def to_homogeneous(r_m: np.ndarray, t_v: np.ndarray) -> np.ndarray:
    te = np.eye(4, dtype=np.float64)
    te[:3, :3] = r_m
    te[:3, 3] = t_v
    return te


def ik_two_link(
    xx: float, yy: float, l1: float, l2: float
) -> tuple[float, float] | None:
    r2 = xx * xx + yy * yy
    c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    if c2 > 1.0 or c2 < -1.0:
        return None
    s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(yy, xx) - math.atan2(
        l2 * math.sin(t2), l1 + l2 * math.cos(t2)
    )
    return math.degrees(t1), math.degrees(t2)


def _circle_circle_ix_fb(
    xa: float, ya: float, ra: float, xb: float, yb: float, rb: float
) -> list[tuple[float, float]]:
    dx = xb - xa
    dy = yb - ya
    d2 = dx * dx + dy * dy
    if d2 < CAL_EPS_FB * CAL_EPS_FB:
        return []
    d = math.sqrt(d2)
    if d > ra + rb + CAL_EPS_FB or d < abs(ra - rb) - CAL_EPS_FB:
        return []
    a = (ra * ra - rb * rb + d * d) / (2.0 * d)
    hh = ra * ra - a * a
    if hh < -CAL_EPS_FB:
        return []
    hh = max(0.0, hh)
    hh_r = math.sqrt(hh)
    mx = xa + dx * a / d
    my = ya + dy * a / d
    ox = -(dy / d) * hh_r
    oy = (dx / d) * hh_r
    p1 = (mx + ox, my + oy)
    p2 = (mx - ox, my - oy)
    return [p1] if hh <= CAL_EPS_FB else [p1, p2]


def _choose_elbow_higher_y(
    uv1: tuple[float, float], uv2: tuple[float, float]
) -> tuple[float, float]:
    return uv1 if uv1[1] >= uv2[1] else uv2


def _segment_dist_ok_fb(xa: float, ya: float, xb: float, yb: float, rex: float) -> bool:
    return abs(math.hypot(xa - xb, ya - yb) - rex) <= 5.0


def fb_inv_deg_symmetric(
    Ex: float,
    Ey: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
    lal: float,
    pal: float,
    lar: float,
    par: float,
    jz1_deg: float,
    jz2_deg: float,
    s1: float,
    s2: float,
) -> tuple[float, float] | None:
    """与固件 fb_inv（app_kinematics.c）语义一致：输出命令关节角 °。"""
    pts_c = _circle_circle_ix_fb(ax, ay, lal, Ex, Ey, pal)
    if not pts_c:
        return None
    if len(pts_c) == 2:
        Cx, Cy = _choose_elbow_higher_y(pts_c[0], pts_c[1])
    else:
        Cx, Cy = pts_c[0]

    pts_d = _circle_circle_ix_fb(bx, by, lar, Ex, Ey, par)
    if not pts_d:
        return None
    if len(pts_d) == 2:
        Dx, Dy = _choose_elbow_higher_y(pts_d[0], pts_d[1])
    else:
        Dx, Dy = pts_d[0]

    if not _segment_dist_ok_fb(Ex, Ey, Cx, Cy, pal) or not _segment_dist_ok_fb(
        Ex, Ey, Dx, Dy, par
    ):
        return None

    ia1_deg = math.degrees(math.atan2(Cy - ay, Cx - ax))
    ia2_deg = math.degrees(math.atan2(Dy - by, Dx - bx))
    t1 = s1 * (ia1_deg + jz1_deg)
    t2 = s2 * (ia2_deg + jz2_deg)
    return t1, t2


def undistort_uv(
    u: float, v: float, k: np.ndarray, d: np.ndarray
) -> tuple[float, float]:
    fx, fy, cx, cy = k[0, 0], k[1, 1], k[0, 2], k[1, 2]
    x = (u - cx) / fx
    y = (v - cy) / fy
    k1, k2, p1, p2, k3 = d.ravel()[:5]
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_dist = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_dist = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    x = x - (x_dist - x)
    y = y - (y_dist - y)
    return cx + fx * x, cy + fy * y


def pipeline_arm_xy(
    u: float,
    v: float,
    z_mm: float,
    k: np.ndarray,
    dist: np.ndarray,
    r_tc: np.ndarray,
    t_tc: np.ndarray,
    t4: np.ndarray,
) -> np.ndarray:
    ud, vd = undistort_uv(u, v, k, dist)
    zc = float(z_mm)
    xc = (ud - k[0, 2]) * zc / k[0, 0]
    yc = (vd - k[1, 2]) * zc / k[1, 1]
    pc = np.array([xc, yc, zc], dtype=np.float64)
    pt = r_tc @ pc + t_tc
    ph = np.array([pt[0], pt[1], pt[2], 1.0], dtype=np.float64)
    pa = t4 @ ph
    return pa[:3]


def tray_mm_to_cam_xyz(
    tray_xyz: np.ndarray,
    r_tc: np.ndarray,
    t_tc: np.ndarray,
) -> np.ndarray:
    """tray_from_cam 约定：P_tray = R @ P_cam + t → P_cam = Rᵀ @ (P_tray - t)。"""
    p = np.asarray(tray_xyz[:3], dtype=np.float64).reshape(3)
    rt = np.asarray(r_tc, dtype=np.float64).reshape(3, 3)
    tt = np.asarray(t_tc, dtype=np.float64).reshape(3)
    return rt.T @ (p - tt)


def cam_xyz_to_uv_distorted(
    pc_cam: np.ndarray,
    k: np.ndarray,
    dist: np.ndarray,
) -> tuple[float, float]:
    """相机坐标系下 3D 点投影到像素（含畸变）；需 opencv-python。"""
    import cv2

    kk = np.asarray(k, dtype=np.float64)
    dd = np.asarray(dist, dtype=np.float64).reshape(-1, 1)
    pts, _ = cv2.projectPoints(
        np.asarray(pc_cam, dtype=np.float64).reshape(1, 1, 3),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        kk,
        dd,
    )
    return float(pts[0, 0, 0]), float(pts[0, 0, 1])


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        sample = f.read(8192)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
        except csv.Error:
            dialect = csv.excel
        rdr = csv.DictReader(f, dialect=dialect)
        assert rdr.fieldnames is not None
        for row in rdr:
            first_cell = next(iter(row.values()), "")
            if str(first_cell).strip().startswith("#"):
                continue
            out.append(dict(row))
    return out


def _pick_float(row: dict[str, str], keys: tuple[str, ...]) -> float | None:
    for kk in keys:
        if kk in row and row[kk].strip():
            try:
                return float(row[kk])
            except ValueError:
                return None
    return None


def _ensure_camera_in_results(data: dict[str, Any], cfg: dict[str, Any]) -> None:
    """保证 verify / one-table 能从 results.camera 读到 fx,fy,cx,cy,D。"""
    cam = data.setdefault("camera", {})
    if cam.get("fx") is not None and cam.get("fy") is not None:
        return
    cc = cfg.get("camera")
    if not isinstance(cc, dict) or "K" not in cc:
        return
    K = np.asarray(cc["K"], dtype=np.float64)
    cam["fx"] = float(K[0, 0])
    cam["fy"] = float(K[1, 1])
    cam["cx"] = float(K[0, 2])
    cam["cy"] = float(K[1, 2])
    dd = cc.get("D", [0.0] * 5)
    cam["D"] = [float(x) for x in dd]


def _glob_extrinsic_images(folder: Path) -> list[Path]:
    """收集 extrinsic 目录下常见图像后缀。"""
    exts = ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG")
    seen: set[Path] = set()
    out: list[Path] = []
    for pat in exts:
        for p in folder.glob(pat):
            rp = p.resolve()
            if rp not in seen:
                seen.add(rp)
                out.append(p)
    return sorted(out, key=lambda x: x.name.lower())


def pulses_to_meas_joint_deg(
    p1: float,
    p2: float,
    zp1: float,
    zp2: float,
    s1: float,
    s2: float,
    pulse_per_rev: float,
    ratio: float,
) -> tuple[float, float]:
    """
    伺服绝对脉冲 → 命令关节角 °（与固件 pulse = joint_deg * ratio * pulse_per_rev / 360 互逆）。
    zp* 为关节角定义零点处的脉冲偏置（CALIB_JOINT*_ZERO_PULSE）。
    """
    m1 = (p1 - zp1) * 360.0 / pulse_per_rev / ratio * s1
    m2 = (p2 - zp2) * 360.0 / pulse_per_rev / ratio * s2
    return m1, m2


def _csv_role(row: dict[str, str]) -> str:
    raw = row.get("role", "fit")
    if raw is None or str(raw).strip() == "":
        return "fit"
    return str(raw).strip().lower()


# --- GP100 solve-board（原 solve_board_pose.py，内联；需 opencv-python）---
_SB_CAM_K = np.array(
    [[608.356, 0.0, 323.566], [0.0, 607.887, 246.759], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_SB_CAM_D = np.zeros((5, 1), dtype=np.float64)
_SB_BOARD_COLS = 11
_SB_BOARD_ROWS = 8
_SB_SQUARE_MM_X = 6.0
_SB_SQUARE_MM_Y = 6.0


def _sb_load_camera_board_from_cfg(path: Path) -> tuple[np.ndarray, np.ndarray, Tuple[int, int], Tuple[float, float]]:
    with path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)
    c = cfg["camera"]
    K = np.array(c["K"], dtype=np.float64)
    D = np.array(c["D"], dtype=np.float64).reshape(-1, 1)
    b = cfg["board_gp100"]
    w, h = b["opencv_inner_corners"]
    sq = float(b["square_mm"])
    return K, D, (int(w), int(h)), (sq, sq)


def _sb_chess_board_corners(cv2: Any, gray: np.ndarray, board_size: Tuple[int, int]) -> np.ndarray:
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)
    if not ret or corners is None:
        raise RuntimeError(f"未找到棋盘角点 pattern={board_size}")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return corners_refined.reshape(-1, 2).astype(np.float64)


def _sb_reorder_corners_open_cv_style(
    pts: np.ndarray, board_size: Tuple[int, int]
) -> np.ndarray:
    w, h = board_size[0], board_size[1]
    out = pts.copy()
    if out[0, 0] > out[-1, 0]:
        for row in range(h):
            for j in range(w // 2):
                a = row * w + j
                b = (row + 1) * w - j - 1
                out[[a, b]] = out[[b, a]]
    if out[0, 1] < out[-1, 1]:
        for col in range(w):
            for j in range(h // 2):
                a = j * w + col
                b = (h - j - 1) * w + col
                out[[a, b]] = out[[b, a]]
    return out


def _sb_build_object_points(
    board_size: Tuple[int, int], square_mm: Tuple[float, float]
) -> np.ndarray:
    w, h = board_size
    sx, sy = square_mm
    obj = []
    for j in range(h):
        for i in range(w):
            obj.append([i * sx, j * sy, 0.0])
    return np.asarray(obj, dtype=np.float64)


def _sb_board_to_tray_from_cam(
    R_bc: np.ndarray, t_bc: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    R_tray = R_bc.T
    t_tray = (-R_bc.T @ t_bc.reshape(3, 1)).ravel()
    return R_tray, t_tray


def _sb_load_cam_board_defaults(
    config_path: Path | None,
) -> tuple[np.ndarray, np.ndarray, Tuple[int, int], Tuple[float, float]]:
    """加载相机 K,D 与棋盘内角点数、格距 mm。"""
    K, D = _SB_CAM_K, _SB_CAM_D
    board_size: Tuple[int, int] = (_SB_BOARD_COLS, _SB_BOARD_ROWS)
    square_mm = (_SB_SQUARE_MM_X, _SB_SQUARE_MM_Y)
    if config_path is not None and config_path.is_file():
        K, D, board_size, square_mm = _sb_load_camera_board_from_cfg(config_path)
    return K, D, board_size, square_mm


def gp100_estimate_extrinsic(
    image_path: Path,
    config_path: Path | None,
) -> tuple[dict[str, Any] | None, str]:
    """
    估计 GP100 外参并返回 patch；附带棋盘平均/最大重投影误差（像素）。
    失败返回 (None, 错误说明)。
    """
    try:
        import cv2
    except ImportError:
        return None, "需要 opencv-python：pip install opencv-python"

    try:
        K, D, board_size, square_mm = _sb_load_cam_board_defaults(config_path)
    except (KeyError, TypeError, ValueError) as e:
        return None, f"读取 calibration_config 失败: {e}"

    img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if img is None or img.size == 0:
        return None, "无法读取图像"

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    try:
        pts = _sb_chess_board_corners(cv2, gray, board_size)
    except RuntimeError as e:
        return None, str(e)

    pts = _sb_reorder_corners_open_cv_style(pts, board_size)
    obj = _sb_build_object_points(board_size, square_mm)
    img_pts = pts.astype(np.float32).reshape(-1, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(
        obj.astype(np.float32),
        img_pts,
        K,
        D,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None, "solvePnP 失败"

    proj, _ = cv2.projectPoints(obj.astype(np.float64), rvec, tvec, K, D)
    proj = proj.reshape(-1, 2).astype(np.float64)
    diff = np.linalg.norm(proj - pts, axis=1)
    mean_px = float(np.mean(diff))
    max_px = float(np.max(diff))

    R_bc, _ = cv2.Rodrigues(rvec)
    t_bc = tvec.reshape(3)
    R_tray, t_tray = _sb_board_to_tray_from_cam(R_bc, t_bc)

    patch = {
        "extrinsic_board_to_cam": {"R": R_bc.tolist(), "t_mm": t_bc.tolist()},
        "tray_from_cam": {"R": R_tray.tolist(), "t_mm": t_tray.tolist()},
    }

    info: dict[str, Any] = {
        "patch": patch,
        "mean_reproj_px": mean_px,
        "max_reproj_px": max_px,
        "K": K,
        "D": D,
        "board_size": board_size,
        "square_mm": square_mm,
        "gray": gray,
        "img_pts": img_pts,
        "rvec": rvec,
        "tvec": tvec,
        "obj_pts": obj,
    }
    return info, ""


def run_gp100_solve_board(
    image_path: Path,
    config_path: Path | None,
    save_corners: Path | None,
    json_out: Path | None,
) -> int:
    try:
        import cv2
    except ImportError:
        print(
            "solve-board 需要 opencv-python，请执行: pip install opencv-python",
            file=sys.stderr,
        )
        return 1

    est, err = gp100_estimate_extrinsic(image_path, config_path)
    if est is None:
        print(err, file=sys.stderr)
        return 1

    K = est["K"]
    D = est["D"]
    board_size = est["board_size"]
    img_pts = est["img_pts"]
    patch = est["patch"]
    R_bc = np.asarray(patch["extrinsic_board_to_cam"]["R"], dtype=np.float64)
    t_bc = np.asarray(patch["extrinsic_board_to_cam"]["t_mm"], dtype=np.float64)
    R_tray = np.asarray(patch["tray_from_cam"]["R"], dtype=np.float64)
    t_tray = np.asarray(patch["tray_from_cam"]["t_mm"], dtype=np.float64)

    print(f"棋盘重投影误差: mean={est['mean_reproj_px']:.4f}px max={est['max_reproj_px']:.4f}px")
    print("R_board_to_cam (P_cam = R @ P_board + t):")
    print(R_bc)
    print("t_board_to_cam_mm:", t_bc)
    print("R_tray_from_cam (P_tray = R @ P_cam + t):")
    print(R_tray)
    print("t_tray_from_cam_mm:", t_tray)

    if save_corners:
        gray = est["gray"]
        try:
            import cv2
        except ImportError:
            cv2 = None  # type: ignore[assignment]
        assert cv2 is not None
        if gray.ndim == 2:
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            bgr = gray.copy()
        save_corners.parent.mkdir(parents=True, exist_ok=True)
        cv2.drawChessboardCorners(bgr, board_size, img_pts, True)
        cv2.imwrite(str(save_corners), bgr)

    if json_out:
        json_out.parent.mkdir(parents=True, exist_ok=True)
        json_out.write_text(
            json.dumps(patch, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        print("已写入", json_out)

    return 0


# --- handlers ---


def cmd_solve_board(args: argparse.Namespace) -> int:
    img = Path(args.image)
    if not img.is_file():
        print("找不到图像:", img, file=sys.stderr)
        return 1
    cfg_p = Path(args.config)
    _sep("solve-board：棋盘外参（GP100）")
    print("图像:", _abs_or_rel(img))
    print("结果将写入 --results:", _abs_or_rel(Path(args.results)))
    if cfg_p.is_file():
        print("配置:", _abs_or_rel(cfg_p))
    else:
        print(
            "警告: 未找到 calibration_config.json，将使用内置默认相机/GP100 棋盘参数。",
            file=sys.stderr,
        )
    if args.json_out:
        print("外参片段 JSON:", _abs_or_rel(Path(args.json_out)))
    rc = run_gp100_solve_board(
        img,
        cfg_p if cfg_p.is_file() else None,
        Path(args.save_corners) if args.save_corners else None,
        Path(args.json_out) if args.json_out else None,
    )
    if rc == 0:
        json_part = (
            f'python calibrate_device.py merge-patch "{args.json_out}"'
            if args.json_out
            else "python calibrate_device.py merge-patch Calibration/output/extrinsic_*.json"
        )
        _hint_next(
            f"将 patch 合并到标定结果: {json_part}  "
            f"(或指定多个 patch 文件一次合并)"
        )
    return rc


def cmd_merge_patch(args: argparse.Namespace) -> int:
    path_res = Path(args.results)
    _sep("merge-patch：合并外参/片段到 results")
    print("写入:", _abs_or_rel(path_res))
    merged_top_keys: list[str] = []
    data = load_results_json(path_res)
    for pj in args.patch_json:
        pth = Path(pj)
        if not pth.is_file():
            print("找不到:", pth, file=sys.stderr)
            return 1
        patch = json.loads(pth.read_text(encoding="utf-8"))
        if not isinstance(patch, dict):
            return 1
        for dk in list(patch):
            if isinstance(dk, str) and dk.startswith("_"):
                del patch[dk]
        merged_top_keys.extend(sorted(patch.keys()))
        data.update(patch)
    data["merged_patches_from"] = [str(Path(x).resolve()) for x in args.patch_json]
    save_results_json(path_res, data)
    print("已合并至:", _abs_or_rel(path_res))
    if merged_top_keys:
        uniq = sorted(set(merged_top_keys))
        print("本次补丁涉及的顶层键:", ", ".join(uniq))
    _hint_next(
        "求 T_tray_to_arm: python calibrate_device.py fit-tray --csv data/tray_arm_points.csv"
        "（已有点位与外参后继续）或先运行 verify 检查几何链路。"
    )
    return 0


def cmd_fit_tray(args: argparse.Namespace) -> int:
    _sep("fit-tray：触点刚体对齐 -> T_tray_to_arm")
    print("CSV:", _abs_or_rel(Path(args.csv)))
    print("结果文件:", _abs_or_rel(Path(args.results)))
    cfg = _load_cfg(args.config if args.config.is_file() else None)
    board = cfg.get("board_gp100", {})
    sq = (
        args.square_mm
        if getattr(args, "square_mm", None) is not None
        else float(board.get("square_mm", 6.0))
    )
    trays: list[list[float]] = []
    arms: list[list[float]] = []
    for row in read_csv_rows(Path(args.csv)):
        xa = _pick_float(row, ("x_arm_mm", "x_arm"))
        ya = _pick_float(row, ("y_arm_mm", "y_arm"))
        za = _pick_float(row, ("z_arm_mm", "z_arm")) or 0.0
        if xa is None or ya is None:
            continue
        tx = _pick_float(row, ("tray_x_mm",))
        ty = _pick_float(row, ("tray_y_mm",))
        ci = _pick_float(row, ("corner_col_idx", "col_idx"))
        ri = _pick_float(row, ("corner_row_idx", "row_idx"))
        if tx is not None and ty is not None:
            tz = _pick_float(row, ("tray_z_mm",)) or 0.0
            trays.append([tx, ty, tz])
        elif ci is not None and ri is not None:
            trays.append([ci * sq, ri * sq, 0.0])
        else:
            print("跳过行:", row)
            continue
        arms.append([xa, ya, za])

    if len(trays) < 3:
        print("有效触点少于 3 个")
        return 1
    aa = np.asarray(trays, dtype=np.float64)
    bb = np.asarray(arms, dtype=np.float64)
    r_m, t_v = rigid_transform_3d(aa, bb)
    tea = to_homogeneous(r_m, t_v)

    # 残差：预测臂位 = R @ tray + t（与 rigid_transform_3d 约定一致）
    pred = (r_m @ aa.T).T + t_v.reshape(1, 3)
    diff = bb - pred
    dist = np.linalg.norm(diff, axis=1)
    rms_pts = float(np.sqrt(np.mean(dist**2)))
    max_pts = float(np.max(dist))
    print(f"使用 {len(trays)} 组触点; 棋盘格距 square_mm={sq}")
    print(f"刚体对齐残差 RMS={rms_pts:.4f} mm, max={max_pts:.4f} mm")
    for i, (d_i, di) in enumerate(zip(diff, dist)):
        print(
            f"  点{i + 1}: Δarm=({d_i[0]:+.3f},{d_i[1]:+.3f},{d_i[2]:+.3f}) mm, |Δ|={di:.3f} mm"
        )

    outp = Path(args.results)
    data = load_results_json(outp)
    data["T_tray_to_arm"] = tea.tolist()
    save_results_json(outp, data)
    print("已写入 T_tray_to_arm 到:", _abs_or_rel(outp))
    _hint_next(
        "关节与杆长: python calibrate_device.py fit-arm --csv data/joint_observations.csv"
        "；然后复测: python calibrate_device.py verify"
    )
    return 0


def cmd_fit_arm(args: argparse.Namespace) -> int:
    _sep("fit-arm：对称五杆 + 零点（与 app_kinematics.c 一致）")
    print("CSV:", _abs_or_rel(Path(args.csv)))
    print("结果文件:", _abs_or_rel(Path(args.results)))

    outp = Path(args.results)
    data = load_results_json(outp)
    cfg = _load_cfg(args.config if args.config.is_file() else None)
    if isinstance(cfg.get("arm_nominal_mm"), dict):
        data.setdefault("arm_nominal_mm", cfg["arm_nominal_mm"])

    if _HAVE_APPLY_CALIB_MOD:
        resolve_arm_geometry(data)
    else:
        print(
            "警告: 无法 import resolve_arm_geometry，使用 results 中的 arm 原样字段。",
            file=sys.stderr,
        )

    obs: list[tuple[float, float, float, float]] = []
    for row in read_csv_rows(Path(args.csv)):
        try:
            xw = float(row["xw_mm"])
            yw = float(row["yw_mm"])
            m1 = float(row["meas_theta1_deg"])
            m2 = float(row["meas_theta2_deg"])
        except (KeyError, ValueError):
            continue
        obs.append((xw, yw, m1, m2))

    if not obs:
        print("无有效关节观测 CSV 行（需列 xw_mm,yw_mm,meas_theta1_deg,...）")
        return 1

    arm = data.get("arm", {})
    if not isinstance(arm, dict):
        print("results 缺少 arm 字段。", file=sys.stderr)
        return 1

    ax = float(arm.get("ax_mm", -250.0))
    ay = float(arm.get("ay_mm", 0.0))
    bx = float(arm.get("bx_mm", 250.0))
    by = float(arm.get("by_mm", 0.0))
    lal = (
        float(args.l1)
        if args.l1 is not None
        else float(arm.get("L_active_left_mm", arm.get("L1_mm", 600.0)))
    )
    lar = (
        float(args.l4)
        if args.l4 is not None
        else float(arm.get("L_active_right_mm", arm.get("L4_mm", 600.0)))
    )
    pal = float(arm.get("L_passive_left_mm", 700.0))
    par = float(arm.get("L_passive_right_mm", 700.0))
    arm["L_active_left_mm"] = lal
    arm["L_active_right_mm"] = lar
    arm["L_passive_left_mm"] = pal
    arm["L_passive_right_mm"] = par
    arm["L1_mm"] = float(lal)
    arm["L4_mm"] = float(lar)

    j1_base = float(arm.get("joint1_zero_deg", 0.0))
    j2_base = float(arm.get("joint2_zero_deg", 0.0))
    j1s = float(getattr(args, "joint1_sign", arm.get("joint1_sign", 1.0)))
    j2s = float(getattr(args, "joint2_sign", arm.get("joint2_sign", 1.0)))

    arm["joint1_sign"] = j1s
    arm["joint2_sign"] = j2s

    best = (1e9, 0.0, 0.0)

    def rms_for_deltas(dd1: float, dd2: float) -> float | None:
        errs2: list[float] = []
        jza = j1_base + dd1
        jzb = j2_base + dd2
        for xw, yw, mm1, mm2 in obs:
            sol = fb_inv_deg_symmetric(
                xw, yw, ax, ay, bx, by, lal, pal, lar, par, jza, jzb, j1s, j2s
            )
            if sol is None:
                return None
            p1, p2 = sol
            errs2.append((p1 - mm1) ** 2 + (p2 - mm2) ** 2)
        return math.sqrt(sum(errs2) / len(errs2))

    for d1 in np.linspace(-5.0, 5.0, 11):
        for d2 in np.linspace(-5.0, 5.0, 11):
            r = rms_for_deltas(float(d1), float(d2))
            if r is None:
                continue
            if r < best[0]:
                best = (r, float(d1), float(d2))

    if best[0] > 1e8:
        print(
            "拟合失败：当前五杆几何下网格内无完备逆解，请检查触点/观测角或加长主动杆检索范围。",
            file=sys.stderr,
        )
        return 1

    rms, dd1, dd2 = best
    sol_note = "five_bar_grid"

    if getattr(args, "scipy", False):
        if not _HAVE_SCIPY:
            print("警告: 未安装 scipy，忽略 --scipy。", file=sys.stderr)
        else:

            def pack(v: np.ndarray) -> np.ndarray:
                e: list[float] = []
                jza = j1_base + float(v[0])
                jzb = j2_base + float(v[1])
                for xw, yw, mm1, mm2 in obs:
                    sol = fb_inv_deg_symmetric(
                        xw,
                        yw,
                        ax,
                        ay,
                        bx,
                        by,
                        lal,
                        pal,
                        lar,
                        par,
                        jza,
                        jzb,
                        j1s,
                        j2s,
                    )
                    if sol is None:
                        e.extend([50.0, 50.0])
                    else:
                        p1, p2 = sol
                        e.append(p1 - mm1)
                        e.append(p2 - mm2)
                return np.array(e, dtype=np.float64)

            ls = least_squares(
                pack,
                x0=np.array([dd1, dd2], dtype=np.float64),
                bounds=(
                    np.array([-8.0, -8.0], dtype=np.float64),
                    np.array([8.0, 8.0], dtype=np.float64),
                ),
                max_nfev=600,
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
            )
            dd1 = float(ls.x[0])
            dd2 = float(ls.x[1])
            rs = rms_for_deltas(dd1, dd2)
            if rs is not None:
                rms = rs
            sol_note = "five_bar_grid+scipy_leastsq"

    arm["joint1_zero_deg"] = float(j1_base + dd1)
    arm["joint2_zero_deg"] = float(j2_base + dd2)
    arm.setdefault(
        "joint1_zero_pulse", int(arm.get("joint1_zero_pulse", 0))
    )
    arm.setdefault(
        "joint2_zero_pulse", int(arm.get("joint2_zero_pulse", 0))
    )
    data.setdefault("fit_arm", {})
    data["fit_arm"].update(
        {"rms_joint_deg": rms, "solver": sol_note, "points": len(obs)}
    )
    save_results_json(outp, data)
    print(
        f"观测点数={len(obs)}; rms_joint_deg={rms:.4f}, "
        f"zero_deg=({arm['joint1_zero_deg']:.4f},{arm['joint2_zero_deg']:.4f}) "
        f"(基线+微调 {dd1:+.4f},{dd2:+.4f})"
    )
    print("已写入 arm 与 fit_arm 至:", _abs_or_rel(outp))
    _hint_next(
        "像素+深度复测: python calibrate_device.py verify --csv <自建复测.csv> "
        "（一表流可用总表 role=verify）"
    )
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    res_path = Path(args.results)
    _sep("verify：像素+深度 -> 臂基 XY 误差")
    print("读取:", _abs_or_rel(res_path))
    data = load_results_json(res_path)
    cfg = _load_cfg(args.config if args.config.is_file() else None)

    if not data:
        print("缺少", res_path, file=sys.stderr)
        return 1

    cam = data.get("camera", {})
    fx = float(cam.get("fx", 608.356))
    fy = float(cam.get("fy", 607.887))
    cx = float(cam.get("cx", 323.566))
    cy = float(cam.get("cy", 246.759))
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = np.array(cam.get("D", [0.0] * 5), dtype=np.float64).reshape(-1, 1)
    tc = data.get("tray_from_cam", {})
    r_tc = np.asarray(tc.get("R", np.eye(3).tolist()), dtype=np.float64)
    t_tc = np.asarray(tc.get("t_mm", [0.0, 0.0, 0.0]), dtype=np.float64)
    t4 = np.asarray(data.get("T_tray_to_arm", np.eye(4)), dtype=np.float64)
    max_acc = float(data.get("quality", {}).get("max_error_mm", 3.0))

    cpath = Path(args.csv)
    if not cpath.is_file():
        print(
            "复测必须提供存在的 CSV（请在命令行指定 --csv）。",
            file=sys.stderr,
        )
        print(f"当前路径不存在: {cpath.resolve()}", file=sys.stderr)
        return 1

    points: list[tuple[float, float, float, float, float]] = []
    for row in read_csv_rows(cpath):
        uu = _pick_float(row, ("u_px", "u"))
        vv = _pick_float(row, ("v_px", "v"))
        zz = _pick_float(row, ("z_mm", "z"))
        xt = _pick_float(row, ("x_arm_truth_mm",))
        yt = _pick_float(row, ("y_arm_truth_mm",))
        if None in (uu, vv, zz, xt, yt):
            continue
        points.append((uu, vv, zz, xt, yt))

    if not points:
        print(
            "CSV 中无有效复测行（需列 u_px,v_px,z_mm,x_arm_truth_mm,y_arm_truth_mm）。",
            file=sys.stderr,
        )
        return 1

    print("复测点来源: CSV", _abs_or_rel(cpath))

    errs: list[float] = []
    for uu, vv, zz, xt, yt in points:
        pr = pipeline_arm_xy(uu, vv, zz, k, dist, r_tc, t_tc, t4)
        ee = float(np.linalg.norm(pr[:2] - np.array([xt, yt])))
        errs.append(ee)
        print(
            f"u,v,z={uu},{vv},{zz} -> pred=({pr[0]:.2f},{pr[1]:.2f}) truth=({xt},{yt}) err={ee:.3f} mm",
        )

    mean_e = float(np.mean(errs))
    max_e = float(np.max(errs))
    print(f"mean_mm {mean_e} max_mm {max_e} accept_max_mm {max_acc}")
    if max_e <= max_acc:
        print(f"判定: PASS（最大误差不超过阈值 {max_acc} mm）")
    else:
        print(
            f"判定: FAIL（最大误差 {max_e} mm 超过阈值 {max_acc} mm），请检查外参、触点与相机深度。",
            file=sys.stderr,
        )

    qc = data.setdefault("quality", {})
    qc["verify_mean_mm"] = mean_e
    qc["verify_max_mm"] = max_e
    cov = data.setdefault("conveyor", {})
    if isinstance(cfg.get("conveyor"), dict):
        cov.update(cfg["conveyor"])
    cov.setdefault("mm_per_pulse", 0.0)
    cov.setdefault("delta_pulse_for_test", 0)
    save_results_json(res_path, data)
    print("已更新 quality.verify_* 并写回:", _abs_or_rel(res_path))
    _hint_next(
        "写入 F407: python apply_calibration_to_f407.py --config calibration_config.json "
        f"--results {res_path.name} --apply "
        "或: python calibrate_device.py apply-f407 --apply"
    )
    return 0 if max_e <= max_acc else 2


def cmd_one_table(args: argparse.Namespace) -> int:
    """
    最短标定链：多张 GP100 外参自动择优 → fit-tray →（可选脉冲）fit-arm →（可选）verify。
    """
    _sep("one-table：GP100 自动选图 + 触点总表 → tray/arm")
    cfg_path = Path(args.config)
    cfg = _load_cfg(cfg_path if cfg_path.is_file() else None)
    res_path = Path(args.results)
    data = load_results_json(res_path)
    _ensure_camera_in_results(data, cfg)

    img_dir = Path(args.images_dir)
    if not img_dir.is_dir():
        print("图像目录不存在:", img_dir.resolve(), file=sys.stderr)
        return 1

    cand = _glob_extrinsic_images(img_dir)
    if not cand:
        print("目录内无 jpg/jpeg/png:", img_dir.resolve(), file=sys.stderr)
        return 1

    print(f"扫描外参图 {len(cand)} 张: {_abs_or_rel(img_dir)}")

    best_img: Path | None = None
    best_est: dict[str, Any] | None = None
    best_mean = float("inf")

    cfg_for_gp = cfg_path if cfg_path.is_file() else None
    for img in cand:
        est, err = gp100_estimate_extrinsic(img, cfg_for_gp)
        if est is None:
            print(f"  跳过 {_abs_or_rel(img)}: {err}")
            continue
        mpx = float(est["mean_reproj_px"])
        print(
            f"  {_abs_or_rel(img)} mean_reproj={mpx:.4f}px "
            f"max={est['max_reproj_px']:.4f}px"
        )
        if mpx < best_mean:
            best_mean = mpx
            best_img = img
            best_est = est

    if best_est is None or best_img is None:
        print("没有任何图像成功估计外参。", file=sys.stderr)
        return 1

    patch = best_est["patch"]
    data.update(patch)
    data["auto_selected_extrinsic_image"] = str(best_img.resolve())
    data["auto_extrinsic_reproj_mean_px"] = float(best_est["mean_reproj_px"])
    data["auto_extrinsic_reproj_max_px"] = float(best_est["max_reproj_px"])

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    frag_path = out_dir / "extrinsic_best_fragment.json"
    frag_path.write_text(
        json.dumps(patch, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"最佳外参图: {_abs_or_rel(best_img)}")
    print(f"patch 已写入: {_abs_or_rel(frag_path)}")

    try:
        import cv2

        gray = best_est["gray"]
        board_size = best_est["board_size"]
        img_pts = best_est["img_pts"]
        if gray.ndim == 2:
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            bgr = gray.copy()
        corn_png = out_dir / "board_corners_best.png"
        cv2.drawChessboardCorners(bgr, board_size, img_pts, True)
        cv2.imwrite(str(corn_png), bgr)
        print(f"角点可视化: {_abs_or_rel(corn_png)}")
    except ImportError:
        pass

    pts_path = Path(args.points)
    if not pts_path.is_file():
        print("触点总表不存在:", pts_path.resolve(), file=sys.stderr)
        return 1

    board = cfg.get("board_gp100", {})
    sq = (
        float(args.square_mm)
        if getattr(args, "square_mm", None) is not None
        else float(board.get("square_mm", 6.0))
    )

    arm_blk = data.setdefault("arm", {})
    zp1 = (
        float(args.joint1_zero_pulse)
        if getattr(args, "joint1_zero_pulse", None) is not None
        else float(arm_blk.get("joint1_zero_pulse", 0))
    )
    zp2 = (
        float(args.joint2_zero_pulse)
        if getattr(args, "joint2_zero_pulse", None) is not None
        else float(arm_blk.get("joint2_zero_pulse", 0))
    )
    s1 = (
        float(args.joint1_sign)
        if getattr(args, "joint1_sign", None) is not None
        else float(arm_blk.get("joint1_sign", 1.0))
    )
    s2 = (
        float(args.joint2_sign)
        if getattr(args, "joint2_sign", None) is not None
        else float(arm_blk.get("joint2_sign", 1.0))
    )
    arm_blk["joint1_sign"] = s1
    arm_blk["joint2_sign"] = s2
    arm_blk["joint1_zero_pulse"] = int(zp1)
    arm_blk["joint2_zero_pulse"] = int(zp2)

    pulse_per_rev = float(args.pulse_per_rev)
    ratio = float(args.motor_ratio)

    trays: list[list[float]] = []
    arms: list[list[float]] = []
    joint_rows: list[tuple[float, float, float, float]] = []

    for row in read_csv_rows(pts_path):
        if _csv_role(row) != "fit":
            continue
        xa = _pick_float(row, ("x_arm_mm", "x_arm"))
        ya = _pick_float(row, ("y_arm_mm", "y_arm"))
        za = _pick_float(row, ("z_arm_mm", "z_arm"))
        if za is None:
            za = 0.0
        if xa is None or ya is None:
            print("跳过 fit 行（缺机械臂坐标）:", row)
            continue
        tx = _pick_float(row, ("tray_x_mm",))
        ty = _pick_float(row, ("tray_y_mm",))
        ci = _pick_float(row, ("corner_col_idx", "col_idx"))
        ri = _pick_float(row, ("corner_row_idx", "row_idx"))
        if tx is not None and ty is not None:
            tz = _pick_float(row, ("tray_z_mm",)) or 0.0
            trays.append([tx, ty, tz])
        elif ci is not None and ri is not None:
            trays.append([ci * sq, ri * sq, 0.0])
        else:
            print("跳过 fit 行（无 tray 坐标或角点号）:", row)
            continue
        arms.append([xa, ya, za])

        p1 = _pick_float(row, ("servo1_abs_pulse", "pulse_motor1_abs"))
        p2 = _pick_float(row, ("servo2_abs_pulse", "pulse_motor2_abs"))
        if p1 is not None and p2 is not None:
            m1, m2 = pulses_to_meas_joint_deg(
                p1, p2, zp1, zp2, s1, s2, pulse_per_rev, ratio
            )
            joint_rows.append((float(xa), float(ya), m1, m2))

    if len(trays) < 3:
        print("fit 行有效触点少于 3 个", file=sys.stderr)
        return 1

    aa = np.asarray(trays, dtype=np.float64)
    bb = np.asarray(arms, dtype=np.float64)
    r_m, t_v = rigid_transform_3d(aa, bb)
    tea = to_homogeneous(r_m, t_v)
    pred = (r_m @ aa.T).T + t_v.reshape(1, 3)
    diff = bb - pred
    dist = np.linalg.norm(diff, axis=1)
    rms_pts = float(np.sqrt(np.mean(dist**2)))
    max_pts = float(np.max(dist))
    print(f"fit-tray: {len(trays)} 组触点; square_mm={sq}")
    print(f"刚体对齐残差 RMS={rms_pts:.4f} mm, max={max_pts:.4f} mm")
    for i, (d_i, di) in enumerate(zip(diff, dist)):
        print(
            f"  点{i + 1}: Δarm=({d_i[0]:+.3f},{d_i[1]:+.3f},{d_i[2]:+.3f}) mm, |Δ|={di:.3f} mm"
        )

    data["T_tray_to_arm"] = tea.tolist()
    data.setdefault("quality", {}).setdefault(
        "max_error_mm", float(cfg.get("quality", {}).get("max_error_mm", 3.0))
    )
    save_results_json(res_path, data)
    print("已写入 T_tray_to_arm →", _abs_or_rel(res_path))

    jpath = out_dir / "joint_observations_one_table.csv"
    if joint_rows:
        with jpath.open("w", encoding="utf-8", newline="") as jf:
            w = csv.writer(jf)
            w.writerow(
                ["xw_mm", "yw_mm", "meas_theta1_deg", "meas_theta2_deg", "note"]
            )
            for xw, yw, m1, m2 in joint_rows:
                w.writerow([xw, yw, m1, m2, "one-table"])
        print(
            f"关节观测（由脉冲换算）: {_abs_or_rel(jpath)} 共 {len(joint_rows)} 行"
        )
        fa = argparse.Namespace()
        fa.csv = jpath
        fa.results = args.results
        fa.config = args.config
        fa.l1 = getattr(args, "l1", None)
        fa.l4 = getattr(args, "l4", None)
        fa.joint1_sign = s1
        fa.joint2_sign = s2
        fa.scipy = bool(getattr(args, "scipy", False))
        rc_fa = cmd_fit_arm(fa)
        if rc_fa != 0:
            return rc_fa
        lim = float(getattr(args, "fit_arm_max_rms_deg", 35.0))
        if lim > 0.0:
            data_chk = load_results_json(res_path)
            rms_j = float((data_chk.get("fit_arm") or {}).get("rms_joint_deg", 0.0))
            if rms_j > lim:
                print(
                    f"中止: fit-arm 关节残差 RMS={rms_j:.4f}° 超过阈值 {lim}° "
                    "（检查触点、脉冲零点 joint*_zero_pulse、joint*_sign；"
                    "或放宽 --fit-arm-max-rms-deg / 设为 0 关闭检查）。",
                    file=sys.stderr,
                )
                return 1
    else:
        print(
            "提示: fit 行未提供伺服脉冲列，跳过 fit-arm。",
            file=sys.stderr,
        )

    if bool(getattr(args, "skip_verify", False)):
        _hint_next(
            "写入 F407: python apply_calibration_to_f407.py --config calibration_config.json "
            f"--results {res_path.name} --apply"
        )
        return 0

    data = load_results_json(res_path)
    _ensure_camera_in_results(data, cfg)
    cam = data.get("camera", {})
    fx = float(cam.get("fx", 608.356))
    fy = float(cam.get("fy", 607.887))
    cx = float(cam.get("cx", 323.566))
    cy = float(cam.get("cy", 246.759))
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = np.array(cam.get("D", [0.0] * 5), dtype=np.float64).reshape(-1, 1)
    tc = data.get("tray_from_cam", {})
    r_tc = np.asarray(tc.get("R", np.eye(3).tolist()), dtype=np.float64)
    t_tc = np.asarray(tc.get("t_mm", [0.0, 0.0, 0.0]), dtype=np.float64)
    t4 = np.asarray(data.get("T_tray_to_arm", np.eye(4)), dtype=np.float64)
    max_acc = float(data.get("quality", {}).get("max_error_mm", 3.0))

    v_errs: list[float] = []
    v_modes: list[str] = []

    for row in read_csv_rows(pts_path):
        if _csv_role(row) != "verify":
            continue
        xa = _pick_float(row, ("x_arm_truth_mm", "x_arm_mm", "x_arm"))
        ya = _pick_float(row, ("y_arm_truth_mm", "y_arm_mm", "y_arm"))
        if xa is None or ya is None:
            continue
        tx = _pick_float(row, ("tray_x_mm",))
        ty = _pick_float(row, ("tray_y_mm",))
        ci = _pick_float(row, ("corner_col_idx", "col_idx"))
        ri = _pick_float(row, ("corner_row_idx", "row_idx"))
        if tx is not None and ty is not None:
            tz = _pick_float(row, ("tray_z_mm",)) or 0.0
            txyz = (tx, ty, tz)
        elif ci is not None and ri is not None:
            txyz = (ci * sq, ri * sq, 0.0)
        else:
            print("跳过 verify 行（无 tray 坐标）:", row)
            continue

        uu = _pick_float(row, ("u_px", "u"))
        vv = _pick_float(row, ("v_px", "v"))
        zz = _pick_float(row, ("z_mm", "z"))

        if uu is not None and vv is not None and zz is not None:
            pr = pipeline_arm_xy(uu, vv, zz, k, dist, r_tc, t_tc, t4)
            ee = float(np.linalg.norm(pr[:2] - np.array([xa, ya])))
            v_errs.append(ee)
            v_modes.append("pixel")
            print(
                f"[verify px] u,v,z={uu},{vv},{zz} -> pred=({pr[0]:.2f},{pr[1]:.2f}) "
                f"truth=({xa},{ya}) err={ee:.3f} mm"
            )
        else:
            pc = tray_mm_to_cam_xyz(np.array(txyz, dtype=np.float64), r_tc, t_tc)
            zc = float(pc[2])
            used_backproj = False
            if zc > 0.5:
                try:
                    u_syn, v_syn = cam_xyz_to_uv_distorted(pc, k, dist)
                    pr = pipeline_arm_xy(
                        u_syn, v_syn, zc, k, dist, r_tc, t_tc, t4
                    )
                    ee = float(np.linalg.norm(pr[:2] - np.array([xa, ya])))
                    v_errs.append(ee)
                    v_modes.append("backproj_pixel")
                    used_backproj = True
                    print(
                        f"[verify backproj] tray=({txyz[0]:.3f},{txyz[1]:.3f},{txyz[2]:.3f}) "
                        f"synth_u,v,z_c={u_syn:.2f},{v_syn:.2f},{zc:.3f} "
                        f"-> pred=({pr[0]:.2f},{pr[1]:.2f}) truth=({xa},{ya}) err={ee:.3f} mm"
                    )
                except ImportError:
                    pass
            if not used_backproj:
                ph = np.array([txyz[0], txyz[1], txyz[2], 1.0], dtype=np.float64)
                pa = t4 @ ph
                ee = float(np.linalg.norm(pa[:2] - np.array([xa, ya])))
                v_errs.append(ee)
                v_modes.append("tray_xy")
                note = ""
                if zc <= 0.5:
                    note = "（相机坐标 Z≤0.5mm，退回 tray→arm）"
                print(
                    f"[verify T] tray=({txyz[0]:.3f},{txyz[1]:.3f},{txyz[2]:.3f}) "
                    f"-> pred_arm=({pa[0]:.2f},{pa[1]:.2f}) truth=({xa},{ya}) "
                    f"err={ee:.3f} mm{note}"
                )

    if not v_errs:
        print("未找到 verify 行，跳过验收（可用 --skip-verify 明确跳过）。")
    else:
        mean_e = float(np.mean(v_errs))
        max_e = float(np.max(v_errs))
        print(
            f"one-table 验收 mean_mm={mean_e:.4f} max_mm={max_e:.4f} "
            f"accept_max_mm={max_acc} modes={','.join(sorted(set(v_modes)))}"
        )
        qc = data.setdefault("quality", {})
        qc["verify_mean_mm"] = mean_e
        qc["verify_max_mm"] = max_e
        qc["one_table_verify_modes"] = sorted(set(v_modes))
        cov = data.setdefault("conveyor", {})
        if isinstance(cfg.get("conveyor"), dict):
            cov.update(cfg["conveyor"])
        cov.setdefault("mm_per_pulse", 0.0)
        cov.setdefault("delta_pulse_for_test", 0)
        save_results_json(res_path, data)
        print("已更新 quality.verify_* →", _abs_or_rel(res_path))
        if max_e > max_acc:
            print(
                f"判定: FAIL（最大误差 {max_e} mm 超过阈值 {max_acc} mm）",
                file=sys.stderr,
            )
            _hint_next("排查外参/触点/脉冲零点后再运行 one-table")
            return 2
        print(f"判定: PASS（最大误差不超过阈值 {max_acc} mm）")

    _hint_next(
        "写入 F407: python apply_calibration_to_f407.py --config calibration_config.json "
        f"--results {res_path.name} --apply"
    )
    return 0


def cmd_apply_f407(args: argparse.Namespace) -> int:
    _sep("apply-f407：生成 F407 app_calibration_params.h")
    print("结果 JSON:", _abs_or_rel(Path(args.results)))
    if args.config.is_file():
        print("配置:", _abs_or_rel(Path(args.config)))
    exe = [
        sys.executable,
        str(CAL_DIR / "apply_calibration_to_f407.py"),
        "--results",
        str(args.results),
    ]
    if args.config.is_file():
        exe += ["--config", str(args.config)]
    if getattr(args, "header", None):
        exe += ["--header", str(args.header)]
    if getattr(args, "apply", False):
        exe.append("--apply")
    if getattr(args, "force", False):
        exe.append("--force")
    return subprocess.call(exe)


def main() -> int:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "例：合并外参后写入 Keil 头文件\n"
            "  python calibrate_device.py merge-patch output/ex_frag.json \\\n"
            "      apply-f407 --apply\n\n"
            "一表流： python calibrate_device.py one-table --points data/calibration_points.csv \\\n"
            "      --images images/extrinsic\n\n"
            "子命令也可用： python calibrate_device.py solve-board --image …"
        ),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=CAL_DIR / "calibration_config.json",
        help="相机/GP100 路径等；若无此文件则用 results 内含参数",
    )
    parser.add_argument(
        "--results",
        type=Path,
        default=CAL_DIR / "calibration_results.json",
        help="读写标定结果的 JSON",
    )

    sub = parser.add_subparsers(dest="cmd", required=True)

    p1 = sub.add_parser(
        "solve-board",
        help="GP100 棋盘外参（需 pip install opencv-python）",
    )
    p1.add_argument("--image", type=Path, required=True)
    p1.add_argument("--save-corners", type=Path)
    p1.add_argument("--json-out", type=Path)

    p2 = sub.add_parser("merge-patch", help="补丁 JSON merge 至 --results")
    p2.add_argument("patch_json", nargs="+", type=Path)

    p3 = sub.add_parser(
        "fit-tray",
        help="CSV 触点求 T_tray_to_arm",
    )
    p3.add_argument("--csv", type=Path, required=True)
    p3.add_argument("--square-mm", type=float)

    p4 = sub.add_parser(
        "fit-arm",
        help="CSV 观测：对称五杆逆解 + 网格搜索 joint 零点；可选 scipy 精修",
    )
    p4.add_argument("--csv", type=Path, required=True)
    p4.add_argument(
        "--l1",
        type=float,
        help="覆盖左主动杆长 mm（默认用 results.arm.L_active_left_mm / L1_mm）",
    )
    p4.add_argument(
        "--l4",
        type=float,
        help="覆盖右主动杆长 mm（默认用 results.arm.L_active_right_mm / L4_mm）",
    )
    p4.add_argument("--joint1-sign", type=float, default=1.0)
    p4.add_argument("--joint2-sign", type=float, default=1.0)
    p4.add_argument(
        "--scipy",
        action="store_true",
        help="网格初值后用 scipy.optimize.least_squares 精修（需 pip install scipy）",
    )

    p5 = sub.add_parser(
        "verify",
        help="CSV 像素+深度复测（CSV 文件必须存在且含有效数据行）",
    )
    p5.add_argument(
        "--csv",
        type=Path,
        required=True,
        help="复测点 CSV 路径（须存在；列 u_px,v_px,z_mm,x_arm_truth_mm,y_arm_truth_mm）",
    )

    p_ot = sub.add_parser(
        "one-table",
        help="最短链：多张 GP100 自动择优外参 + 触点总表 fit-tray / fit-arm / verify",
    )
    p_ot.add_argument(
        "--points",
        type=Path,
        default=CAL_DIR / "data" / "calibration_points.csv",
        help="触点总表 CSV（fit / verify 行）",
    )
    p_ot.add_argument(
        "--images-dir",
        "--images",
        type=Path,
        dest="images_dir",
        default=CAL_DIR / "images" / "extrinsic",
        help="GP100 外参照片目录（jpg/jpeg/png）；等价别名 --images",
    )
    p_ot.add_argument(
        "--output-dir",
        type=Path,
        default=CAL_DIR / "output",
        help="输出最佳 patch、角点图、joint_observations_one_table.csv",
    )
    p_ot.add_argument(
        "--square-mm",
        type=float,
        default=None,
        help="覆盖棋盘格距 mm（默认读 calibration_config.json board_gp100.square_mm）",
    )
    p_ot.add_argument(
        "--pulse-per-rev",
        type=float,
        default=DEFAULT_SERVO_PULSE_PER_REV,
        help="伺服电机每物理圈脉冲数（默认 10000）",
    )
    p_ot.add_argument(
        "--motor-ratio",
        type=float,
        default=DEFAULT_MOTOR_TO_JOINT_RATIO,
        help="电机角:关节角减速倍率（默认 30，与固件 CFG_ICHUANDONG_RATIO 一致）",
    )
    p_ot.add_argument(
        "--joint1-zero-pulse",
        type=float,
        default=None,
        help="关节 1 零点脉冲偏置（默认 results.arm.joint1_zero_pulse 或 0）",
    )
    p_ot.add_argument(
        "--joint2-zero-pulse",
        type=float,
        default=None,
        help="关节 2 零点脉冲偏置（默认 results.arm.joint2_zero_pulse 或 0）",
    )
    p_ot.add_argument(
        "--joint1-sign",
        type=float,
        default=None,
        help="关节 1 方向符号（默认 results.arm.joint1_sign 或 +1）",
    )
    p_ot.add_argument(
        "--joint2-sign",
        type=float,
        default=None,
        help="关节 2 方向符号（默认 results.arm.joint2_sign 或 +1）",
    )
    p_ot.add_argument(
        "--l1",
        type=float,
        default=None,
        help="覆盖 fit-arm 左主动杆长 mm",
    )
    p_ot.add_argument(
        "--l4",
        type=float,
        default=None,
        help="覆盖 fit-arm 右主动杆长 mm",
    )
    p_ot.add_argument(
        "--scipy",
        action="store_true",
        help="fit-arm 启用 scipy 最小二乘精修（需 pip install scipy）",
    )
    p_ot.add_argument(
        "--skip-verify",
        action="store_true",
        help="跳过总表中 role=verify 的验收",
    )
    p_ot.add_argument(
        "--fit-arm-max-rms-deg",
        type=float,
        default=35.0,
        help="fit-arm 完成后若 rms_joint_deg 超过该值则中止（≤0 关闭检查；默认 35）",
    )

    p6 = sub.add_parser("apply-f407", help="生成 app_calibration_params.h")
    p6.add_argument("--header", type=Path)
    p6.add_argument("--apply", action="store_true")
    p6.add_argument(
        "--force",
        action="store_true",
        help="传递给 apply_calibration_to_f407.py：跳过 JSON 结构校验（慎用）",
    )

    args = parser.parse_args()
    hm: dict[str, object] = {
        "solve-board": cmd_solve_board,
        "merge-patch": cmd_merge_patch,
        "fit-tray": cmd_fit_tray,
        "fit-arm": cmd_fit_arm,
        "verify": cmd_verify,
        "one-table": cmd_one_table,
        "apply-f407": cmd_apply_f407,
    }
    handler = hm.get(args.cmd)
    if not callable(handler):
        return 1
    return int(handler(args))


if __name__ == "__main__":
    sys.exit(main())
