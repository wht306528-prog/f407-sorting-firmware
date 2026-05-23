#!/usr/bin/env python3
"""读取 calibration_config.json（可选）与 calibration_results.json，生成 F407 app_calibration_params.h。

用法:
  python apply_calibration_to_f407.py --config calibration_config.json \\
      --results calibration_results.json --header ../F407/.../app_calibration_params.h --apply
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def load_merged(config_path: Path | None, results_path: Path) -> dict:
    data: dict = {}
    if config_path and config_path.is_file():
        with config_path.open("r", encoding="utf-8") as f:
            cfg = json.load(f)
        c = cfg.get("camera", {})
        if "K" in c:
            k = c["K"]
            data["camera"] = {
                "fx": k[0][0],
                "fy": k[1][1],
                "cx": k[0][2],
                "cy": k[1][2],
                "width": c.get("width", 640),
                "height": c.get("height", 480),
                "D": c.get("D", [0.0] * 5),
            }
        if "paths" in cfg and "f407_header_out" in cfg["paths"]:
            data["_paths_meta"] = {"f407_header_out": cfg["paths"]["f407_header_out"]}
        if isinstance(cfg.get("arm_nominal_mm"), dict):
            data.setdefault("arm_nominal_mm", cfg["arm_nominal_mm"])

    with results_path.open("r", encoding="utf-8") as f:
        res = json.load(f)
    strip_keys(res)
    if "camera" in res:
        data["camera"] = res["camera"]
    data.update(res)
    return data


def strip_keys(obj: dict) -> None:
    """原地删除 JSON 中非数据键以便 merge。"""
    dels = [k for k in obj if k.startswith("_") or k.endswith("_comment")]
    for k in dels:
        del obj[k]


def f32(x: float) -> str:
    return f"{float(x):.8f}f"


def i64_pulse(x: int | float) -> str:
    return f"{int(round(float(x)))}L"


def resolve_arm_geometry(data: dict) -> None:
    """补全五杆与零位脉冲缺省，兼容仅含 L1_mm/L4_mm 的旧 results。"""
    nom = data.get("arm_nominal_mm")
    if not isinstance(nom, dict):
        nom = {}
    l_pass_fallback = float(nom.get("L2_passive", 700.0))
    arm = data.setdefault("arm", {})
    if not isinstance(arm, dict):
        data["arm"] = {}
        arm = data["arm"]

    l1_legacy = float(arm.get("L1_mm", 600.0))
    l4_legacy = float(arm.get("L4_mm", 600.0))
    arm.setdefault("ax_mm", float(arm.get("ax_mm", -250.0)))
    arm.setdefault("ay_mm", float(arm.get("ay_mm", 0.0)))
    arm.setdefault("bx_mm", float(arm.get("bx_mm", 250.0)))
    arm.setdefault("by_mm", float(arm.get("by_mm", 0.0)))

    arm.setdefault("L_active_left_mm", float(arm.get("L_active_left_mm", l1_legacy)))
    arm.setdefault("L_active_right_mm", float(arm.get("L_active_right_mm", l1_legacy)))
    arm.setdefault("L_passive_left_mm", float(arm.get("L_passive_left_mm", l_pass_fallback)))
    arm.setdefault("L_passive_right_mm", float(arm.get("L_passive_right_mm", l_pass_fallback)))

    arm.setdefault("L1_mm", l1_legacy)
    arm.setdefault("L4_mm", l4_legacy)

    arm.setdefault("joint1_zero_deg", float(arm.get("joint1_zero_deg", 0.0)))
    arm.setdefault("joint2_zero_deg", float(arm.get("joint2_zero_deg", 0.0)))
    arm.setdefault("joint1_sign", float(arm.get("joint1_sign", 1.0)))
    arm.setdefault("joint2_sign", float(arm.get("joint2_sign", 1.0)))
    arm.setdefault("joint1_zero_pulse", int(arm.get("joint1_zero_pulse", 0)))
    arm.setdefault("joint2_zero_pulse", int(arm.get("joint2_zero_pulse", 0)))


def _is_num_matrix(m: object, rows: int, cols: int) -> bool:
    if not isinstance(m, list) or len(m) != rows:
        return False
    for row in m:
        if not isinstance(row, list) or len(row) != cols:
            return False
        for x in row:
            if not isinstance(x, (int, float)) or isinstance(x, bool):
                return False
    return True


def _almost_identity_r33(r: list[list]) -> bool:
    for i in range(3):
        for j in range(3):
            v = float(r[i][j])
            if i == j:
                if abs(v - 1.0) > 1e-5:
                    return False
            else:
                if abs(v) > 1e-5:
                    return False
    return True


def validate_calibration_data(data: dict) -> tuple[list[str], list[str]]:
    """校验合并后的数据结构，避免因 emit 默认值写入固件。返回 (errors, warnings)。"""
    errs: list[str] = []
    warns: list[str] = []

    cam = data.get("camera")
    if not isinstance(cam, dict):
        errs.append("缺少 camera（合并后无相机 fx/fy/cx/cy/D）")
    else:
        for k in ("fx", "fy", "cx", "cy"):
            if k not in cam:
                errs.append(f"camera 缺少必填字段 '{k}'")
        dcoef = cam.get("D")
        if not isinstance(dcoef, list) or len(dcoef) < 5:
            errs.append("camera.D 必须是长度至少 5 的数组 [k1,k2,p1,p2,k3]")

    tc = data.get("tray_from_cam")
    if not isinstance(tc, dict):
        errs.append("缺少 tray_from_cam（需 merge 棋盘外参或填写 tray_from_cam）")
    else:
        r_tc = tc.get("R")
        t_tc = tc.get("t_mm")
        if not _is_num_matrix(r_tc, 3, 3):
            errs.append("tray_from_cam.R 必须是 3x3 数值矩阵")
        if not isinstance(t_tc, list) or len(t_tc) != 3:
            errs.append("tray_from_cam.t_mm 必须是长度 3 的数组（毫米）")
        elif _is_num_matrix(r_tc, 3, 3) and _almost_identity_r33(r_tc):
            if all(abs(float(x)) < 1e-4 for x in t_tc):
                warns.append(
                    "tray_from_cam 近似单位阵和平移为零：可能是模板/占位，请确认已完成真实外参标定。"
                )

    t4 = data.get("T_tray_to_arm")
    if not _is_num_matrix(t4, 4, 4):
        errs.append("T_tray_to_arm 必须是 4x4 数值矩阵")

    arm = data.get("arm")
    if not isinstance(arm, dict):
        errs.append("缺少 arm（请运行 fit-arm 或在 JSON 内填写连杆/零点）")
    else:
        for lk in (
            "L_active_left_mm",
            "L_passive_left_mm",
            "L_active_right_mm",
            "L_passive_right_mm",
        ):
            try:
                if float(arm[lk]) <= 1e-6:
                    errs.append(f"arm.{lk} 必须为正数（毫米）")
            except (TypeError, ValueError):
                errs.append(f"arm.{lk} 非数值")

        for nk in ("joint1_sign", "joint2_sign"):
            try:
                if abs(float(arm[nk])) < 1e-9:
                    errs.append(f"arm.{nk} 不能为 0")
            except (TypeError, ValueError):
                errs.append(f"arm.{nk} 非数值")

    return errs, warns


def emit_header(data: dict) -> str:
    cam = data.get("camera", {})
    fx = cam.get("fx", 608.356)
    fy = cam.get("fy", 607.887)
    cx = cam.get("cx", 323.566)
    cy = cam.get("cy", 246.759)
    d = cam.get("D", [0.0, 0.0, 0.0, 0.0, 0.0])
    while len(d) < 5:
        d.append(0.0)

    tc = data.get("tray_from_cam", {})
    r = tc.get("R", [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    t = tc.get("t_mm", [0.0, 0.0, 0.0])
    t4 = data.get(
        "T_tray_to_arm",
        [[1, 0, 0, 250], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    )
    arm = data.get("arm", {})
    ax = float(arm.get("ax_mm", -250.0))
    ay = float(arm.get("ay_mm", 0.0))
    bx = float(arm.get("bx_mm", 250.0))
    by = float(arm.get("by_mm", 0.0))
    lal = float(arm.get("L_active_left_mm", 600.0))
    pal = float(arm.get("L_passive_left_mm", 700.0))
    lar = float(arm.get("L_active_right_mm", 600.0))
    par = float(arm.get("L_passive_right_mm", 700.0))
    jz1 = float(arm.get("joint1_zero_deg", 0.0))
    jz2 = float(arm.get("joint2_zero_deg", 0.0))
    s1 = float(arm.get("joint1_sign", 1.0))
    s2 = float(arm.get("joint2_sign", 1.0))
    zp1 = int(arm.get("joint1_zero_pulse", 0))
    zp2 = int(arm.get("joint2_zero_pulse", 0))
    l1 = float(arm.get("L1_mm", lal))
    l4 = float(arm.get("L4_mm", lar))
    cov = data.get("conveyor", {})
    mmpp = cov.get("mm_per_pulse", 0.0)
    dpulse = int(cov.get("delta_pulse_for_test", 0))
    q = data.get("quality", {})
    max_e = q.get("max_error_mm", 3.0)

    lines = [
        "/**",
        " * app_calibration_params.h — 由 Calibration/apply_calibration_to_f407.py 自动生成，请勿手改。",
        " * 维护说明见 F407/docs/F407_维护说明.txt 相机章节。",
        " * 重新标定: python apply_calibration_to_f407.py --config calibration_config.json --apply",
        " */",
        "#ifndef APP_CALIBRATION_PARAMS_H",
        "#define APP_CALIBRATION_PARAMS_H",
        "",
        f"#define CALIB_CAM_FX_PX   ({f32(fx)})",
        f"#define CALIB_CAM_FY_PX   ({f32(fy)})",
        f"#define CALIB_CAM_CX_PX   ({f32(cx)})",
        f"#define CALIB_CAM_CY_PX   ({f32(cy)})",
        f"#define CALIB_DIST_K1     ({f32(d[0])})",
        f"#define CALIB_DIST_K2     ({f32(d[1])})",
        f"#define CALIB_DIST_P1     ({f32(d[2])})",
        f"#define CALIB_DIST_P2     ({f32(d[3])})",
        f"#define CALIB_DIST_K3     ({f32(d[4])})",
        "",
    ]
    for i in range(3):
        for j in range(3):
            lines.append(
                f"#define CALIB_R_TRAY_FROM_CAM_{i}{j} ({f32(r[i][j])})"
            )
    lines.append(f"#define CALIB_T_TRAY_FROM_CAM_X_MM ({f32(t[0])})")
    lines.append(f"#define CALIB_T_TRAY_FROM_CAM_Y_MM ({f32(t[1])})")
    lines.append(f"#define CALIB_T_TRAY_FROM_CAM_Z_MM ({f32(t[2])})")
    lines.append("")
    for i in range(4):
        for j in range(4):
            lines.append(
                f"#define CALIB_T_TRAY_TO_ARM_{i}{j} ({f32(t4[i][j])})"
            )
    lines += [
        "",
        "/* 对称五连杆（臂平面 Xw,Yw）；轴心与杆长由标定 / fit-arm 写入 */",
        f"#define CALIB_ARM_AX_MM ({f32(ax)})",
        f"#define CALIB_ARM_AY_MM ({f32(ay)})",
        f"#define CALIB_ARM_BX_MM ({f32(bx)})",
        f"#define CALIB_ARM_BY_MM ({f32(by)})",
        f"#define CALIB_ARM_L_ACTIVE_LEFT_MM ({f32(lal)})",
        f"#define CALIB_ARM_L_PASSIVE_LEFT_MM ({f32(pal)})",
        f"#define CALIB_ARM_L_ACTIVE_RIGHT_MM ({f32(lar)})",
        f"#define CALIB_ARM_L_PASSIVE_RIGHT_MM ({f32(par)})",
        f"#define CALIB_JOINT1_ZERO_PULSE ({i64_pulse(zp1)})",
        f"#define CALIB_JOINT2_ZERO_PULSE ({i64_pulse(zp2)})",
        "",
        "/* 脚本遗留字段（两杆占位），勿删以保持 JSON 兼容 */",
        f"#define CALIB_ARM_L1_MM ({f32(l1)})",
        f"#define CALIB_ARM_L4_MM ({f32(l4)})",
        "",
        f"#define CALIB_JOINT1_ZERO_DEG ({f32(jz1)})",
        f"#define CALIB_JOINT2_ZERO_DEG ({f32(jz2)})",
        f"#define CALIB_JOINT1_SIGN ({f32(s1)})",
        f"#define CALIB_JOINT2_SIGN ({f32(s2)})",
        f"#define CALIB_MAX_ERROR_MM_ACCEPT ({f32(max_e)})",
        "",
        "/* 传送带占位（当前测试阶段常为 0；运动补偿逻辑可在 app_conveyor / 上层启用） */",
        f"#define CALIB_CONV_MM_PER_PULSE ({f32(mmpp)})",
        f"#define CALIB_CONV_DELTA_PULSE ({int(dpulse)})",
        "",
        "#endif /* APP_CALIBRATION_PARAMS_H */",
        "",
    ]
    return "\n".join(lines)


def print_calibration_summary(data: dict, header_out: Path, results_src: Path) -> None:
    """关键参数摘要写入 stderr，便于人机核对且不干扰 stdout 管道。"""
    cam = data.get("camera", {})
    fx = cam.get("fx", float("nan"))
    fy = cam.get("fy", float("nan"))
    cx = cam.get("cx", float("nan"))
    cy = cam.get("cy", float("nan"))
    tc = data.get("tray_from_cam", {})
    t_mm = tc.get("t_mm", [0.0, 0.0, 0.0])
    arm = data.get("arm", {})
    q = data.get("quality", {})
    print("[标定写入摘要]", file=sys.stderr)
    print(f"  results: {results_src.resolve()}", file=sys.stderr)
    print(f"  头文件: {header_out.resolve()}", file=sys.stderr)
    print(
        f"  相机 fx={fx:.4f} fy={float(fy):.4f} cx={float(cx):.4f} cy={float(cy):.4f}",
        file=sys.stderr,
    )
    print(
        f"  tray t_mm=({float(t_mm[0]):.3f},{float(t_mm[1]):.3f},{float(t_mm[2]):.3f}) mm",
        file=sys.stderr,
    )
    print(
        "  五杆 A=("
        f"{arm.get('ax_mm')},{arm.get('ay_mm')}) B=({arm.get('bx_mm')},{arm.get('by_mm')})",
        file=sys.stderr,
    )
    print(
        "  Lal/Lpl/Lar/Lpr="
        f"{arm.get('L_active_left_mm')}/{arm.get('L_passive_left_mm')}/"
        f"{arm.get('L_active_right_mm')}/{arm.get('L_passive_right_mm')} mm",
        file=sys.stderr,
    )
    print(
        f"  零位 deg=({arm.get('joint1_zero_deg')},{arm.get('joint2_zero_deg')}) "
        f"pulse=({arm.get('joint1_zero_pulse')},{arm.get('joint2_zero_pulse')})",
        file=sys.stderr,
    )
    vm = q.get("verify_mean_mm")
    vx = q.get("verify_max_mm")
    if vm is not None and vx is not None:
        ma = q.get("max_error_mm", 3.0)
        print(
            f"  复测 quality: mean={float(vm):.4f} mm max={float(vx):.4f} mm (阈值 accept {ma})",
            file=sys.stderr,
        )
        if float(vx) > float(ma):
            print(
                "  警告: 复测 max 已超过验收阈值，生成头文件前请再次确认。",
                file=sys.stderr,
            )


def resolve_header_path(args_header: Path | None, merged: dict) -> Path:
    if args_header is not None:
        return args_header
    meta = merged.get("_paths_meta", {})
    rel = meta.get("f407_header_out")
    if rel:
        base = Path(__file__).resolve().parent
        return (base / rel).resolve()
    return (
        Path(__file__).resolve().parents[1]
        / "F407"
        / "Project"
        / "User"
        / "calibration"
        / "app_calibration_params.h"
    )


def main() -> int:
    ap = argparse.ArgumentParser(
        description="由 calibration_results.json 生成 F407 app_calibration_params.h",
    )
    ap.add_argument("--config", type=Path, default=None)
    ap.add_argument(
        "--results",
        type=Path,
        default=Path(__file__).resolve().parent / "calibration_results.json",
    )
    ap.add_argument("--header", type=Path, default=None)
    ap.add_argument(
        "--apply",
        action="store_true",
        help="写入头文件；默认仅打印头文件到 stdout",
    )
    ap.add_argument(
        "--force",
        action="store_true",
        help="跳过结构校验（仅当你确认 JSON 已完整时使用，避免误写入）",
    )
    args = ap.parse_args()

    if not args.results.is_file():
        print("找不到 results:", args.results, file=sys.stderr)
        return 1
    data = load_merged(args.config, args.results)
    resolve_arm_geometry(data)
    errs, warns = validate_calibration_data(data)
    for w in warns:
        print("警告:", w, file=sys.stderr)
    if errs and not args.force:
        for e in errs:
            print("错误:", e, file=sys.stderr)
        print(
            "校验未通过：已中止。若确信数据无误可加 --force（不推荐）。",
            file=sys.stderr,
        )
        return 1
    if errs and args.force:
        print("警告: --force 已跳过以下校验错误:", file=sys.stderr)
        for e in errs:
            print("  -", e, file=sys.stderr)

    text = emit_header(data)
    header_out = resolve_header_path(args.header, data)
    print_calibration_summary(data, header_out, args.results)

    if args.apply:
        header_out.parent.mkdir(parents=True, exist_ok=True)
        header_out.write_text(text, encoding="utf-8")
        print("已写入", header_out.resolve())
        print(
            "\n>>> 下一步: 在 Keil 中编译 F407 工程并烧录，现场再跑一轮机械复测。",
            file=sys.stderr,
        )
    else:
        print(
            "(完整头文件见下方 stdout；校验摘要见 stderr。可使用 > file.h 重定向)",
            file=sys.stderr,
        )
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
