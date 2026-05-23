# Calibration

照片与完整理论说明见 **`安装后标定流程.txt`**。现场最短路径：**拍外参图 + 填 `data/calibration_points.csv` + `one-table`**（见文内 ★★；模板 `data/calibration_points.example.csv`）。分步子命令（fit-tray / fit-arm / verify）仅供参考或排错，相关 CSV **需自建**。

## 依赖

```bash
pip install numpy opencv-python
```

`opencv-python` 仅在 **`solve-board`** 时需要。**`fit-arm --scipy`** 另需：`pip install scipy`。

## 本目录只有两个 Python 程序

| 程序 | 作用 |
|------|------|
| **`calibrate_device.py`** | 标定流水线：**`one-table`**（推荐：多图外参择优 + 总表 fit-tray / fit-arm / verify）；亦可分步：`solve-board`、`merge-patch`、`fit-tray`、**`fit-arm`**（可选 **`--scipy`**）、`verify`。结果写入 `calibration_results.json`。子命令 **`apply-f407`** 仅转调下方脚本。 |
| **`apply_calibration_to_f407.py`** | 读取 `calibration_results.json`（可选 `calibration_config.json` 带出 `camera` / **`arm_nominal_mm`**）。自动补全五杆缺省后与固件对齐；校验后生成 **`CALIB_ARM_*` / `CALIB_JOINT*_ZERO_PULSE`** 等。**`--apply`** 写入；**`--force`** 慎用。 |

**注意**

- **`verify`**：须 **`--csv <路径>`**，文件必须存在，且至少一行有效列 `u_px,v_px,z_mm,x_arm_truth_mm,y_arm_truth_mm`。一表流验收优先用总表里 **`role=verify`**。不再有内置隐藏测试点。
- **`verify`** 超过 `quality.max_error_mm` 时退出码为 **2**（终端显示 `FAIL`），仍会更新 `quality.verify_*`。
- **`Matrix_Raw`**：列/行为 **1 基**，`END checksum` 后须单独一行 **`tray_total=3`**（不参与 checksum）；详见 `F407/docs/F407_维护说明.txt`。
- 不要用仓库自带的 `calibration_results.json` 当现场终稿，须用真实数据跑通流水线。
- **TCP Matrix 客户端演示**已不再放在本目录；协议见 `F407/docs/`。

## 写入 F407（在 `Calibration/` 目录下）

```powershell
cd Calibration
copy calibration_config.example.json calibration_config.json
python apply_calibration_to_f407.py --config calibration_config.json --results calibration_results.json --apply
```

或：`python calibrate_device.py apply-f407 --apply`，然后 Keil 编译烧录。

## 子目录

`images/intrinsic`、`images/extrinsic`、`images/verify`、`data`、`output`；总表示例见 **`data/calibration_points.example.csv`**。
