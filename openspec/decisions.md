# 决策记录

## D001 - F407 仓库单独管理

结论：

- 只把 `F407` 目录作为固件 Git 仓库。
- 不在外层 `sorting_device` 目录初始化 Git。

原因：

- 外层目录包含 ROS2、资料、图片、测试工程和临时工具。
- 直接用外层 Git 会导致 VS Code 出现几千个未跟踪文件。

当前状态：

- 外层误建 `.git` 已删除。
- F407 子仓库正常。

## D002 - USART3 Modbus 不再按通信失败排查

结论：

- 当前 USART3 Modbus 链路已闭环。
- 后续问题先按应用层、几何层、标定层排查。

依据：

- `pkt 15/15`
- `rows 150/150`
- `mberr 0`
- CRC OK
- `MAT commit OK`
- `valid 150/150`

## D003 - Tray ERR 显示语义修正

结论：

- LCD 主界面不再把混合盘显示为 `ERR`。
- 改为显示每盘 E/W/Y 数量。

原因：

- 鲁班猫输出穴位级矩阵。
- 单盘 50 格内 `class_id=0/1/2` 混合是正常业务状态。

注意：

- 只改 LCD 显示层。
- 保留 `AppMatrix_CheckTrayFullOrEmpty()` 给原状态机使用。

已提交：

```text
4319c86 Show tray class counts on LCD / 显示托盘空白黄统计
```

## D004 - Calibration 工具应纳入 F407 仓库

结论：

- 外层 `Calibration` 应复制到 `F407/Calibration`。
- 原始外层目录不移动，避免破坏旧路径。

原因：

- F407 README 和 `app_calibration_params.h` 都引用 `Calibration/apply_calibration_to_f407.py`。
- 但 F407 仓库之前缺少该目录，导致标定流程断档。

排除项：

- `__pycache__/`
- `*.pyc`
- `calibration_config.json`
- `calibration_results.json`

这些属于本地缓存或现场结果，不默认提交。

## D005 - 不用伪标定驱动真实电机

结论：

- 可以讨论 test-only 几何仿真模式。
- 不能把随便拟的标定参数用于真实机械臂运动。

原因：

- 当前没有机械臂本体。
- 电机/伺服也未验证。
- tray->arm 和关节零点都没有现场数据。

## D006 - 无机械臂阶段启用软件仿真链路

结论：

- 当前没有机械臂和电机，允许临时开启软件仿真模式验证 F407 内部状态机。
- 使用两个宏：
  - `CFG_MATRIX_GEOM_SIM_MODE=1`
  - `CFG_ARM_MOTION_SIM_MODE=1`

目的：

- 让 KEY2 收到矩阵后，Final 矩阵的 `geom_ok` 通过；
- 让 `AppSort` 可以从 WAIT_MATRIX 继续往 FL2/FL3 等系统标志位推进；
- 观察 RunFlag、SYS、Tray E/W/Y、Final 行变化是否符合业务逻辑。

限制：

- 这是无硬件调试模式，不是真实标定；
- 真实电机/机械臂接入前必须改回 0；
- LCD 主界面会显示 `SIM:G1A1`；
- SER 页会显示 `SIM MODE: geom 1 arm 1, no real arm/motor movement`。
