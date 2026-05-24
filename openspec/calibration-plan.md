# 几何和标定计划

## 当前事实

当前 F407 的几何代码框架存在，但标定参数没有现场落地。

占位参数示例：

```c
#define CALIB_T_TRAY_FROM_CAM_Z_MM (0.00000000f)
#define CALIB_T_TRAY_TO_ARM_03 (250.00000000f)
#define CALIB_JOINT1_ZERO_PULSE (0L)
#define CALIB_JOINT2_ZERO_PULSE (0L)
```

因此当前 `geom:BAD` 是合理现象。

## 标定需要解决的坐标链

目标链路：

```text
RGB pixel u/v
  -> camera ray
  -> tray plane point
  -> arm plane xw/yw
  -> five-bar IK theta1/theta2
  -> motor pulse target
```

需要的参数：

1. 相机内参：
   - fx, fy, cx, cy, D
2. 相机到托盘平面的外参：
   - `CALIB_R_TRAY_FROM_CAM_*`
   - `CALIB_T_TRAY_FROM_CAM_*`
3. 托盘坐标到机械臂坐标：
   - `CALIB_T_TRAY_TO_ARM_*`
4. 机械臂参数：
   - 轴心位置；
   - 主动杆/从动杆长度；
   - 关节角方向；
   - 关节零点角；
   - 关节零点脉冲。

## 没有机械臂时能做什么

可以做：

- 验证 Modbus 矩阵；
- 验证 Final 矩阵；
- 验证 LCD 统计；
- 查看 `u/v/z` 是否合理；
- 准备标定工具和文档；
- 讨论 test-only 几何仿真模式。

不能做：

- 真实 tray->arm 标定；
- 真实 IK 可达性确认；
- 真实电机目标验证；
- 真实 pick/place。

## 关于伪标定

可以临时拟参数用于无硬件软件链路测试，但必须满足：

- 有明确宏开关；
- 命名必须包含 SIM 或 TEST；
- 默认关闭；
- 文档明确禁止用于真实机械臂。

建议名称：

```c
CFG_MATRIX_GEOM_SIM_MODE
```

用途：

- 证明 `Final Matrix -> geom fields -> LCD` 软件链路可以显示非塌缩坐标。

不允许用途：

- 驱动真实电机；
- 作为现场标定结果；
- 替代正式手眼标定。

## 后续有机械臂时的最小采点

至少采 6 到 9 个点，每个点记录：

```text
tray_id,row,col
u_px,v_px,z_mm
x_arm_truth_mm,y_arm_truth_mm
```

优先采：

- T1 左上；
- T1 右下；
- T2 中心；
- T3 左上；
- T3 右下；
- 横向/纵向边缘点。

如果能读电机反馈，还需要：

```text
joint1_pulse
joint2_pulse
```

## 工具目录

标定工具已经复制到：

```text
F407/Calibration
```

主要脚本：

- `Calibration/calibrate_device.py`
- `Calibration/apply_calibration_to_f407.py`

真实现场配置和结果默认不提交：

- `Calibration/calibration_config.json`
- `Calibration/calibration_results.json`

## 当前 SIM 实现

当前已采用两个宏：

```c
CFG_MATRIX_GEOM_SIM_MODE
CFG_ARM_MOTION_SIM_MODE
```

用途：

- 证明 `Final Matrix -> geom fields -> LCD` 软件链路可以显示非塌缩坐标；
- 证明 `AppSort -> AppArm_PickPlace -> AppMatrix_ApplyTransfer` 软件状态链路能推进。

不允许用途：

- 驱动真实电机；
- 作为现场标定结果；
- 替代正式手眼标定。
