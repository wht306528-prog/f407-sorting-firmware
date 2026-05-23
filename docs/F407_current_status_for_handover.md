# F407 当前接手现状说明

本文记录当前 F407 端在鲁班猫 Modbus 联调后的实际状态，方便后续接手和排查。
它不是最终设计文档，而是当前阶段的事实记录。

## 1. 当前已经确认的事情

USART3 Modbus 矩阵通信已经打通。

KEY2 后，F407 LCD 已看到：

- `MAT U3 OK`
- `rows 150/150`
- `pkt 15/15`
- `mberr 0`
- `rtu_crc_fail 0`
- `MAT commit OK`
- `valid 150/150`
- `Fin:OK`
- 鲁班猫端 requests 一轮约增加 17 次。

所以当前卡点不是：

- RS485 接线；
- Modbus 从站地址；
- 波特率；
- 15 包读取；
- Modbus CRC16；
- 矩阵 CRC32；
- 150 格寄存器布局。

## 2. 当前已经跑通的数据流

F407 按 KEY2 后会走这一轮：

1. 读头部寄存器 `0x0000..0x000F`。
2. 从 `0x0010` 开始读 15 个数据包，每包 90 个 holding registers。
3. 再读一次头部作为 TAIL。
4. 检查 ready/updating/count/tray_total/batch/CRC32。
5. 重建 150 行 `MatrixFinalRow_t`。
6. 调 `AppMatrix_SetFromTcpParser()` 写入 Final 矩阵。
7. 冻结一份 RAW 快照给 LCD 显示。

相关文件：

- `Project/User/matrix/app_matrix_modbus.c`
- `Project/User/matrix/app_matrix.c`
- `Project/User/matrix/app_matrix_raw_validator.c`
- `Project/System/global_config.h`

当前矩阵语义：

- 总共 150 格；
- 3 个盘，每盘 10 行 x 5 列；
- 每格 9 个 holding registers；
- `class_id=0` 表示 empty；
- `class_id=1` 表示 white_ball；
- `class_id=2` 表示 yellow_ball；
- `u/v` 是图像像素坐标；
- `z_mm` 是 RealSense 对齐深度，单位 mm；
- confidence 在 Modbus 里是 0..10000 的定点整数。

## 3. Final 矩阵现在是什么状态

现在不是卡在“没有 Final 矩阵”。

从 LCD 看：

```text
MAT commit OK
valid 150/150
Fin:OK
```

这说明 F407 已经收到了鲁班猫矩阵，并且已经生成 Final 矩阵。

现在卡的是 Final 矩阵里的几何字段：

- `Xc_mm`
- `Yc_mm`
- `Xw_mm`
- `Yw_mm`
- `theta1_deg`
- `theta2_deg`
- `pulse_motor1_abs`
- `pulse_motor2_abs`
- `geom_ok`

目前这些几何字段没有算成功，所以显示：

```text
geom:BAD
```

## 4. Tray T1/T2/T3=ERR 问题状态

之前的现象：

```text
Tray T1=ERR T2=ERR T3=ERR
```

原因是旧显示逻辑把“同一盘内 class_id 混合”当成 ERR。

但鲁班猫当前输出的是穴位级矩阵，一盘 50 格里有空穴、白球、黄球混合是正常业务状态。

现在已经改成 LCD 主界面显示每盘统计：

```text
Tray E/W/Y T1=空/白/黄 T2=空/白/黄 T3=空/白/黄
```

例如现场已看到：

```text
Tray E/W/Y T1=48/1/1 T2=41/5/4 T3=41/6/3
```

注意：

- 这只改了 LCD 显示层；
- 没有改状态机使用的 `AppMatrix_CheckTrayFullOrEmpty()`；
- 所以没有改分拣动作逻辑。

相关文件：

- `Project/User/matrix/app_matrix.h`
- `Project/User/matrix/app_matrix.c`
- `Project/User/ui/app_display.c`

## 5. 当前真正卡住的问题：geom:BAD

SER 页当前显示类似：

```text
GEOM bad 150 first #0 T1 C1 R1 ... ik-2
GEOM uvz 156 102 1146 xw/yw 250 0
```

含义：

- 150 行 Final 矩阵都有；
- 但 150 行全部几何/IK 失败；
- 第一条失败行输入是 `u=156, v=102, z=1146`；
- 算出来的机械臂平面坐标却固定为 `xw=250, yw=0`；
- IK 返回 `ik-2`，表示几何不可达或求解失败。

这不是鲁班猫矩阵格式问题。

直接原因在：

```text
Project/User/calibration/app_calibration_params.h
```

当前有占位参数：

```c
#define CALIB_T_TRAY_FROM_CAM_Z_MM (0.00000000f)
```

而几何代码里：

```c
scal = -CALIB_T_TRAY_FROM_CAM_Z_MM / den;
```

当 `CALIB_T_TRAY_FROM_CAM_Z_MM` 为 0 时，所有像素射线求交比例都会变成 0。
后面的 tray->arm 占位变换又会把点塌缩到类似：

```text
xw = 250
yw = 0
```

所以现在 `geom:BAD` 的本质是：

```text
几何/手眼/机械臂标定参数没有落地。
```

相关文件：

- `Project/User/arm/app_kinematics.c`
- `Project/User/calibration/app_calibration_params.h`
- `Project/User/matrix/app_matrix.c`

## 6. F407 几何和机械臂代码到底做了什么

已经写了代码框架：

- 相机针孔模型；
- 畸变校正；
- 像素 UV 射线；
- 射线与托盘平面求交；
- tray 坐标到 arm 坐标的齐次变换；
- 对称五连杆正/逆解；
- 每格生成 `geom_ok`、关节角、目标脉冲；
- 机械臂动作前检查 `geom_ok`，失败就不下发动作。

没有完成或当前是占位：

- 相机到托盘平面的外参；
- 托盘平面到机械臂坐标系的变换；
- 真实机械臂杆长/轴心校验；
- 关节零点脉冲；
- 两轴电机/伺服 RS485 联调；
- 真实 pick/place 动作闭环。

当前状态可以概括成：

```text
矩阵通信：已通
矩阵结构校验：已通
LCD 托盘统计：已修
Final 矩阵：已有
几何/IK：代码有，但参数缺失
电机/机械臂：代码有，但现场硬件未验证
```

## 7. 能不能先随便拟一个标定值

可以，但只能用于“无机械臂、无电机”的软件验证。

不能用于真实机械臂运动。

原因：

- 随便给 `CALIB_T_TRAY_FROM_CAM_Z_MM` 一个非零值，可以让 `xw/yw` 不再固定为 `250/0`；
- 但没有真实 tray->arm 变换和关节零点，算出来的电机目标可能完全不对应真实空间；
- 如果接了真实电机，这样做有安全风险。

如果当前确实没有机械臂、没有电机，可以考虑增加一个明确的测试模式，例如：

```text
CFG_MATRIX_GEOM_SIM_MODE
```

这个模式只用于证明：

```text
Final 矩阵 -> 几何字段 -> LCD 显示
```

这条软件链路能跑起来。

但它必须明确标注为 test-only，并且不能用于真实机械臂。

## 8. 当前没有机械臂时，应该怎么继续

因为现在没有机械臂，也不确定电机能不能动，所以不要急着做真实手眼标定。

推荐顺序：

1. 继续保持 Modbus 链路不动。
2. 把当前 F407 项目结构、状态、卡点记录清楚。
3. 如果需要继续演示，可以讨论是否加“几何仿真/旁路模式”。
4. 如果后面接真实机械臂，再做正式标定：
   - 相机内参；
   - 相机到托盘平面的外参；
   - 托盘坐标系到机械臂坐标系；
   - 机械臂杆长；
   - 两轴零点脉冲；
   - 方向符号；
   - 可达范围。

## 9. LCD 页面怎么读

主界面：

- `MAT U3 OK`：USART3 Modbus 本轮成功。
- `rows 150/150`：收到了 150 格。
- `pkt 15/15`：15 包数据都读完。
- `mberr 0`：最近一次 Modbus 事务无错误。
- `Tray E/W/Y ...`：每盘空穴/白球/黄球统计。
- `MAT commit OK valid 150/150`：Final 矩阵已经提交。
- `geom:BAD`：Final 矩阵有了，但几何/IK 没过。
- `Fin:OK`：Final 矩阵存在且结构 OK，不代表 IK OK。

SER 页：

- `GEOM bad N`：几何/IK 失败的行数。
- `GEOM uvz u v z xw/yw X Y`：第一条失败行的输入和输出。
- 如果 `xw/yw` 一直是 `250 0`，说明标定仍是占位。

