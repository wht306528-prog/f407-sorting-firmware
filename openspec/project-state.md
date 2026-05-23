# F407 项目状态总览

更新时间：2026-05-23

## 当前阶段目标

当前阶段是乒乓球演示联调，不是完整作物/机械臂闭环量产状态。

鲁班猫负责：

- RealSense / D435iF RGB + 对齐深度采集；
- 识别 3 个苗盘；
- 输出 150 格穴位级矩阵；
- 通过 USART3 Modbus RTU 从站提供 holding registers。

F407 负责：

- 作为 USART3 Modbus 主站，KEY2 后读取矩阵；
- 校验头部、15 包数据、TAIL、CRC32；
- 重建 Final 矩阵；
- 做几何转换和 IK；
- 后续驱动伺服、电磁阀、输送带。

## 已确认通过

USART3 Modbus 链路已经通过。

现场 LCD 已看到：

```text
MAT U3 OK
rows 150/150
pkt 15/15
mberr 0
rtu_crc_fail 0
MAT commit OK
valid 150/150
Fin:OK
```

鲁班猫端 requests 一轮约增加 17：

```text
HDR 1 + DATA 15 + TAIL 1
```

因此当前问题不在 RS485 通信层。

## 当前剩余主问题

当前主问题是：

```text
geom:BAD
GEOM bad 150
GEOM uvz ... xw/yw 250 0
```

原因是 `Project/User/calibration/app_calibration_params.h` 中几何标定参数仍是占位值，尤其：

```c
#define CALIB_T_TRAY_FROM_CAM_Z_MM (0.00000000f)
```

这会导致所有像素射线求交塌缩，最终输出固定 `xw/yw 250/0`，IK 返回 `ik-2`。

## 代码状态判断

F407 不是完全没做几何/机械臂代码。

已经有：

- 相机针孔模型；
- 像素射线与托盘平面求交；
- tray 到 arm 的齐次变换；
- 对称五连杆 IK；
- Final 矩阵行内的 `geom_ok`、角度、脉冲字段；
- `geom_ok==0` 时禁止机械臂下发动作。

但缺少：

- 现场相机到托盘平面外参；
- 托盘坐标到机械臂坐标转换；
- 关节零点脉冲；
- 电机/伺服实物联调；
- 真实机械臂 pick/place 验证。

## Git 管理状态

F407 仓库路径：

```text
E:\Tang\Peng\Ros2\sorting_device\sorting_device\F407
```

远程仓库：

```text
git@github.com:wht306528-prog/f407-sorting-firmware.git
```

提交规则：

- 修改后先本地编译和现场测试；
- 未经确认不提交；
- 未经确认不推送；
- commit message 需要中英文说明。

