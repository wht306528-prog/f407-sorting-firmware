# F407 现场测试手册

## 1. 编译

Keil 中：

1. 打开：
   ```text
   F407/Project/Project.uvprojx
   ```
2. 选择 target：
   ```text
   F407_Target
   ```
3. 按 `F7` 编译。

通过标准：

```text
".\Objects\Target407.axf" - 0 Error(s)
```

warning 如果是已有未使用函数或 scatter 提示，不一定阻塞；新增 warning 需要看原因。

## 2. 烧录

硬件：

- ST-Link V2 接电脑 USB；
- ST-Link 接 F407 SWD：
  - `SWDIO -> SWDIO`
  - `SWCLK -> SWCLK`
  - `GND -> GND`
  - `3.3V -> 3.3V` 或 F407 独立供电但必须共地。

Keil 中：

```text
Flash -> Download
```

通过标准：

```text
Erase Done.
Programming Done.
Verify OK.
Flash Load finished
```

## 3. KEY2 Modbus 矩阵读取测试

前提：

- 鲁班猫服务已启动；
- `START_MODBUS_RTU_SLAVE=true`；
- F407 USART3 RS485 已接到鲁班猫 USB-RS485；
- 从站 ID 为 1；
- 波特率 115200 8N1。

鲁班猫看日志：

```bash
cd ~/sorting_robot
scripts/pingpong_service.sh logs
```

F407 操作：

1. F407 上电或复位。
2. 按 KEY2。
3. 看 LCD 主界面。

通过标准：

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

鲁班猫日志里的 requests 应增长。

## 4. Tray 统计显示测试

按 KEY2 后主界面应显示类似：

```text
Tray E/W/Y T1=48/1/1 T2=41/5/4 T3=41/6/3
```

含义：

- E = empty，`class_id=0`
- W = white_ball，`class_id=1`
- Y = yellow_ball，`class_id=2`

通过标准：

- 混合盘不再显示 `ERR`。
- 只有结构不完整或非法 class 时才显示 `Tray ERR`。

## 5. geom:BAD 诊断

按 KEY2 后进入 SER 页，观察：

```text
GEOM bad ...
GEOM uvz ...
```

当前预期现象：

```text
GEOM bad 150
GEOM uvz ... xw/yw 250 0
```

这表示 Final 矩阵有了，但几何标定缺失。

不要把该现象当作 Modbus 失败。

## 6. 当前禁止事项

在没有真实机械臂和电机验证前：

- 不要启动真实分拣动作；
- 不要用随便拟的标定值驱动电机；
- 不要把 `geom:BAD` 当成鲁班猫格式错误；
- 不要改已经通过的 Modbus 链路。

## 7. 无机械臂 SIM 模式测试

当前无机械臂阶段，固件允许打开两个调试开关：

```c
CFG_MATRIX_GEOM_SIM_MODE = 1
CFG_ARM_MOTION_SIM_MODE  = 1
```

含义：

- `GEOM_SIM`：收到矩阵后，不用真实标定和 IK，给每个格子生成稳定的假 X/Y/theta/pulse，并让 `geom_ok=1`。
- `ARM_MOTION_SIM`：状态机调用机械臂取放时，不发电机 Modbus、不驱动电磁阀，只更新内存矩阵。

烧录后检查：

1. F407 上电。
2. 主界面 RunFlag 行应能看到：
   ```text
   SIM:G1A1
   ```
3. SER 页顶部应能看到：
   ```text
   SIM MODE: geom 1 arm 1, no real arm/motor movement
   ```
4. 按 KEY2 读取矩阵。
5. 期望：
   ```text
   MAT U3 OK
   rows 150/150
   pkt 15/15
   MAT commit OK
   valid 150/150
   geom:OK
   ```
6. 进入 FIN 页查看 Final 行：
   - `geom` 应为 1；
   - `Xw/Yw` 不应再全部是 `250/0`；
   - `P1/P2` 应有稳定的假脉冲值。

如果看到 `geom:BAD`：

- 先确认 `CFG_MATRIX_GEOM_SIM_MODE` 是否为 1；
- 确认烧录的是新固件；
- 再拍主界面和 SER 页。

如果后续接入真实电机/机械臂：

1. 必须把两个宏改为 0。
2. 重新编译烧录。
3. 先做真实标定和电机单轴测试。
4. 不允许直接用 SIM 模式跑真实硬件。
