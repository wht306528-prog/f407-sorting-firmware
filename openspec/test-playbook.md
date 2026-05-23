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

