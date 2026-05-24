# 2026-05-24 电机 Modbus 模拟驱动器电脑侧排查记录

## 测试目标

验证 F407 USART2 电机 Modbus 输出是否能被 PC 端模拟驱动器接收，并由模拟驱动器返回伺服应答。

预期链路：

```text
F407 USART2 电机总线 -> USB-RS485/USB-TTL -> PC COM 口 -> tools/servo_modbus_sim.py
```

## 当前用户接线描述

用户描述为：

```text
电脑 USB 口接到疑似 F407 PA2 / PA3
```

风险/疑点：

- PA2/PA3 是 F407 的 USART2 TTL 引脚，不是 USB 差分口。
- 如果用的是 USB-TTL 模块，需要 TX/RX 交叉并共地。
- 如果用的是 USB-RS485 模块，不能直接接 PA2/PA3，应接 RS485 收发器的 A/B 总线侧。
- 当前固件的 USART2 总线设计还包含 PC0 作为 RS485 DE/~RE 控制脚；若绕过 RS485 收发器直连 PA2/PA3，需要确认收发方向和电平。

## 已执行电脑侧检查

工作目录：

```text
E:\Tang\Peng\Ros2\sorting_device\sorting_device\F407
```

检查串口枚举：

```powershell
Get-CimInstance Win32_SerialPort
[System.IO.Ports.SerialPort]::GetPortNames()
Get-ItemProperty -Path 'HKLM:\HARDWARE\DEVICEMAP\SERIALCOMM'
```

结果：

```text
未枚举出任何 COM 口。
```

尝试查询 PnP 设备：

```powershell
Get-PnpDevice -Class Ports
Get-PnpDevice -Class USB ...
```

结果：

```text
查询超时；但 SERIALCOMM 和 .NET 串口列表均为空，已足够说明当前没有可用串口。
```

## 当前结论

目前 PC 端无法启动 `tools/servo_modbus_sim.py` 进行实际收发测试，因为没有可打开的 COM 口。

这说明当前问题优先级是：

```text
USB 转串口/USB-RS485 是否被 Windows 识别
接线是否接到了正确的串口/RS485 总线
驱动是否安装
```

而不是 F407 是否已经发出 Modbus。

## 下一步建议

1. 确认电脑设备管理器是否出现 `COMx`。
2. 如果使用 USB-TTL：
   - USB-TTL TX -> F407 PA3/RX
   - USB-TTL RX -> F407 PA2/TX
   - USB-TTL GND -> F407 GND
   - 只能用 3.3V TTL，不能用 5V。
3. 如果使用 USB-RS485：
   - USB-RS485 A/B -> F407 板上 RS485 收发器 A/B
   - 不要直接接 PA2/PA3。
4. 出现 COM 口后运行：

```powershell
python tools\servo_modbus_sim.py --port COMx --baud 115200 -v
```

5. F407 烧录当前配置后按 KEY2，观察模拟器是否打印 `FC03/FC16/FC06`。

## 追加测试：COM4 模拟驱动器闭环成功

时间：

```text
2026-05-24 14:34:29 左右
```

PC 端串口：

```text
COM4
```

运行命令：

```powershell
python tools\servo_modbus_sim.py --port COM4 --baud 115200 -v
```

现象：

- 模拟器成功打开 COM4。
- F407 按 KEY2 后，矩阵侧已能 `MAT U3 OK`、`MAT commit OK`。
- 随后 F407 进入 FL2，并通过 USART2 对两个电机从站发出真实 Modbus RTU。
- 模拟器收到并应答后，F407 连续执行了多次源点/目标点移动闭环。

日志统计：

```text
log_lines=1449
rx_count=724
tx_count=724
target_writes=120
enable_writes=120
trigger_writes=120
clear_writes=119
reach_reads=120
pos_reads=125
```

代表性帧：

```text
RX 01 03 0B 07 00 02 77 EE -> read 0x0B07/2, pos=0, reached=True
TX 01 03 04 00 00 00 00 FA 33
RX 01 10 10 0E 00 02 04 08 D4 00 00 FD BB -> write regs 0x100E/2, target=2260
TX 01 10 10 0E 00 02 24 CB
RX 01 06 0D 12 01 FB 6B 70 -> write 0x0D12=0x01FB, en=True, estop=False
TX 01 06 0D 12 01 FB 6B 70
RX 01 06 0D 08 00 03 4A A5 -> write 0x0D08=0x0003, en=True, estop=False
TX 01 06 0D 08 00 03 4A A5
RX 01 03 0B 04 00 01 C7 EF -> read 0x0B04/1, pos=2260, reached=True
TX 01 03 02 00 01 79 84
RX 01 06 0D 08 00 00 0A A4 -> write 0x0D08=0x0000, en=True, estop=False
TX 01 06 0D 08 00 00 0A A4
```

分析结论：

- F407 的电机 Modbus 输出链路成立：`Final -> FL2/FL3 -> AppArm_PickPlace -> AppMotor_* -> USART2 Modbus`。
- 电机 1/2 的关键流程均已出现：读 P0B-07、写 P10-14、P0D-18=507 使能、P0D-08=3 触发、读 P0B-04/P0B-07 到位、P0D-08=0 清触发。
- 当前测试仍是 PC 模拟驱动器，不能代表真实电机安全可动；接真实硬件前仍需关闭无关 SIM、核对标定、限位、急停和单轴小行程。
