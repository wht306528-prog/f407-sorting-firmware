# 双 RS485 量产构建与核对清单

按项勾选；改代码后执行 **Rebuild all target files** 与板级烟测。

## 1. 环境与目标

- [ ] 打开 Keil：[`Project.uvprojx`](Project.uvprojx)。
- [ ] 选中量产 Target（如 `F407_Target`）。
- [ ] **Rebuild all target files**。

## 2. 通过条件

- [ ] **0 Error**。
- [ ] 若链接报错含 `app_tcp_server` / `lwip` / `ethernet`，说明误把网口源文件加回工程，请从 `Project.uvprojx` 移除对应 FileGroup。

## 3. 链路核对（仅两路 485）

- [ ] **KEY1 / 电机与七色灯 RS485**：`bsp_uart.c`（**USART2：PA2=TX、PA3=RX，PC0=DE**）+ `modbus_master.c`。
- [ ] **KEY2 / 矩阵 Modbus**：`bsp_uart3.c` + `app_matrix_modbus.c`（**USART3：PB10=TX、PB11=RX**）。

## 4. 板级烟测（最小）

- [ ] 按 **KEY1**：屏或 HEX trace 可见电机口 **TX** 灯帧（仅发亦可）。
- [ ] 按 **KEY2**：`read_status` 终态为成功或 RunFlag 含 `MAT OK`；**RAW** 子页有 CSV 快照（来自 `AppProtocol_FreezeRawLastFromRows`，非网口）。

## 5. 可选：源码自检

- [ ] 在 `F407/Project/User` 搜索 `TCP`、`lwIP`、`以太`、`RJ45`：**活跃 UI/业务路径**中不应再出现「依赖以太网矩阵」语义；未编入工程的 `app_tcp_server.c` 等可保留归档说明。
