# F407 分拣固件（唯一 Markdown 入口）

- **可编译工程**：`F407/Project/Project.uvprojx`（STM32F407ZGTx，裸机 + lwIP + TCP Matrix_Raw）。
- **说明文档（TXT）**：
  - [`docs/F407_维护说明.txt`](docs/F407_维护说明.txt) — 协议、KEY、校验标志、测试流程、机械参数索引
  - [`docs/F407_IO引脚与逻辑表.txt`](docs/F407_IO引脚与逻辑表.txt) — RS485 / GPIO / 按键
- **配置真源**：`F407/Project/System/global_config.h`
- **Matrix TCP 跑通**：默认 `MATRIX_SAMPLE_WINDOW_FRAMES=1`（单帧即提交）、`CFG_TCP_MATRIX_KEEP_RX_OPEN_AFTER_OK=1`（提交后同连接可继续发）；详见 `docs/F407_维护说明.txt` 节「三之二」与「四」。
- **相机 / 臂平面标定**：`Project/User/calibration/app_calibration_params.h`（由 `Calibration/apply_calibration_to_f407.py` 或 `Calibration/calibrate_device.py apply-f407` 根据 `calibration_results.json` 生成）

旧版多份 `.md` 已合并/淘汰为上述 TXT；第三方库自带说明仍在各自目录。

**注意**：本 README 中若提及 Cube + FreeRTOS + W5500 等路径，属历史或其它分支，日常调试请以 `Project` 内 Keil 工程与 `docs/*.txt` 为准。
