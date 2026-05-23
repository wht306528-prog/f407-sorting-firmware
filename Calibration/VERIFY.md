### Calibration 自检

本目录仅保留 **`calibrate_device.py`** 与 **`apply_calibration_to_f407.py`**，详见 `README.md`。

```bash
pip install numpy opencv-python
# fit-arm --scipy 时才需要 scipy

python -m py_compile calibrate_device.py apply_calibration_to_f407.py

python calibrate_device.py verify --csv <自建像素复测.csv>

python calibrate_device.py one-table --points data/calibration_points.csv --images images/extrinsic
# fit-arm RMS 超限默认中止，可用 --fit-arm-max-rms-deg 0 关闭检查

python apply_calibration_to_f407.py \
  --config calibration_config.example.json \
  --results calibration_results.json --apply
```

未带 `--apply` 时头文件内容在 stdout，摘要在 stderr。结构校验失败可加 `--force`（慎用）。
