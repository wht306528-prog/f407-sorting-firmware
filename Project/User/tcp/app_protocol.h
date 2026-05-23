/**
 * app_protocol.h — Matrix_Raw 文本解析与 Raw 快照 API
 *
 * 实现：app_protocol.c。量产矩阵经 USART3 Modbus（KEY2）+ `AppProtocol_FreezeRawLastFromRows`。
 * OnStream **仅**供调试字节流或串口桥；当前 Keil 目标无网口矩阵。
 */
#ifndef __APP_PROTOCOL_H__
#define __APP_PROTOCOL_H__

#include <stdint.h>

#include "app_matrix.h"

void AppProtocol_Init(void);

/**
 * AppProtocol_OnStream：按换行组行解析 ASCII Matrix_Raw（测试/调试注入字节时用）。
 */
void AppProtocol_OnStream(const uint8_t *data, uint16_t len);

/**
 * 本拍是否允许把字节流送入 `AppProtocol_OnStream`（采样关闸）；测试钩子可查询。
 */
uint8_t AppProtocol_ShouldAcceptStream(void);

/** 关闸后再次开窗采集（清空当前窗口计数）；由传送带 FL4/FL5/FL6 的 `run_conveyor_flag` 开头调用。 */
void AppProtocol_ArmSampleWindow(void);

/**
 * 择优写入成功后仅重新开窗、清空采样候选，保留 `AppProtocol_LastChecksumOk`。
 * 用于 CFG_MATRIX_KEEP_RX_OPEN_AFTER_OK：鲁班猫持续下发时不吞后续帧。
 */
void AppProtocol_ReopenGateKeepLastOk(void);

/** 1=关闸前窗口打开，上位机矩阵字节会进入采样器 */
uint8_t AppProtocol_IsGateOpen(void);

/** 当前窗口内已缓存的合法帧数 0..MATRIX_SAMPLE_WINDOW_FRAMES */
uint8_t AppProtocol_GetSampleValidCount(void);

/** 上一帧 `END` 校验是否通过（不含 START/END 行，仅数据行 ASCII 累加 %65536） */
uint8_t AppProtocol_LastChecksumOk(void);

/** 最近一次冻结的 Matrix_Raw 正文行数（数据 CSV，不含 START/END），可能少于 declared */
uint16_t AppProtocol_GetRawLastLineCount(void);

/** 拷贝第 idx 行 Raw CSV（0..count-1）；失败返回 0 */
uint8_t AppProtocol_GetRawLastLine(uint16_t idx, char *dst, uint16_t cap);

/** START 行声明的 count（期望 MATRIX_EXPECTED_ROWS） */
uint16_t AppProtocol_GetRawLastDeclaredRows(void);

/** 上一帧 END checksum（ASCII 累加）是否通过 */
uint8_t AppProtocol_GetRawLastChecksumOk(void);

/** 在校验通过后 `AppMatrix_SetFromTcpParser` 是否成功（几何/格位唯一等） */
uint8_t AppProtocol_GetRawLastGridOk(void);

/** BODY 阶段当前累加器（未取模）；用于 RAM/demo 合成帧拼 END，与协议内部一致 */
uint32_t AppProtocol_GetDataLinesChecksumAccumulator(void);

/** 注入解析器的字节累计（AppProtocol_Init 时清零；量产主路径为 Modbus FreezeRaw） */
uint32_t AppProtocol_GetStreamRxBytes(void);

/** 完整矩阵提交成功次数（s_last_ok 置 1），AppProtocol_Init 时清零 */
uint32_t AppProtocol_GetMatrixFramesOkCount(void);

/*
 * AppProtocol_FreezeRawLastFromRows：
 * 将已解析好的 Final 行向量写入「原始矩阵」LCD 子页使用的 s_raw_last 文本快照。
 * 与 START/END 文本帧路径无关：Modbus 落盘不写 ASCII 累加；chk_ok/grid_ok
 * 置 1 仅表示「快照有效」；`AppProtocol_GetRawLastChecksumOk` 对 Freeze 路径恒 0。
 */
void AppProtocol_FreezeRawLastFromRows(const MatrixFinalRow_t *rows,
				     uint16_t n_rows);

#endif
