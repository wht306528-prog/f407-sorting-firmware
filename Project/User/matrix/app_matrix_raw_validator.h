/**
 * app_matrix_raw_validator.h — Matrix_Raw（TCP 文本帧）校验与结果标志
 *
 * 职责：`app_protocol` 收字节流、`app_tcp_server` 喂给 OnStream，
 * demo/RAM 注入亦走同一路径——规则一致。**FL2/FL3 静默丢 RJ45 前由 tcp 分层实现，不写本模块。**
 */
#ifndef APP_MATRIX_RAW_VALIDATOR_H
#define APP_MATRIX_RAW_VALIDATOR_H

#include <stdint.h>

#include "app_matrix.h"

/** END 行 checksum 与数据行累加一致（mod 65536） */
#define APP_MATRIX_RAW_FL_TCP_CHK     0x01u
/** grid_full_unique 通过且已写入 app_matrix */
#define APP_MATRIX_RAW_FL_GRID_OK     0x02u

void AppMatrixRaw_ChecksumAccumulate(uint32_t *acc, const char *data_line_no_crlf);

uint32_t AppMatrixRaw_ChecksumMod65536(uint32_t acc);

/**
 * 150 行、穴唯一且无缺漏，class_id∈0..2，tray/col/row 合法（与 grid_full_unique 一致）
 */
uint8_t AppMatrixRaw_ValidateFullGrid(const MatrixFinalRow_t *rows, uint16_t n,
				      char *failbuf, uint16_t fail_cap);

/**
 * 稀疏帧（1≤n<M_expected）：条目合法、托盘穴位不重复即可，不要求铺满 150 穴。
 */
uint8_t AppMatrixRaw_ValidateSparseSlots(const MatrixFinalRow_t *rows, uint16_t n,
					char *failbuf, uint16_t fail_cap);

void AppMatrixRaw_ClearFrameFlags(void);

/** 在单帧 END 处理末尾调用：tcp_chk_ok=1 表示数据行累加与 END checksum 一致；grid_ok=SetFromTcpParser 成功 */
void AppMatrixRaw_NotifyTcpFrameDone(uint8_t tcp_chk_ok, uint8_t grid_ok);

uint8_t AppMatrixRaw_GetLastFlags(void);

#endif /* APP_MATRIX_RAW_VALIDATOR_H */
