/*
 * app_matrix_modbus.h
 *
 * 矩阵侧通信：USART3(PB10/PB11) Modbus RTU 主站。由按键 KEY2 触发一轮完整读取，
 * 不做后台自动轮询（空闲时不占用串口）。
 *
 * KEY2 一轮状态流（3588 侧约定，实现见 .c）：按 KEY2 后走 HDR→15 包 DATA→TAIL。
 * HDR：FC03 读保持 0x0000、数量 16，解析得 H[]；门禁通过需 H[0]==1（首寄存器 ready）等条件。
 * DATA：chunk 0..14，每步一发一收读 90 寄存器，填满 90×15=s_matrix_regs；rows_received 同步进度。
 * TAIL：再读 16 字头比对 batch/counters，CRC32（IEEE，大端寄存器拼接）与 H[9]/H[10] 快照一致后
 * 调用 AppMatrix_SetFromTcpParser，并 AppProtocol_FreezeRawLastFromRows 生成 LCD「原始矩阵」CSV（来自行表，非网口正文）。
 *
 * AppMatrixModbusDiag_t.read_status（屏显「读取状态」）取值约定：
 *   0 = 待机：未在读取，或 AppMatrixModbus_Init 后；上一轮若成功/失败会保留终态
 *       直至下一次 Init 或 Key2Request。
 *   1 = 读取中：已接受 Key2Request，状态机在 HDR/DATA/TAIL 任一步。
 *   2 = 成功：150 穴位 CRC32 与头内一致，且 AppMatrix_SetFromTcpParser 成功；
 *       已调用 AppProtocol_FreezeRawLastFromRows 供「原始矩阵」子页浏览。
 *   4 = 失败：任一步 Modbus 错、头门禁失败、batch 变化、CRC32 不符、矩阵重建失败等。
 *
 * AppMatrixModbusDiag_t.rows_received：当前轮已确认的穴位行数（每收到一包矩阵数据 +10），
 *   成功结束时为 150；失败时为失败前最后进度。
 *
 * phase：0=空闲 1=读头 2=读 15 包 3=读尾头并 CRC/落盘。
 *
 * 首步请求形态：FC03 读保持 0x0000、数量 16（报文 01 03 00 00 00 10 + CRC，SA 以 CFG 为准）；
 * 应答 32 字节 payload：H[0]=1 等为门禁通过条件，见 app_matrix_modbus.c 头注释与 hdr_gate_check。
 */

#ifndef APP_MATRIX_MODBUS_H
#define APP_MATRIX_MODBUS_H

#include <stdint.h>

#define APP_MAT_RD_ST_IDLE     0u
#define APP_MAT_RD_ST_READING  1u
#define APP_MAT_RD_ST_OK       2u
#define APP_MAT_RD_ST_ERR      4u

void AppMatrixModbus_Init(void);

void AppMatrixModbus_Poll(uint32_t tick_ms);

uint8_t AppMatrixModbus_Key2Request(void);

void AppMatrixModbus_OnMatrixCleared(void);

uint8_t AppMatrixModbus_LastCommitOk(void);

uint8_t AppMatrixModbus_IsBusy(void);

void AppMatrixModbus_GetLastRunMsg(char *dst, unsigned cap);

typedef struct
{
	uint8_t  busy;
	uint8_t  phase;
	uint8_t  chunk_done;
	uint8_t  read_status;
	uint16_t rows_received;
	uint8_t  last_txn_err;
	uint16_t hdr_ready;
	uint16_t hdr_updating;
	uint16_t hdr_batch;
	uint16_t hdr_upd_ctr;
	uint16_t hdr_count;
	uint16_t hdr_tray_total;
	uint16_t hdr_crc_hi;
	uint16_t hdr_crc_lo;
	uint32_t crc32_calc;
	uint32_t crc32_expect;
	uint32_t crc_fails;
	uint32_t to_fails;
	uint32_t len_fails;
	uint32_t commits_ok;
	uint32_t rx_ovf_u3;
} AppMatrixModbusDiag_t;

void AppMatrixModbus_GetDiag(AppMatrixModbusDiag_t *d);

#endif
