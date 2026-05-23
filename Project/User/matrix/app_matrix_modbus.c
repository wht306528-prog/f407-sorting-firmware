/*
 * app_matrix_modbus.c
 *
 * KEY2 单轮：首帧 FC03 读保持寄存器 0x0000 共 16 个字（含 ready/updating/batch 等），
 * 门禁通过后再读 15 包矩阵区（每包 90 寄存器=10 穴位×9），再读一次头比对 batch，
 * 对 1350 个寄存器按大端字节做 IEEE CRC32，与头里快照寄存器（高 16 在 H[9]、低 16 在 H[10]）
 * 比较；通过后组 150 行 MatrixFinalRow_t，AppMatrix_SetFromTcpParser 写入几何与分拣用矩阵，
 * 并 AppProtocol_FreezeRawLastFromRows 填满 LCD「原始矩阵」页的 CSV 快照（非 TCP 正文路径）。
 *
 * RunFlag：mat_cycle_finish 写入；门禁失败为明细句（保持寄存器地址与实值）：
 *   MAT HDR MB ERR   首头 Modbus 收发/CRC/从站应答异常
 *   MAT HDR gate 0xADDR=实值 need期望…
 *   MAT DATA pkt N/15 @0x…. err码
 *   MAT TAIL MB ERR  尾头再读失败
 *   MAT BATCH bat 首>尾 seq 首>尾
 *   MAT TAIL gate 0xADDR=…
 *   MAT CRC32 FAIL   本地 CRC32 与头里矩阵 CRC 不一致
 *   MAT REBUILD FAIL SetFromTcpParser/格点校验失败
 *   MAT OK CRC+RB    全流程成功
 *
 * 【首帧报文形状（站号 = CFG_MATRIX_MODBUS_SLAVE_ID，量产常为 1）】
 * 主机请求：SA FC03 RegHi RegLo QtyHi QtyLo CRClo CRChi
 *   例 SA=01：01 03 00 00 00 10 + CRC16（读 0x0000 起共 16 个保持寄存器）。
 * 从机应答：SA FC03 0x20 + 32 字节寄存器数据（大端）+ CRC16；
 *   regs_bytes_to_u16 得到 H[0..15]。门禁 hdr_gate_check：H[0]=1 ready，H[1]=0 updating，
 *   H[13]=MATRIX_EXPECTED_ROWS（150，如 0x0096），H[15]=3 托盘；H[9]/H[10] 为矩阵 CRC32 快照高/低字。
 */

#include "app_matrix_modbus.h"

#include "app_display.h"
#include "app_matrix.h"
#include "app_protocol.h"
#include "bsp_uart3.h"
#include "delay.h"
#include "global_config.h"
#include "modbus_crc.h"
#include "modbus_master.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef enum
{
	MBS_IDLE = 0,
	MBS_K2_HDR = 1,
	MBS_K2_DATA = 2,
	MBS_K2_TAIL = 3,
} app_mbs_phase_t;

static app_mbs_phase_t s_phase;
static uint8_t         s_busy;
static uint8_t         s_chunk;
static uint8_t         s_chunks_ok;

static uint16_t s_snap_batch;
static uint16_t s_snap_ctr;
static uint32_t s_snap_crc32;

static uint16_t s_hdr_diag[16];

static MatrixFinalRow_t s_build[MATRIX_EXPECTED_ROWS];

#define MX_MATRIX_NREGS (CFG_MATRIX_PACKET_COUNT * CFG_MATRIX_REGS_PER_PACKET)

static uint16_t s_matrix_regs[MX_MATRIX_NREGS];

static uint32_t s_crc_fails;
static uint32_t s_to_fails;
static uint32_t s_len_fails;
static uint32_t s_commits_ok;
static uint8_t  s_last_commit_ok;
static uint8_t  s_last_txn_err;

static uint32_t s_diag_crc32_calc;
static uint32_t s_diag_crc32_expect;

static char s_last_run_msg[96];

static uint8_t  s_read_status;
static uint16_t s_rows_recv;

#define MX_RX_CAP 280u

#if MX_MATRIX_NREGS != (MATRIX_EXPECTED_ROWS * CFG_MATRIX_REGS_PER_CELL)
#error "matrix register total must match 150 * 9"
#endif

static uint32_t mx_rtu_gap_ms(void)
{
	uint32_t baud = CFG_MATRIX_MODBUS_BAUD;

	if (baud > 19200u)
	{
		return 2u;
	}
	if (baud < 1200u)
	{
		baud = 9600u;
	}
	{
		uint32_t bpm = baud / 1000u;

		if (bpm == 0u)
		{
			bpm = 1u;
		}
		return (38u + bpm - 1u) / bpm;
	}
}

/* RTU：帧间总线应无声≥3.5 字符时间；RS485 半双工换向后再发下一询 */
static void mx_rtu_pause_before_tx(void)
{
	uint32_t g = mx_rtu_gap_ms();

	if (g < 2u)
	{
		g = 2u;
	}
	Delay_ms(g);
}

/*
 * RTU：按静音间隔收满一帧后 *out_len=len；仅在 return OK 路径各调用一次
 * BSP_USART3_TracePushRxFrameEnd()，与 HEX 显示「单 ADU 仅一处帧结束、续行无第二个 RX:」一致。
 */
static uint8_t mx_recv_adu(uint8_t *rx, uint16_t rx_cap, uint16_t *out_len,
			   uint32_t timeout_ms)
{
	uint32_t t0 = SysTick_GetMs();
	uint32_t last_rx = 0u;
	uint16_t len = 0u;
	uint32_t gap = mx_rtu_gap_ms();

	*out_len = 0u;

	for (;;)
	{
		uint8_t  b;
		uint32_t now = SysTick_GetMs();

		while (BSP_USART3_ReadByte(&b) != 0u)
		{
			if (len >= rx_cap)
			{
				return MB_MASTER_ERR_BUF;
			}
			rx[len] = b;
			len++;
			last_rx = SysTick_GetMs();
		}
		now = SysTick_GetMs();

		if (len > 0u)
		{
			if ((now - last_rx) >= gap)
			{
				*out_len = len;
				BSP_USART3_TracePushRxFrameEnd();
				return MB_MASTER_OK;
			}
		}

		if ((now - t0) >= timeout_ms)
		{
			if (len == 0u)
			{
				return MB_MASTER_ERR_TIMEOUT;
			}
			if ((now - last_rx) >= gap)
			{
				*out_len = len;
				BSP_USART3_TracePushRxFrameEnd();
				return MB_MASTER_OK;
			}
			return MB_MASTER_ERR_TIMEOUT;
		}
	}
}

static uint8_t mx_fc03_read(uint16_t reg_addr, uint16_t reg_qty, uint8_t *data_out,
			    uint16_t *out_data_len)
{
	uint8_t  tx[8];
	uint8_t  rx[MX_RX_CAP];
	uint16_t rlen;
	uint8_t  br;

	if ((reg_qty == 0u) || (reg_qty > 123u) || (data_out == NULL) ||
	    (out_data_len == NULL))
	{
		return MB_MASTER_ERR_ARG;
	}

	tx[0] = (uint8_t)CFG_MATRIX_MODBUS_SLAVE_ID;
	tx[1] = 0x03u;
	tx[2] = (uint8_t)((reg_addr >> 8) & 0xFFu);
	tx[3] = (uint8_t)(reg_addr & 0xFFu);
	tx[4] = (uint8_t)((reg_qty >> 8) & 0xFFu);
	tx[5] = (uint8_t)(reg_qty & 0xFFu);
	Modbus_AppendCRC(tx, 6u);

	mx_rtu_pause_before_tx();
	BSP_USART3_RxFlush();
	BSP_USART3_SendBlocking(tx, 8u);

	br = mx_recv_adu(rx, sizeof(rx), &rlen, CFG_MATRIX_MODBUS_RSP_TIMEOUT_MS);
	if (br != MB_MASTER_OK)
	{
		s_last_txn_err = br;
		if (br == MB_MASTER_ERR_TIMEOUT)
		{
			s_to_fails++;
		}
		return br;
	}

	if (!Modbus_CheckCRC(rx, rlen))
	{
		s_crc_fails++;
		s_last_txn_err = MB_MASTER_ERR_CRC;
		return MB_MASTER_ERR_CRC;
	}
	if (rlen < 5u || rx[0] != (uint8_t)CFG_MATRIX_MODBUS_SLAVE_ID)
	{
		s_last_txn_err = MB_MASTER_ERR_SLAVE;
		return MB_MASTER_ERR_SLAVE;
	}

	{
		ModbusTxnParsed_t pr;
		uint8_t           pe =
		    ModbusMaster_ParseResponse(rx, rlen, 0x03u, &pr);

		if (pe != MB_MASTER_OK)
		{
			s_last_txn_err = pe;
			return pe;
		}
		if (pr.is_exception != 0u)
		{
			s_last_txn_err = MB_MASTER_ERR_EXCEPTION;
			return MB_MASTER_ERR_EXCEPTION;
		}
		if ((uint16_t)pr.data_len != (uint16_t)(reg_qty * 2u))
		{
			s_len_fails++;
			s_last_txn_err = MB_MASTER_ERR_LEN;
			return MB_MASTER_ERR_LEN;
		}
		memcpy(data_out, pr.data, pr.data_len);
		*out_data_len = pr.data_len;
	}

	s_last_txn_err = MB_MASTER_OK;
	return MB_MASTER_OK;
}

static void regs_bytes_to_u16(const uint8_t *p, uint16_t *dst, uint16_t nregs)
{
	uint16_t i;

	for (i = 0u; i < nregs; i++)
	{
		dst[i] =
		    (uint16_t)(((uint16_t)p[2u * i] << 8) | (uint16_t)p[2u * i + 1u]);
	}
}

static void fill_cell_from_u16(const uint16_t *cell9, MatrixFinalRow_t *row)
{
	uint16_t w0 = cell9[0];
	uint16_t w1 = cell9[1];

	row->tray_id = (uint16_t)(w0 >> 8);
	row->col = (uint16_t)(w0 & 0xFFu);
	row->row = (uint16_t)(w1 >> 8);
	row->class_id = (uint16_t)(w1 & 0xFFu);
	row->confidence = cell9[2];
	row->u = (int32_t)(((uint32_t)cell9[3] << 16) | (uint32_t)cell9[4]);
	row->v = (int32_t)(((uint32_t)cell9[5] << 16) | (uint32_t)cell9[6]);
	row->z_mm = (int32_t)(((uint32_t)cell9[7] << 16) | (uint32_t)cell9[8]);
	row->Xc_mm = 0.f;
	row->Yc_mm = 0.f;
	row->Xw_mm = 0.f;
	row->Yw_mm = 0.f;
	row->theta1_deg = 0.f;
	row->theta2_deg = 0.f;
	row->geom_ok = 0u;
	row->pulse_motor1_abs = 0;
	row->pulse_motor2_abs = 0;
}

static uint32_t crc32_ieee_regs_be(const uint16_t *regs, uint16_t nregs)
{
	uint32_t crc = 0xFFFFFFFFu;
	uint16_t ri;

	for (ri = 0u; ri < nregs; ri++)
	{
		uint32_t b;
		uint8_t  byte;
		uint16_t r = regs[ri];

		byte = (uint8_t)((r >> 8) & 0xFFu);
		crc ^= byte;
		for (b = 0u; b < 8u; b++)
		{
			crc = (crc >> 1) ^
			      (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
		}

		byte = (uint8_t)(r & 0xFFu);
		crc ^= byte;
		for (b = 0u; b < 8u; b++)
		{
			crc = (crc >> 1) ^
			      (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
		}
	}
	return ~crc;
}

/*
 * 16 字头 H[]：与首/尾 FC03 读 0x0000×16 解析顺序一致。
 * 成功返回 1；失败返回 0 并在 msg 写入 RunFlag 短句（地址 + 实值 + 期望）。
 */
static uint8_t hdr_gate_check(const uint16_t *h, const char *phase_tag,
			      char *msg, size_t msg_cap)
{
	const char *tag =
	    (phase_tag != NULL) ? phase_tag : "MAT HDR";

	if ((msg != NULL) && (msg_cap > 0u))
	{
		msg[0] = '\0';
	}
	if (h[0] != 1u)
	{
		if ((msg != NULL) && (msg_cap > 0u))
		{
			(void)snprintf(msg, msg_cap, "%s gate 0x%04X=%u need1",
				       tag,
				       (unsigned)(CFG_MATRIX_REG_HEADER_BASE + 0u),
				       (unsigned)h[0]);
		}
		return 0u;
	}
	if (h[1] != 0u)
	{
		if ((msg != NULL) && (msg_cap > 0u))
		{
			(void)snprintf(msg, msg_cap, "%s gate 0x%04X=%u need0",
				       tag,
				       (unsigned)(CFG_MATRIX_REG_HEADER_BASE + 1u),
				       (unsigned)h[1]);
		}
		return 0u;
	}
	if (h[13] != (uint16_t)MATRIX_EXPECTED_ROWS)
	{
		if ((msg != NULL) && (msg_cap > 0u))
		{
			(void)snprintf(msg, msg_cap, "%s gate 0x%04X=%u need%u",
				       tag,
				       (unsigned)(CFG_MATRIX_REG_HEADER_BASE + 13u),
				       (unsigned)h[13],
				       (unsigned)MATRIX_EXPECTED_ROWS);
		}
		return 0u;
	}
	if (h[15] != 3u)
	{
		if ((msg != NULL) && (msg_cap > 0u))
		{
			(void)snprintf(msg, msg_cap, "%s gate 0x%04X=%u need3",
				       tag,
				       (unsigned)(CFG_MATRIX_REG_HEADER_BASE + 15u),
				       (unsigned)h[15]);
		}
		return 0u;
	}
	return 1u;
}

static void copy_hdr_diag(const uint16_t *h)
{
	memcpy(s_hdr_diag, h, sizeof(s_hdr_diag));
}

static void mat_cycle_finish(const char *run_msg, uint8_t read_st,
			     uint8_t clr_commit_ok)
{
	BSP_USART3_TraceSetEnabled(0u);
	s_busy = 0u;
	s_phase = MBS_IDLE;
	s_chunk = 0u;
	s_read_status = read_st;
	if (clr_commit_ok != 0u)
	{
		s_last_commit_ok = 0u;
	}
	if (run_msg != NULL)
	{
		(void)strncpy(s_last_run_msg, run_msg, sizeof(s_last_run_msg) - 1u);
		s_last_run_msg[sizeof(s_last_run_msg) - 1u] = '\0';
		AppDisplay_SetRunFlagText(run_msg);
	}
	else
	{
		s_last_run_msg[0] = '\0';
	}
}

void AppMatrixModbus_Init(void)
{
	BSP_USART3_TraceSetEnabled(0u);
	s_phase = MBS_IDLE;
	s_busy = 0u;
	s_chunk = 0u;
	s_chunks_ok = 0u;
	s_snap_batch = 0u;
	s_snap_ctr = 0u;
	s_snap_crc32 = 0u;
	s_crc_fails = 0u;
	s_to_fails = 0u;
	s_len_fails = 0u;
	s_commits_ok = 0u;
	s_last_commit_ok = 0u;
	s_last_txn_err = 0u;
	s_diag_crc32_calc = 0u;
	s_diag_crc32_expect = 0u;
	s_last_run_msg[0] = '\0';
	s_read_status = APP_MAT_RD_ST_IDLE;
	s_rows_recv = 0u;
	memset(s_hdr_diag, 0, sizeof(s_hdr_diag));
}

void AppMatrixModbus_OnMatrixCleared(void)
{
	s_last_commit_ok = 0u;
}

uint8_t AppMatrixModbus_LastCommitOk(void)
{
	return s_last_commit_ok;
}

uint8_t AppMatrixModbus_IsBusy(void)
{
	return s_busy;
}

void AppMatrixModbus_GetLastRunMsg(char *dst, unsigned cap)
{
	if ((dst == NULL) || (cap == 0u))
	{
		return;
	}
	(void)strncpy(dst, s_last_run_msg, (size_t)cap - 1u);
	dst[cap - 1u] = '\0';
}

void AppMatrixModbus_GetDiag(AppMatrixModbusDiag_t *d)
{
	if (d == NULL)
	{
		return;
	}
	memset(d, 0, sizeof(*d));
	d->busy = s_busy;
	d->phase = (uint8_t)s_phase;
	d->chunk_done = s_chunks_ok;
	d->read_status = s_read_status;
	d->rows_received = s_rows_recv;
	d->last_txn_err = s_last_txn_err;
	d->hdr_ready = s_hdr_diag[0];
	d->hdr_updating = s_hdr_diag[1];
	d->hdr_batch = s_hdr_diag[2];
	d->hdr_upd_ctr = s_hdr_diag[3];
	d->hdr_count = s_hdr_diag[13];
	d->hdr_tray_total = s_hdr_diag[15];
	d->hdr_crc_hi = s_hdr_diag[9];
	d->hdr_crc_lo = s_hdr_diag[10];
	d->crc32_calc = s_diag_crc32_calc;
	d->crc32_expect = s_diag_crc32_expect;
	d->crc_fails = s_crc_fails;
	d->to_fails = s_to_fails;
	d->len_fails = s_len_fails;
	d->commits_ok = s_commits_ok;
	d->rx_ovf_u3 = BSP_USART3_RxOverflowCount();
}

uint8_t AppMatrixModbus_Key2Request(void)
{
	if (s_busy != 0u)
	{
		return 1u;
	}
	s_busy = 1u;
	s_phase = MBS_K2_HDR;
	s_chunk = 0u;
	s_chunks_ok = 0u;
	s_read_status = APP_MAT_RD_ST_READING;
	s_rows_recv = 0u;
	s_last_run_msg[0] = '\0';
	s_diag_crc32_calc = 0u;
	s_diag_crc32_expect = 0u;
	BSP_USART3_TraceReset();
	BSP_USART3_TraceSetEnabled(1u);
	AppDisplay_ResetU3HexScrollAccum();
	return 0u;
}

void AppMatrixModbus_Poll(uint32_t tick_ms)
{
	uint8_t py;

	(void)tick_ms;

	if (s_busy == 0u)
	{
		return;
	}

	switch (s_phase)
	{
	case MBS_K2_HDR : {
		uint8_t  dbuf[40];
		uint16_t dlen;
		uint16_t H[16];
		char     gf[96];

		py = mx_fc03_read(CFG_MATRIX_REG_HEADER_BASE, 16u, dbuf, &dlen);
		if (py != MB_MASTER_OK)
		{
#if CFG_MATRIX_K2_BYPASS_GATES
			memset(H, 0, sizeof(H));
			copy_hdr_diag(H);
			s_snap_batch = 0u;
			s_snap_ctr = 0u;
			s_snap_crc32 = 0u;
			s_diag_crc32_expect = 0u;
			AppDisplay_SetRunFlagText("MAT HDR MB bypass");
#else
			mat_cycle_finish("MAT HDR MB ERR", APP_MAT_RD_ST_ERR, 0u);
			return;
#endif
		}
		else
		{
			regs_bytes_to_u16(dbuf, H, 16u);
			copy_hdr_diag(H);
		}

		if (hdr_gate_check(H, "MAT HDR", gf, sizeof(gf)) == 0u)
		{
#if CFG_MATRIX_K2_BYPASS_GATES
			AppDisplay_SetRunFlagText("MAT HDR gate bypass");
#else
			mat_cycle_finish(gf, APP_MAT_RD_ST_ERR, 0u);
			return;
#endif
		}

		s_snap_batch = H[2];
		s_snap_ctr = H[3];
		s_snap_crc32 = ((uint32_t)H[9] << 16) | (uint32_t)H[10];
		s_diag_crc32_expect = s_snap_crc32;

		s_chunk = 0u;
		s_chunks_ok = 0u;
		s_rows_recv = 0u;
		s_phase = MBS_K2_DATA;
		return;
	}

	case MBS_K2_DATA : {
		uint8_t  dbuf[220];
		uint16_t dlen;
		char     de[96];
		uint16_t reg_addr =
		    (uint16_t)(CFG_MATRIX_REG_MATRIX_BASE +
			       (uint16_t)s_chunk * CFG_MATRIX_REGS_PER_PACKET);
		uint16_t off = (uint16_t)s_chunk * CFG_MATRIX_REGS_PER_PACKET;

		py = mx_fc03_read(reg_addr, CFG_MATRIX_REGS_PER_PACKET, dbuf, &dlen);
		if (py != MB_MASTER_OK)
		{
#if CFG_MATRIX_K2_BYPASS_GATES
			(void)snprintf(de, sizeof(de), "bypass DATA %u/%u err%u",
				       (unsigned)((uint16_t)s_chunk + 1u),
				       (unsigned)CFG_MATRIX_PACKET_COUNT,
				       (unsigned)py);
			AppDisplay_SetRunFlagText(de);
			memset(&s_matrix_regs[off], 0,
			       (size_t)CFG_MATRIX_REGS_PER_PACKET *
				   sizeof(s_matrix_regs[0]));
#else
			(void)snprintf(de, sizeof(de),
				       "MAT DATA pkt %u/%u @0x%04X err%u",
				       (unsigned)((uint16_t)s_chunk + 1u),
				       (unsigned)CFG_MATRIX_PACKET_COUNT,
				       (unsigned)reg_addr, (unsigned)py);
			mat_cycle_finish(de, APP_MAT_RD_ST_ERR, 0u);
			return;
#endif
		}
		else
		{
			regs_bytes_to_u16(dbuf, &s_matrix_regs[off],
					   CFG_MATRIX_REGS_PER_PACKET);
		}
		s_chunk++;
		s_chunks_ok = s_chunk;
		s_rows_recv = (uint16_t)(s_chunks_ok * CFG_MATRIX_CELLS_PER_PACKET);
		if (s_chunk >= CFG_MATRIX_PACKET_COUNT)
		{
			s_phase = MBS_K2_TAIL;
		}
		return;
	}

	case MBS_K2_TAIL : {
		uint8_t       dbuf[40];
		uint16_t      dlen;
		uint16_t      H[16];
		uint32_t      crcv;
		uint16_t      i;
		char          fb[40];
		char          gf[96];
		char          bc[96];
		uint8_t       wk;

		py = mx_fc03_read(CFG_MATRIX_REG_HEADER_BASE, 16u, dbuf, &dlen);
		if (py != MB_MASTER_OK)
		{
#if CFG_MATRIX_K2_BYPASS_GATES
			AppDisplay_SetRunFlagText("MAT TAIL rd bypass");
#else
			mat_cycle_finish("MAT TAIL MB ERR", APP_MAT_RD_ST_ERR, 0u);
			return;
#endif
		}
		else
		{
			regs_bytes_to_u16(dbuf, H, 16u);
			copy_hdr_diag(H);

#if !CFG_MATRIX_K2_BYPASS_GATES
			if ((H[2] != s_snap_batch) || (H[3] != s_snap_ctr))
			{
				(void)snprintf(bc, sizeof(bc),
					       "MAT BATCH bat %u>%u seq %u>%u",
					       (unsigned)s_snap_batch,
					       (unsigned)H[2],
					       (unsigned)s_snap_ctr,
					       (unsigned)H[3]);
				mat_cycle_finish(bc, APP_MAT_RD_ST_ERR, 0u);
				return;
			}
			if (hdr_gate_check(H, "MAT TAIL", gf, sizeof(gf)) == 0u)
			{
				mat_cycle_finish(gf, APP_MAT_RD_ST_ERR, 0u);
				return;
			}
#endif
		}

		crcv = crc32_ieee_regs_be(s_matrix_regs, MX_MATRIX_NREGS);
		s_diag_crc32_calc = crcv;
		s_diag_crc32_expect = s_snap_crc32;

#if !CFG_MATRIX_K2_BYPASS_GATES
		if (crcv != s_snap_crc32)
		{
			mat_cycle_finish("MAT CRC32 FAIL", APP_MAT_RD_ST_ERR, 0u);
			return;
		}
#endif

		for (i = 0u; i < MATRIX_EXPECTED_ROWS; i++)
		{
			fill_cell_from_u16(
			    &s_matrix_regs[(uint16_t)(i * CFG_MATRIX_REGS_PER_CELL)],
			    &s_build[i]);
		}

		wk = AppMatrix_SetFromTcpParser(s_build, MATRIX_EXPECTED_ROWS,
						MATRIX_EXPECTED_ROWS, fb,
						(uint16_t)sizeof(fb));
		if (wk == 0u)
		{
#if CFG_MATRIX_K2_BYPASS_GATES
			mat_cycle_finish("MAT BYPASS RBFAIL", APP_MAT_RD_ST_ERR, 1u);
#else
			mat_cycle_finish("MAT REBUILD FAIL", APP_MAT_RD_ST_ERR, 1u);
#endif
			return;
		}

		s_last_commit_ok = 1u;
		s_commits_ok++;
		s_rows_recv = MATRIX_EXPECTED_ROWS;
		AppProtocol_FreezeRawLastFromRows(s_build, MATRIX_EXPECTED_ROWS);
#if CFG_MATRIX_K2_BYPASS_GATES
		mat_cycle_finish("MAT OK BYPASS+RB", APP_MAT_RD_ST_OK, 0u);
#else
		mat_cycle_finish("MAT OK CRC+RB", APP_MAT_RD_ST_OK, 0u);
#endif
		return;
	}

	default :
		BSP_USART3_TraceSetEnabled(0u);
		s_busy = 0u;
		s_phase = MBS_IDLE;
		return;
	}
}
