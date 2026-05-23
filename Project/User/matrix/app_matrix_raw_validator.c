/**
 * app_matrix_raw_validator.c — Matrix_Raw（TCP）校验：满格 150 / 稀疏 1～149。
 */

#include "app_matrix_raw_validator.h"

#include <stdio.h>
#include <string.h>

#include "global_config.h"

static uint8_t s_last_flags;

/*
 * 功能：按协议把一行 ASCII（无 CRLF）逐字节累加到 checksum 累加器。
 * 交互：外部由 app_protocol 数据行分支调用。
 */
void AppMatrixRaw_ChecksumAccumulate(uint32_t *acc, const char *data_line_no_crlf)
{
	const char *p;

	if (acc == NULL || data_line_no_crlf == NULL)
	{
		return;
	}
	for (p = data_line_no_crlf; *p != '\0'; p++)
	{
		*acc += (uint32_t)(uint8_t)*p;
	}
}

/*
 * 功能：将累加结果对 65536 取模（与 END checksum 语义一致）。
 * 交互：外部测试或协议侧可选；与 AppMatrixRaw_ChecksumAccumulate 配合。
 */
uint32_t AppMatrixRaw_ChecksumMod65536(uint32_t acc)
{
	return acc % 65536u;
}

/*
 * 功能：托盘号+行列映射到线性 slot 索引 [0, EXPECTED_ROWS)；越界写 fb。
 * 交互：内部被满格/稀疏校验循环调用。
 */
static uint16_t tray_rowcol_to_slot_no(uint16_t tid, uint16_t cc, uint16_t rr,
				       char *fb, uint16_t fbcap)
{
	uint16_t cc0;
	uint16_t rr0;
	uint32_t slot;

	if (tid < 1u || tid > (uint16_t)MATRIX_TRAY_COUNT)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:tray");
		}
		return (uint16_t)MATRIX_EXPECTED_ROWS;
	}
	if (cc < (uint16_t)MATRIX_COL_ROW_BASE ||
	    cc >= (uint16_t)MATRIX_COL_ROW_BASE + (uint16_t)MATRIX_TRAY_COLS ||
	    rr < (uint16_t)MATRIX_COL_ROW_BASE ||
	    rr >= (uint16_t)MATRIX_COL_ROW_BASE + (uint16_t)MATRIX_TRAY_ROWS)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:c/r");
		}
		return (uint16_t)MATRIX_EXPECTED_ROWS;
	}

	cc0 = (uint16_t)(cc - (uint16_t)MATRIX_COL_ROW_BASE);
	rr0 = (uint16_t)(rr - (uint16_t)MATRIX_COL_ROW_BASE);

	slot = (uint32_t)((uint32_t)tid - 1u) *
		       ((uint32_t)MATRIX_TRAY_COLS * (uint32_t)MATRIX_TRAY_ROWS) +
	       (uint32_t)rr0 * (uint32_t)MATRIX_TRAY_COLS + (uint32_t)cc0;

	if (slot >= (uint32_t)MATRIX_EXPECTED_ROWS)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:slot");
		}
		return (uint16_t)MATRIX_EXPECTED_ROWS;
	}

	return (uint16_t)slot;
}

/*
 * 功能：校验 150 行满格：行列合法、无重复 slot、无空缺、class 合法。
 * 交互：外部由 AppMatrix_SetFromTcpParser 在 n==EXPECTED 时调用；失败写 fb。
 */
uint8_t AppMatrixRaw_ValidateFullGrid(const MatrixFinalRow_t *rows, uint16_t n,
				     char *fb, uint16_t fbcap)
{
	uint8_t  map[MATRIX_EXPECTED_ROWS];
	uint16_t i;
	uint16_t slots = (uint16_t)((uint16_t)MATRIX_TRAY_COLS * (uint16_t)MATRIX_TRAY_ROWS *
				   (uint16_t)MATRIX_TRAY_COUNT);

	if (slots != MATRIX_EXPECTED_ROWS)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:cfg slots");
		}
		return 0u;
	}
	if (n != MATRIX_EXPECTED_ROWS)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:size");
		}
		return 0u;
	}

	memset(map, 0, sizeof(map));

	for (i = 0u; i < n; i++)
	{
		uint16_t tid = rows[i].tray_id;
		uint16_t cc = rows[i].col;
		uint16_t rr = rows[i].row;
		uint16_t slot;

		if (rows[i].class_id > 2u)
		{
			if (fb && fbcap > 0u)
			{
				(void)snprintf(fb, fbcap, "MAT:cls");
			}
			return 0u;
		}

		slot = tray_rowcol_to_slot_no(tid, cc, rr, fb, fbcap);

		if (slot >= MATRIX_EXPECTED_ROWS)
		{
			return 0u;
		}
		if (map[slot] != 0u)
		{
			if (fb && fbcap > 0u)
			{
				(void)snprintf(fb, fbcap, "MAT:DUP slot");
			}
			return 0u;
		}

		map[slot] = 1u;
	}

	for (i = 0u; i < MATRIX_EXPECTED_ROWS; i++)
	{
		if (map[i] == 0u)
		{
			if (fb && fbcap > 0u)
			{
				(void)snprintf(fb, fbcap, "MAT:MISS cel");
			}
			return 0u;
		}
	}

	return 1u;
}

/*
 * 功能：校验稀疏帧：行数 1..EXPECTED-1，无重复 slot；不要求补满全盘。
 * 交互：外部由 AppMatrix_SetFromTcpParser 在非满格时调用。
 */
uint8_t AppMatrixRaw_ValidateSparseSlots(const MatrixFinalRow_t *rows, uint16_t n,
				       char *fb, uint16_t fbcap)
{
	uint8_t  map[MATRIX_EXPECTED_ROWS];
	uint16_t i;

	if (n == 0u || n >= MATRIX_EXPECTED_ROWS)
	{
		if (fb && fbcap > 0u)
		{
			(void)snprintf(fb, fbcap, "MAT:sparse sz");
		}
		return 0u;
	}

	memset(map, 0, sizeof(map));

	for (i = 0u; i < n; i++)
	{
		uint16_t tid = rows[i].tray_id;
		uint16_t cc = rows[i].col;
		uint16_t rr = rows[i].row;
		uint16_t slot;

		if (rows[i].class_id > 2u)
		{
			if (fb && fbcap > 0u)
			{
				(void)snprintf(fb, fbcap, "MAT:cls");
			}
			return 0u;
		}

		slot = tray_rowcol_to_slot_no(tid, cc, rr, fb, fbcap);

		if (slot >= MATRIX_EXPECTED_ROWS)
		{
			return 0u;
		}
		if (map[slot] != 0u)
		{
			if (fb && fbcap > 0u)
			{
				(void)snprintf(fb, fbcap, "MAT:DUP slot");
			}
			return 0u;
		}

		map[slot] = 1u;
	}

	return 1u;
}

/*
 * 功能：新一帧开始前清零 TCP/网格结果标志快照。
 * 交互：外部由 app_protocol START 分支调用。
 */
void AppMatrixRaw_ClearFrameFlags(void)
{
	s_last_flags = 0u;
}

/*
 * 功能：一帧收尾时根据 TCP checksum 与网格校验结果写入 s_last_flags。
 * 交互：外部由 app_protocol 成功 END、失败清零路径调用。
 */
void AppMatrixRaw_NotifyTcpFrameDone(uint8_t tcp_chk_ok, uint8_t grid_ok)
{
	s_last_flags = 0u;
	if (tcp_chk_ok != 0u)
	{
		s_last_flags |= APP_MATRIX_RAW_FL_TCP_CHK;
	}
	if (grid_ok != 0u)
	{
		s_last_flags |= APP_MATRIX_RAW_FL_GRID_OK;
	}
}

/*
 * 功能：读取上一帧标志位组合（checksum OK / grid OK）。
 * 交互：外部由测试注入、UI/测试展示调用。
 */
uint8_t AppMatrixRaw_GetLastFlags(void)
{
	return s_last_flags;
}
