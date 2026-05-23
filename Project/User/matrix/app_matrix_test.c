/**
 * app_matrix_test.c — 合成 START + 150 行 + END，穴位 Fisher–Yates 打乱，UVZ 在协议允许范围内
 */
#include "app_matrix_test.h"

#include "app_protocol.h"

#include "global_config.h"
#include "delay.h"

#include <stdio.h>
#include <string.h>

#define SLOTS_PER_TRAY  ((unsigned)MATRIX_TRAY_COLS * (unsigned)MATRIX_TRAY_ROWS)

/*
 * 功能：将一行文本经 AppProtocol_OnStream 送入解析器并补换行。
 * 交互：内部仅被本文件合成测试帧使用。
 */
static void feed_line(const char *s)
{
	size_t L;

	if (s == NULL) {
		return;
	}
	L = strlen(s);
	if (L > 0u) {
		AppProtocol_OnStream((const uint8_t *)s, (uint16_t)L);
	}
	AppProtocol_OnStream((const uint8_t *)"\n", 1u);
}

/*
 * 功能：Fisher–Yates 打乱 slot 排列，种子与 SysTick 混合防固定序列。
 * 交互：内部由 AppMatrixTest_InjectRandomViaProtocol 调用。
 */
static void shuffle_perm(uint16_t *perm, uint32_t seed)
{
	uint16_t i;
	uint32_t z = seed ^ (uint32_t)SysTick_GetMs();

	for (i = 0u; i < (uint16_t)MATRIX_EXPECTED_ROWS; i++) {
		perm[i] = i;
	}
	for (i = (uint16_t)MATRIX_EXPECTED_ROWS; i > 1u; i--) {
		uint16_t j;
		uint16_t tmp;

		z = z * 1664525u + 1013904223u;
		j = (uint16_t)(z % (uint32_t)i);
		tmp = perm[i - 1u];
		perm[i - 1u] = perm[j];
		perm[j] = tmp;
	}
}

/*
 * 功能：生成合法满格 Matrix_Raw（START+150 行+END），经与 TCP 相同协议路径注入。
 * 交互：外部由测试/调试入口调用；调用 AppProtocol_Init/OnStream/GetDataLinesChecksumAccumulator。
 */
void AppMatrixTest_InjectRandomViaProtocol(uint32_t seed)
{
	char          line[CFG_APP_PROTO_RAW_LINE_CAP];
	uint16_t      perm[MATRIX_EXPECTED_ROWS];
	unsigned int  k;
	unsigned int  f;

	AppProtocol_Init();

	for (f = 0u; f < (unsigned)MATRIX_SAMPLE_WINDOW_FRAMES; f++) {
		(void)snprintf(line, sizeof(line), "START,count=%u",
			       (unsigned)MATRIX_EXPECTED_ROWS);
		feed_line(line);

		shuffle_perm(perm, seed + f * 997u);

		for (k = 0u; k < (unsigned)MATRIX_EXPECTED_ROWS; k++) {
			uint16_t slot = perm[k];
			uint16_t tray = (uint16_t)(slot / SLOTS_PER_TRAY) + 1u;
			uint16_t rem = (uint16_t)(slot % SLOTS_PER_TRAY);
			uint16_t rr = (uint16_t)(rem / (uint16_t)MATRIX_TRAY_COLS);
			uint16_t cc = (uint16_t)(rem % (uint16_t)MATRIX_TRAY_COLS);
			/* TCP 正文：列/行为 1 基索引 */
			unsigned col1 =
				(unsigned)cc +
				(unsigned)(uint16_t)MATRIX_COL_ROW_BASE;
			unsigned row1 =
				(unsigned)rr +
				(unsigned)(uint16_t)MATRIX_COL_ROW_BASE;
			uint32_t mix = seed * 1664525u + 1013904223u +
				      (uint32_t)slot * 7u +
				      f * 13u;
			uint16_t cls = (uint16_t)(mix % 3u);
			int32_t  u = (int32_t)(4000 + (int)tray * 10 + (int)cc * 50 +
					      (int)(mix % 31u));
			int32_t  v = (int32_t)(3000 + (int)rr * 20 + (int)((mix >> 8) % 29u));
			int32_t  z = (int32_t)(1800);
			/* 小数置信度 ×100 四舍五入后与 TCP 路径一致 */
			double conf_f = 85.0 + (double)tray * 0.3;

			if (conf_f * 100.0 > (double)MATRIX_CONFIDENCE_MAX)
				conf_f =
					(double)MATRIX_CONFIDENCE_MAX / 100.0;
			(void)snprintf(
				line, sizeof(line),
				"%u,%u,%u,%u,%.4f,%ld,%ld,%ld",
				(unsigned)tray, col1, row1, (unsigned)cls,
				conf_f, (long)u, (long)v, (long)z);
			feed_line(line);
		}

		{
			uint32_t acc =
				AppProtocol_GetDataLinesChecksumAccumulator() % 65536u;

			(void)snprintf(line, sizeof(line), "END checksum=%lu",
				       (unsigned long)acc);
			feed_line(line);
			/* checksum 不包含；必须等于 MATRIX_TRAY_COUNT，否则丢弃整帧 */
			(void)snprintf(line, sizeof(line), "tray_total=%u",
				       (unsigned)MATRIX_TRAY_COUNT);
			feed_line(line);
		}
	}
}
