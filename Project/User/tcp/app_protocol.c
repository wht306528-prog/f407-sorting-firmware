/**
 * app_protocol.c — Matrix_Raw 文本帧解析；量产主路径为 Modbus + `AppProtocol_FreezeRawLastFromRows`。
 *
 * 严格语义：非法行/超长行/CSV 列不齐/数值越界 → 本帧废弃，不写入矩阵（并清空旧快照）。
 * checksum：各数据行（不含行尾 CRLF/LF）的每个 ASCII 字节累加，对 65536 取模。
 *
 * `AppProtocol_OnStream`：仅测试或串口桥按字节注入；量产不依赖。
 *
 * Modbus 成功后（app_matrix_modbus）调用 AppProtocol_FreezeRawLastFromRows：按与 Matrix_Raw CSV 相同列填充 s_raw_last_*。
 * s_raw_last_chk_ok/grid_ok 置 1 表示「快照有效」，非 OnStream END 行 ASCII 校验。
 *
 * 变量速查：s_last_ok=择优提交后是否有效；未满窗候选组装过程中恒为 0。AppSort 在未提交前不得沿用旧 true。
 * MATRIX_SAMPLE_WINDOW_FRAMES=1 为默认；CFG_MATRIX_KEEP_RX_OPEN_AFTER_OK=1 时提交后仍开窗。
 */
#include "app_protocol.h"
#include "app_matrix.h"
#include "app_matrix_raw_validator.h"
#include "app_display.h"

#include "global_config.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

typedef enum
{
	PST_IDLE = 0, /* 等 START */
	PST_BODY,     /* CSV 正文 */
	PST_WAIT_TRAY_TOTAL /*!< END checksum OK，等待 tray_total= */
} proto_state_t;

static proto_state_t  s_st;
static char           s_line[CFG_APP_PROTO_RAW_LINE_CAP];
static uint16_t       s_ll;
static uint8_t        s_line_overflow;

static MatrixFinalRow_t s_rows[MATRIX_MAX_ROWS];
static uint16_t       s_nr;
static uint16_t       s_decl_cnt;
static uint32_t       s_chk_acc;
static uint8_t        s_last_ok;

/** 注入解析器的载荷字节累计（连接建立后 AppProtocol_Init 清零） */
static uint32_t       s_stream_rx_bytes;
/** 择优矩阵完整提交成功次数（同连接累计，Init 清零） */
static uint32_t       s_matrix_frames_ok_count;

/** 当前帧正在组装的 Raw CSV 行快照（与 s_rows 并行增长） */
static char           s_raw_build[MATRIX_MAX_ROWS][CFG_APP_PROTO_RAW_LINE_CAP];
static uint16_t       s_raw_build_cnt;

/** 上一帧冻结结果（供 LCD 浏览）；不因 proto_reset 丢失 */
static char           s_raw_last[MATRIX_MAX_ROWS][CFG_APP_PROTO_RAW_LINE_CAP];
static uint16_t       s_raw_last_cnt;
static uint16_t       s_raw_last_declared;
static uint8_t        s_raw_last_chk_ok;
static uint8_t        s_raw_last_grid_ok;

/** 采样窗口候选（仅存解析后行向量，冻结 Raw 由提交时按字段重建） */
typedef struct
{
	MatrixFinalRow_t rows[MATRIX_MAX_ROWS];
	uint16_t         n_rows;
	uint16_t         declared;
	uint16_t         nonempty;
	uint32_t         sum_conf;
	uint8_t          seq;
} sample_candidate_t;

static uint8_t             s_gate_open; /*!< 1=OnStream 开窗收字节；择优提交后可置 0 */
static uint8_t             s_valid_count; /*!< 当前窗口内已收合法帧数 */
static sample_candidate_t  s_cand[MATRIX_SAMPLE_WINDOW_FRAMES];

/*
 * 功能：将 CSV 源行安全拷贝到 Raw 快照缓冲（带尾零 cap）。
 * 交互：内部被 raw_copy路径、CSV 分支调用；仅用 strncpy。
 */
static void raw_copy_line_cap(char *dst, const char *src)
{
	if (dst == NULL || src == NULL)
	{
		return;
	}
	(void)strncpy(dst, src, CFG_APP_PROTO_RAW_LINE_CAP - 1u);
	dst[CFG_APP_PROTO_RAW_LINE_CAP - 1u] = '\0';
}

/*
 * 功能：把当前帧组装的 Raw 行复制到 s_raw_last_*，并记录校验/网格结果供 LCD。
 * 交互：内部被 line_finish 成功/失败冻结、protocol_fail_clear_matrix 调用；写全局 s_raw_last_*。
 */
static void raw_freeze_snapshot(uint8_t chk_ok, uint8_t grid_ok)
{
	uint16_t i;
	uint16_t n = s_raw_build_cnt;

	if (n > MATRIX_MAX_ROWS)
	{
		n = MATRIX_MAX_ROWS;
	}
	for (i = 0u; i < n; i++)
	{
		memcpy(s_raw_last[i], s_raw_build[i], CFG_APP_PROTO_RAW_LINE_CAP);
	}
	s_raw_last_cnt = n;
	s_raw_last_declared = s_decl_cnt;
	s_raw_last_chk_ok = chk_ok;
	s_raw_last_grid_ok = grid_ok;
}

/*
 * 功能：从择优索引重建 Raw 冻结行（供 LCD），与 Matrix_Raw CSV 列一致。
 * 交互：满窗择优提交成功后调用；写 s_raw_last_*。
 */
static void raw_rebuild_freeze_from_winner(uint8_t w, uint8_t chk_ok,
					   uint8_t grid_ok)
{
	uint16_t i;
	uint16_t n;

	if (w >= MATRIX_SAMPLE_WINDOW_FRAMES)
	{
		return;
	}
	n = s_cand[w].n_rows;
	if (n > MATRIX_MAX_ROWS)
	{
		n = MATRIX_MAX_ROWS;
	}
	for (i = 0u; i < n; i++)
	{
		const MatrixFinalRow_t *r = &s_cand[w].rows[i];

		(void)snprintf(s_raw_last[i], CFG_APP_PROTO_RAW_LINE_CAP,
			       "%u,%u,%u,%u,%.3f,%d,%d,%d",
			       (unsigned)r->tray_id, (unsigned)r->col,
			       (unsigned)r->row, (unsigned)r->class_id,
			       (double)r->confidence / 100.0, (int)r->u,
			       (int)r->v, (int)r->z_mm);
	}
	s_raw_last_cnt = n;
	s_raw_last_declared = s_cand[w].declared;
	s_raw_last_chk_ok = chk_ok;
	s_raw_last_grid_ok = grid_ok;
}

/*
 * 功能：在满窗候选中选最优：非空穴位最多 → sum_conf/n_rows 更大 → seq 更新。
 * 交互：line_finish 收满合法帧时调用。
 */
static uint8_t pick_best_candidate_index(void)
{
	uint8_t best;
	uint8_t i;

	best = 0u;
	for (i = 1u; i < MATRIX_SAMPLE_WINDOW_FRAMES; i++)
	{
		if (s_cand[i].nonempty > s_cand[best].nonempty)
		{
			best = i;
			continue;
		}
		if (s_cand[i].nonempty < s_cand[best].nonempty)
		{
			continue;
		}
		{
			uint32_t av_i;
			uint32_t av_b;

			if (s_cand[i].n_rows == 0u)
			{
				continue;
			}
			if (s_cand[best].n_rows == 0u)
			{
				best = i;
				continue;
			}
			av_i = s_cand[i].sum_conf * 1000u /
			       (uint32_t)s_cand[i].n_rows;
			av_b = s_cand[best].sum_conf * 1000u /
			       (uint32_t)s_cand[best].n_rows;
			if (av_i > av_b)
			{
				best = i;
				continue;
			}
			if (av_i < av_b)
			{
				continue;
			}
		}
		if (s_cand[i].seq > s_cand[best].seq)
		{
			best = i;
		}
	}
	return best;
}

/*
 * 功能：复位「收帧传输层」变量（状态机 IDLE、清空行缓冲与临时行缓存），不改 s_raw_last_*。
 * 交互：内部被 AppProtocol_Init、line_finish END 成功末尾、protocol_fail_clear_matrix 调用。
 */
static void proto_reset_transport_state(void)
{
	s_st = PST_IDLE;
	s_ll = 0u;
	s_line_overflow = 0u;
	s_nr = 0u;
	s_chk_acc = 0u;
	s_decl_cnt = 0u;
	s_raw_build_cnt = 0u;
	memset(s_rows, 0, sizeof(s_rows));
}

/*
 * 功能：帧失败路径：清零校验 OK、可选冻结 Raw、复位传输层、清空矩阵并写 LCD 故障。
 * 交互：内部被 line_finish 各错误分支调用；调用 AppMatrixRaw_NotifyTcpFrameDone、raw_freeze_snapshot、AppMatrix_Clear、AppDisplay_SetFaultText。
 */
static void protocol_fail_clear_matrix(const char *txt)
{
	s_last_ok = 0u;
	AppMatrixRaw_NotifyTcpFrameDone(0u, 0u);
	/* 若在 BODY 阶段已有 Raw 行，冻结以便屏上排查（checksum/grid 视为失败） */
	if (((s_st == PST_BODY) || (s_st == PST_WAIT_TRAY_TOTAL)) &&
	    (s_raw_build_cnt > 0u))
	{
		raw_freeze_snapshot(0u, 0u);
	}
	proto_reset_transport_state();
	AppMatrix_Clear();
	AppDisplay_SetFaultText(txt);
}

/*
 * 功能：解析无符号十进制串为 u32，要求整串消费完毕。
 * 交互：内部被 parse_dec_u16_full、parse_end_checksum_value 等调用。
 */
static uint8_t parse_dec_u32_full(const char *s, uint32_t *out)
{
	uint32_t acc = 0u;

	if (s == NULL || *s == '\0')
	{
		return 0u;
	}
	while (*s >= '0' && *s <= '9')
	{
		uint32_t d = (uint32_t)(*s++ - '0');
		if (acc > (0xFFFFFFFFu - d) / 10u)
		{
			return 0u;
		}
		acc = acc * 10u + d;
	}
	if (*s != '\0')
	{
		return 0u;
	}
	*out = acc;
	return 1u;
}

/*
 * 功能：解析无符号十进制串为 u16（基于 parse_dec_u32_full 并检查 ≤0xFFFF）。
 * 交互：内部被 START count、CSV 字段解析使用。
 */
static uint8_t parse_dec_u16_full(const char *s, uint16_t *out)
{
	uint32_t v;

	if (!parse_dec_u32_full(s, &v) || v > 0xFFFFu)
	{
		return 0u;
	}
	*out = (uint16_t)v;
	return 1u;
}

/*
 * 功能：解析有符号十进制串为 s32，支持前导负号，防溢出。
 * 交互：内部被 parse_body_csv_split 填 MatrixFinalRow 的 u/v/z。
 */
static uint8_t parse_dec_s32_full(const char *s, int32_t *out)
{
	const char *t = s;
	uint8_t     neg = 0u;
	uint64_t    mag = 0u;

	if (s == NULL || *s == '\0')
	{
		return 0u;
	}
	if (*t == '-')
	{
		neg = 1u;
		t++;
		if (*t == '\0')
		{
			return 0u;
		}
	}
	while (*t != '\0')
	{
		uint8_t d;

		if (*t < '0' || *t > '9')
		{
			return 0u;
		}
		d = (uint8_t)(*t - '0');
		mag = mag * 10u + (uint64_t)d;
		if (!neg && mag > 0x7FFFFFFFULL)
		{
			return 0u;
		}
		if (neg && mag > 0x80000000ULL)
		{
			return 0u;
		}
		t++;
	}
	*out = neg ? (int32_t)(-(int64_t)mag) : (int32_t)mag;
	return 1u;
}

/*
 * 功能：从 START 行中提取 count=N 并写入全局 s_decl_cnt（1..MATRIX_MAX_ROWS）。
 * 交互：内部被 line_finish START 分支调用；依赖 parse_dec_u16_full。
 */
static uint8_t parse_start_count_loose(const char *line)
{
	const char *pc = strstr(line, "count=");

	s_decl_cnt = 0u;
	if (pc == NULL)
	{
		return 0u;
	}
	return parse_dec_u16_full(pc + 6u, &s_decl_cnt) &&
	       (s_decl_cnt >= 1u) && (s_decl_cnt <= MATRIX_MAX_ROWS);
}

/*
 * 功能：从 END 行解析 checksum=期望值（ASCII 累加模 65536）。
 * 交互：内部被 line_finish END 分支调用。
 */
static uint8_t parse_end_checksum_value(const char *line, uint32_t *expect)
{
	const char *pk = strstr(line, "checksum=");

	if (pk == NULL)
	{
		return 0u;
	}
	return parse_dec_u32_full(pk + 9u, expect);
}

static uint8_t parse_ascii_double_trim(const char *s, double *dv)
{
	char       *ep;
	double      v;

	if (s == NULL || *s == '\0' || dv == NULL)
	{
		return 0u;
	}
	v = strtod(s, &ep);
	if (ep == s)
	{
		return 0u;
	}
	while (*ep != '\0' && (*ep == ' ' || *ep == '\t'))
	{
		ep++;
	}
	if (*ep != '\0')
	{
		return 0u;
	}
	*dv = v;
	return 1u;
}

static long iround_bounded(double x, double lo, double hi)
{
	if (x < lo)
	{
		x = lo;
	}
	if (x > hi)
	{
		x = hi;
	}
	return (long)round(x);
}

/*
 * 功能：CSV 正文行：小数解析后置信度 ×100 四舍五入，行列 1..N、u,v,z 整数化（1-base 行列）。
 */
static uint8_t parse_body_csv_split(char *wb, MatrixFinalRow_t *row)
{
	const char *fields[8];
	char       *cursor = wb;
	uint16_t    i;
	MatrixFinalRow_t tmp;
	memset(&tmp, 0, sizeof(tmp));

	for (i = 0u; i < 7u; i++)
	{
		char *comma = strchr(cursor, ',');

		if (comma == NULL)
		{
			return 0u;
		}
		*comma = '\0';
		fields[i] = cursor;
		cursor = comma + 1u;
	}
	if (strchr(cursor, ',') != NULL)
	{
		return 0u;
	}
	fields[7] = cursor;

	{
		double   dv;
		long     lg;
		int32_t   si;
		uint16_t tray16;

		if (!parse_ascii_double_trim(fields[0], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(dv, 1.0, (double)MATRIX_TRAY_COUNT);
		tray16 = (uint16_t)lg;

		tmp.tray_id = tray16;

		if (!parse_ascii_double_trim(fields[1], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(
			dv, (double)MATRIX_COL_ROW_BASE,
			(double)(MATRIX_COL_ROW_BASE + MATRIX_TRAY_COLS -
				 1u));
		tmp.col = (uint16_t)lg;

		if (!parse_ascii_double_trim(fields[2], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(
			dv, (double)MATRIX_COL_ROW_BASE,
			(double)(MATRIX_COL_ROW_BASE + MATRIX_TRAY_ROWS -
				 1u));
		tmp.row = (uint16_t)lg;

		if (!parse_ascii_double_trim(fields[3], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(dv, 0.0, 2.0);
		tmp.class_id = (uint16_t)lg;
		if (tmp.class_id > 2u)
		{
			return 0u;
		}

		if (!parse_ascii_double_trim(fields[4], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(dv * 100.0, 0.0,
				     (double)MATRIX_CONFIDENCE_MAX);
		tmp.confidence = (uint16_t)lg;

		if (!parse_ascii_double_trim(fields[5], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(
			dv, -(double)(MATRIX_UV_ABS_MAX),
			(double)(MATRIX_UV_ABS_MAX));
		si = (int32_t)lg;
		tmp.u = si;

		if (!parse_ascii_double_trim(fields[6], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(
			dv, -(double)(MATRIX_UV_ABS_MAX),
			(double)(MATRIX_UV_ABS_MAX));
		tmp.v = (int32_t)lg;

		if (!parse_ascii_double_trim(fields[7], &dv))
		{
			return 0u;
		}
		lg = iround_bounded(
			dv, (double)(MATRIX_Z_MM_MIN),
			(double)(MATRIX_Z_MM_MAX));
		tmp.z_mm = (int32_t)lg;
	}

	*row = tmp;
	return 1u;
}

/*
 * 功能：处理一行完整文本（START / END / 数据 CSV）：维护状态机、校验 checksum、写入矩阵。
 * 交互：内部被 AppProtocol_OnStream 遇换行触发；调用 parse_*、protocol_fail_clear_matrix、AppMatrix_*、AppMatrixRaw_*、raw_freeze_snapshot、proto_reset_transport_state。
 */
static void line_finish(void)
{
	uint32_t expect_chk;

	if (s_ll == 0u)
	{
		return;
	}

	if (s_line_overflow != 0u)
	{
		uint8_t in_body = (uint8_t)(s_st == PST_BODY ? 1u : 0u);

		s_line_overflow = 0u;
		s_ll = 0u;
		if (in_body != 0u)
		{
			protocol_fail_clear_matrix("RAW:LINE OVFL");
			return;
		}
		return;
	}

	s_line[s_ll] = '\0';
	s_ll = 0u;

	if (strncmp(s_line, "START", 5) == 0)
	{
		/*
		 * 新帧开始：立即作废上一帧的 checksum 状态并清矩阵；
		 * 避免 OnStream 收帧中途 sort_matrix_ready_strict 仍认为旧矩阵有效。
		 */
		s_last_ok = 0u;
		AppMatrix_Clear();

		AppMatrixRaw_ClearFrameFlags();
		s_st = PST_BODY;
		s_nr = 0u;
		s_chk_acc = 0u;
		s_raw_build_cnt = 0u;
		memset(s_rows, 0, sizeof(s_rows));

		if (!parse_start_count_loose(s_line))
		{
			protocol_fail_clear_matrix("RAW:BAD START");
			return;
		}
		return;
	}

	if (strncmp(s_line, "END", 3) == 0)
	{
		uint8_t chk_ok;

		chk_ok = parse_end_checksum_value(s_line, &expect_chk);

		if (s_st != PST_BODY)
		{
			protocol_fail_clear_matrix("RAW:END no body");
			return;
		}
		if (!chk_ok)
		{
			protocol_fail_clear_matrix("RAW:BAD chk fmt");
			return;
		}
		if ((s_chk_acc % 65536u) != (expect_chk % 65536u))
		{
			protocol_fail_clear_matrix("RAW:CRC ERR");
			return;
		}
		if (s_decl_cnt != s_nr)
		{
			protocol_fail_clear_matrix("RAW:decl!=rows");
			return;
		}

		AppMatrixRaw_NotifyTcpFrameDone(1u, 0u);
		s_last_ok = 0u;
		s_st = PST_WAIT_TRAY_TOTAL;
		return;
	}

	if (s_st == PST_WAIT_TRAY_TOTAL)
	{
		const char *pk = strstr(s_line, "tray_total=");
		uint32_t    tty32;

		if (pk == NULL)
		{
			protocol_fail_clear_matrix("MAT:no trayttl");
			return;
		}
		if (!parse_dec_u32_full(pk + 11u, &tty32))
		{
			protocol_fail_clear_matrix("MAT:tray fmt");
			return;
		}
		if ((uint16_t)tty32 != (uint16_t)MATRIX_TRAY_COUNT)
		{
			protocol_fail_clear_matrix("MAT:tray cnt");
			return;
		}
		{
			char    failbuf[24];

			if (!AppMatrix_SetFromTcpParser(s_rows, s_nr, s_decl_cnt,
							failbuf,
							sizeof(failbuf)))
			{
				s_last_ok = 0u;
				AppMatrixRaw_NotifyTcpFrameDone(1u, 0u);
				AppDisplay_SetFaultText(failbuf);
				raw_freeze_snapshot(1u, 0u);
			}
			else
			{
				/* 采样窗口：连续收满合法帧后择优一次提交 */
				sample_candidate_t *dst;
				uint16_t          ii;

				if (s_valid_count >= MATRIX_SAMPLE_WINDOW_FRAMES)
				{
					protocol_fail_clear_matrix("RAW:sample OVFL");
					return;
				}
				dst = &s_cand[s_valid_count];
				dst->n_rows = s_nr;
				dst->declared = s_decl_cnt;
				dst->seq = s_valid_count;
				dst->nonempty = 0u;
				dst->sum_conf = 0u;
				for (ii = 0u; ii < s_nr; ii++)
				{
					dst->rows[ii] = s_rows[ii];
					if (s_rows[ii].class_id != 0u)
					{
						dst->nonempty++;
					}
					dst->sum_conf += (uint32_t)s_rows[ii].confidence;
				}
				s_valid_count++;
				AppMatrixRaw_NotifyTcpFrameDone(1u, 0u);
				s_last_ok = 0u;
				AppMatrix_Clear();
				if (s_valid_count < MATRIX_SAMPLE_WINDOW_FRAMES)
				{
					char sm[28];

					(void)snprintf(sm, sizeof(sm), "RAW:sample %u/%u",
						       (unsigned)s_valid_count,
						       (unsigned)MATRIX_SAMPLE_WINDOW_FRAMES);
					AppDisplay_SetFaultText(sm);
					proto_reset_transport_state();
					return;
				}
				{
					uint8_t w = pick_best_candidate_index();

					if (!AppMatrix_SetFromTcpParser(s_cand[w].rows,
									s_cand[w].n_rows,
									s_cand[w].declared,
									failbuf,
									sizeof(failbuf)))
					{
						s_last_ok = 0u;
						AppMatrixRaw_NotifyTcpFrameDone(1u, 0u);
						raw_rebuild_freeze_from_winner(w, 1u, 0u);
						AppDisplay_SetFaultText(failbuf);
					}
					else
					{
						s_last_ok = 1u;
						s_matrix_frames_ok_count++;
						AppMatrixRaw_NotifyTcpFrameDone(1u, 1u);
						raw_rebuild_freeze_from_winner(w, 1u, 1u);
						AppDisplay_SetFaultText("RAW:matrix OK");
					}
#if CFG_MATRIX_KEEP_RX_OPEN_AFTER_OK
					AppProtocol_ReopenGateKeepLastOk();
#else
					s_gate_open = 0u;
					s_valid_count = 0u;
					(void)memset(s_cand, 0, sizeof(s_cand));
#endif
				}
			}
		}
		proto_reset_transport_state();
		return;
	}

	if (s_st != PST_BODY || s_nr >= MATRIX_MAX_ROWS)
	{
		return;
	}

	{
		char             wb[CFG_APP_PROTO_RAW_LINE_CAP];
		MatrixFinalRow_t one;
		size_t           L;

		if (s_raw_build_cnt >= MATRIX_MAX_ROWS)
		{
			protocol_fail_clear_matrix("RAW:OVFL");
			return;
		}

		L = strlen(s_line);
		if (L >= sizeof(wb))
		{
			protocol_fail_clear_matrix("RAW:LINE LONG");
			return;
		}

		AppMatrixRaw_ChecksumAccumulate(&s_chk_acc, s_line);
		raw_copy_line_cap(s_raw_build[s_raw_build_cnt], s_line);

		memcpy(wb, s_line, L + 1u);
		if (!parse_body_csv_split(wb, &one))
		{
			s_raw_build_cnt++;
			protocol_fail_clear_matrix("RAW:CSV BAD");
			return;
		}
		s_rows[s_nr] = one;
		s_raw_build_cnt++;
		s_nr++;
	}
}

/*
 * 功能：向当前行缓冲追加一个字符，超长置 overflow 标记。
 * 交互：内部被 AppProtocol_OnStream 连续字节路径调用。
 */
static void line_push_char(char c)
{
	if (s_line_overflow != 0u)
	{
		return;
	}
	if (s_ll + 1u >= sizeof(s_line))
	{
		s_line_overflow = 1u;
		return;
	}
	s_line[s_ll++] = (char)c;
}

/*
 * 功能：复位协议解析器为新连接/服务端启动态；保留上一帧 Raw 冻结供 LCD。
 * 交互：外部被 AppTcpMatrix_Init、matrix_accept、matrix_recv FIN、业务测试调用。
 */
void AppProtocol_Init(void)
{
	s_last_ok = 0u;
	s_stream_rx_bytes = 0u;
	s_matrix_frames_ok_count = 0u;
	s_gate_open = 1u;
	s_valid_count = 0u;
	(void)memset(s_cand, 0, sizeof(s_cand));
	/* 保留 s_raw_last_*：新客户端接入时仍可查看上一帧 Raw/Final 屏显 */
	proto_reset_transport_state();
	s_line_overflow = 0u;
}

/*
 * 功能：流式接收字节并按换行符组行送入 line_finish（OnStream）。
 * 交互：外部被 app_tcp_server.matrix_recv 调用（可经分拣关闸静默丢弃）。
 */
void AppProtocol_OnStream(const uint8_t *data, uint16_t len)
{
	uint16_t i; /* 本次传入缓冲的下标 */

	if (data == NULL) { /* 守势：lwIP 不应传空，防以后误用 */
		return;
	}
	s_stream_rx_bytes += (uint32_t)len;
	for (i = 0u; i < len; i++) { /* 按字节推进：兼容拆包/粘包 */
		uint8_t cc = data[i]; /* 当前 ASCII / 控制字节 */

		if (cc == '\r') { /* Windows 行分隔的第一字节：忽略 */
			continue;
		}
		if (cc == '\n') { /* Unix 行结束：触发 line_finish */
			line_finish();
			continue;
		}
		line_push_char((char)cc); /* 正文进 s_line */
	}
}

/*
 * 功能：返回上一完整帧是否在 END checksum 语义下成功（s_last_ok）。
 * 交互：外部被 app_sort.matrix_ready_strict 等查询 OnStream/快照可信度。
 */
uint8_t AppProtocol_LastChecksumOk(void)
{
	return s_last_ok;
}

/*
 * 功能：返回冻结的上一帧 Raw 行数（供屏显/调试）。
 * 交互：外部被 UI 或测试读取矩阵原始行快照。
 */
uint16_t AppProtocol_GetRawLastLineCount(void)
{
	return s_raw_last_cnt;
}

/*
 * 功能：拷贝冻结 Raw 指定行到调用方缓冲。
 * 交互：外部被 app_display 等；越界返回 0。
 */
uint8_t AppProtocol_GetRawLastLine(uint16_t idx, char *dst, uint16_t cap)
{
	if ((dst == NULL) || (cap == 0u) || (idx >= s_raw_last_cnt))
	{
		return 0u;
	}
	(void)strncpy(dst, s_raw_last[idx], cap - 1u);
	dst[cap - 1u] = '\0';
	return 1u;
}

/*
 * 功能：返回上一冻结帧 START 声称的行数。
 * 交互：外部 UI/调试对照 decl 与实际行。
 */
uint16_t AppProtocol_GetRawLastDeclaredRows(void)
{
	return s_raw_last_declared;
}

/*
 * 功能：返回上一冻结帧 END 行 ASCII checksum 是否通过（与矩阵 grid 分离）。
 * 交互：外部被 matrix_test 标志组合等使用。
 */
uint8_t AppProtocol_GetRawLastChecksumOk(void)
{
	return s_raw_last_chk_ok;
}

/*
 * 功能：返回上一冻结帧网格/语义校验是否通过（AppMatrix_SetFromTcpParser 成功）。
 * 交互：外部与 checksum 并列显示。
 */
uint8_t AppProtocol_GetRawLastGridOk(void)
{
	return s_raw_last_grid_ok;
}

/*
 * 功能：调试：返回当前帧已累加的数据行 checksum（未 END 时为进行中值）。
 * 交互：可选被测试/屏显调用。
 */
uint32_t AppProtocol_GetDataLinesChecksumAccumulator(void)
{
	return s_chk_acc;
}

uint8_t AppProtocol_ShouldAcceptStream(void)
{
	return s_gate_open;
}

void AppProtocol_ArmSampleWindow(void)
{
	s_gate_open = 1u;
	s_valid_count = 0u;
	(void)memset(s_cand, 0, sizeof(s_cand));
	s_last_ok = 0u;
}

void AppProtocol_ReopenGateKeepLastOk(void)
{
	s_gate_open = 1u;
	s_valid_count = 0u;
	(void)memset(s_cand, 0, sizeof(s_cand));
}

uint8_t AppProtocol_IsGateOpen(void)
{
	return s_gate_open;
}

uint8_t AppProtocol_GetSampleValidCount(void)
{
	return s_valid_count;
}

uint32_t AppProtocol_GetStreamRxBytes(void)
{
	return s_stream_rx_bytes;
}

uint32_t AppProtocol_GetMatrixFramesOkCount(void)
{
	return s_matrix_frames_ok_count;
}

/*
 * 见 app_protocol.h：Modbus 成功后填充 Raw 子页 CSV 快照。
 */
void AppProtocol_FreezeRawLastFromRows(const MatrixFinalRow_t *rows,
				       uint16_t n_rows)
{
	uint16_t i;
	uint16_t n = n_rows;

	if (rows == NULL)
	{
		return;
	}
	if (n > MATRIX_MAX_ROWS)
	{
		n = MATRIX_MAX_ROWS;
	}
	for (i = 0u; i < n; i++)
	{
		const MatrixFinalRow_t *r = &rows[i];

		(void)snprintf(s_raw_last[i], CFG_APP_PROTO_RAW_LINE_CAP,
			       "%u,%u,%u,%u,%.3f,%d,%d,%d",
			       (unsigned)r->tray_id, (unsigned)r->col,
			       (unsigned)r->row, (unsigned)r->class_id,
			       (double)r->confidence / 100.0, (int)r->u,
			       (int)r->v, (int)r->z_mm);
	}
	s_raw_last_cnt = n;
	s_raw_last_declared = n;
	s_raw_last_chk_ok = 1u;
	s_raw_last_grid_ok = 1u;
}
