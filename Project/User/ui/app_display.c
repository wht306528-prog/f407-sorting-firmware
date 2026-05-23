/*
 * app_display.c
 *
 * NT35510 文本 UI：主页、原始矩阵子页、最终矩阵子页、串口诊断子页。
 *
 * 底栏：高度 APP_BOTTOM_H，像素坐标 y 从屏顶向下增大；触摸判据为
 * y >= APP_BOTTOM_Y - APP_BOTTOM_HIT_TOP_EXTEND（向下可再扩 APP_BOTTOM_UP_SLOP
 * 仅用于手指抬起瞬间，见 touch_normalize_bottom_bar_release_xy）。分段与 ui_paint_bottom_buttons
 * 等宽一致，保证「绘制区」与「触摸段」对齐。
 *
 * HEX_SCROLL_RING_U2 / HEX_SCROLL_RING_U3：电机 USART2 与矩阵 USART3 的 HEX 环形深度。
 * 主页：顶 7 行为状态；其下整带为 Raw 矩阵（独立 s_main_raw_scroll，抬起在半屏左/右缘翻页）；底栏 3 键 SER|MAIN|FIN；详细 HEX 见 SER 子页。
 * 界面标签为 ASCII（Font8x16 字库按英文点阵）；勿嵌入 UTF-8 多字节以免花屏。
 *
 * AppMatrixModbusDiag_t.read_status 屏显含义见 app_matrix_modbus.h（0 待机 1 读中 2 成功 4 失败）。
 */

#include "app_display.h"

#include "app_matrix.h"
#include "app_matrix_modbus.h"
#include "app_protocol.h"
#include "app_sort.h"
#include "bsp_nt35510_lcd.h"
#include "bsp_uart.h"
#include "bsp_uart3.h"
#include "bsp_led.h"
#include "delay.h"
#include "fonts.h"
#include "global_config.h"
#include "modbus_master.h"

#include <stdio.h>
#include <string.h>

#define APP_UI_LINE_CAP     120u
/** 主页顶区状态行数（行号 0..MAIN_SUMMARY_ROWS-1），自 MAIN_SUMMARY_ROWS 起为 Raw 缓充区 */
#define MAIN_SUMMARY_ROWS   7u
#define MAIN_RAW_CAP        45u
#define APP_LINES           (MAIN_SUMMARY_ROWS + MAIN_RAW_CAP)
/** RS485 HEX 缓冲字符容量（单帧尽量单行显示） */
#define APP_HEX_LINE_CHARS  160u
#define HEX_SCROLL_RING_U2  8u
#define HEX_SCROLL_RING_U3  26u
/** 主页 HEX 可视行数（压缩一行留给分界线 + Final 矩阵多行滚动） */
#define HEX_VISIBLE_ROWS    3u

/** SER 诊断：头 6 + U2 全环 + 分隔 + U3 全环，另预留长行折行槽位 */
#define SERIAL_DIAG_MAX_LINES 96u

/** 主页主文字区单行高度（Font8x16） */
#define APP_MAIN_ROW_PX     16u

/** 底栏像素：触摸与绘制共用（加大以利点按） */
#define APP_BOTTOM_H  60u
#define APP_BOTTOM_Y  ((uint16_t)((uint32_t)LCD_Y_LENGTH - (uint32_t)APP_BOTTOM_H))

/**
 * 子页明细最大行数（按竖向长边 800px 预留缓冲；实际绘制用 matrix_detail_vp_lines()）
 */
#define MATRIX_DETAIL_LINES_MAX                                               \
	((uint16_t)(((uint32_t)NT35510_MORE_PIXEL - (uint32_t)APP_BOTTOM_H) /   \
		    (uint32_t)APP_MAIN_ROW_PX))

/** Raw 明细页：表头行数 */
#define RAW_DETAIL_HEADER_LINES 1u
/** Final 明细页：表头 + 每条记录两行（字段多，分两行显示） */
#define FINAL_DETAIL_HEADER_LINES 2u
#define FINAL_LINES_PER_RECORD    2u

/** 按下：允许的底栏上方扩展命中（像素） */
#define APP_BOTTOM_HIT_TOP_EXTEND 180u
/** 抬起：在按下基础上再放宽，减少手指滑出条带导致松手无效 */
#define APP_BOTTOM_UP_SLOP        96u

/** NT35510_DisplayStringEx：字模 0→背景色、非 0→前景色；0 为正常显示 */
#define DRAW_EX_MODEL 0u
/** 主页 Final 摘要滚动窗口行数（行 10 起） */
#define FINAL_SCROLL_ROWS 8u
/** Final 摘要自动滚动周期 ms */
#define FINAL_AUTOSCROLL_MS 1000u


static char           s_scr[APP_LINES][APP_UI_LINE_CAP];
static char           s_scr_prev[APP_LINES][APP_UI_LINE_CAP];
/** 行 6~8 HEX 专用缓冲（与 s_scr 并行） */
static char           s_hex_lines[HEX_VISIBLE_ROWS][APP_HEX_LINE_CHARS];
static char           s_hex_prev[HEX_VISIBLE_ROWS][APP_HEX_LINE_CHARS];

static char           s_detail[MATRIX_DETAIL_LINES_MAX][APP_UI_LINE_CAP];

static AppIndicatorState_t s_ind;
static uint32_t         s_last_blink_tick;
static uint8_t          s_blink_phase;
static char             s_fault[APP_UI_LINE_CAP];
static char             s_runflg[APP_UI_LINE_CAP];
static char             s_fsm_line[APP_UI_LINE_CAP];
static char             s_serv_main[24];
static char             s_serv_step[24];

static uint32_t         s_last_ui_tick;
static AppDisplayPage_te s_page;
/** 与 `s_page` 不等则整屏清一次，避免残影 */
static int32_t         s_last_painted_page = -1;

static uint16_t         s_detail_scroll;
static uint16_t         s_main_raw_scroll;
static uint16_t         s_final_autoscroll_off;
static uint32_t         s_final_autoscroll_tick;
static int8_t  s_touch_bar_seg   = -1;
static uint8_t s_touch_bar_armed = 0u;
/** 按下瞬间的页面（Down 可能已切页，Up 仍需按按下时页面分支） */
static AppDisplayPage_te s_touch_pg_at_down = APP_DISPLAY_PAGE_MAIN;
/** GT911 8 路归一化候选中选中的序号，-1 无效 */
static int8_t          s_touch_norm_idx = -1;
/** 上一次归一化后的 X，用于在多数候选中择优（-1 表示首次任选首候选） */
static int32_t         s_touch_pick_prev_nx = -1;

static char           s_serial_diag_lines[SERIAL_DIAG_MAX_LINES][APP_UI_LINE_CAP];
static uint16_t       s_serial_diag_nlines;

static const char *const s_main_btn_lbls[3] = {
	"SER", "MAIN", "FIN",
};
static const char *const s_mat_btn_lbls[3] = {
	"Up", "Back", "Dn",
};


/** 电机口 HEX 环：索引 0 最旧 */
static char             s_hex_ring[HEX_SCROLL_RING_U2][APP_HEX_LINE_CHARS];
/** 矩阵口 HEX 环：更长，容纳一帧多行续传 */
static char             s_hex_ring_u3[HEX_SCROLL_RING_U3][APP_HEX_LINE_CHARS];

/** trace 拼行跨刷新保留，避免每圈把半成品压成碎行 */
static char             s_hex_tr_acc_u2[APP_HEX_LINE_CHARS];
static unsigned         s_hex_tr_pos_u2;
static uint8_t          s_hex_tr_ok_u2;
static uint8_t          s_hex_tr_tx_u2;

static char             s_hex_tr_acc_u3[APP_HEX_LINE_CHARS];
static unsigned         s_hex_tr_pos_u3;
static uint8_t          s_hex_tr_ok_u3;
static uint8_t          s_hex_tr_tx_u3;

/*
 * 功能：安全 strncpy 并保证尾零。
 * 交互：本文件各文本缓存写入。
 */
static void copy_trim(char *dst, unsigned cap, const char *src)
{
	if ((dst == NULL) || (cap == 0u))
	{
		return;
	}
	if (src == NULL)
	{
		dst[0] = '\0';
		return;
	}
	(void)strncpy(dst, src, cap - 1u);
	dst[cap - 1u] = '\0';
}

#define TOUCH_NORM_CAND_COUNT 8u

/*
 * 功能：子页明细可视行数（不覆盖底栏）；横竖屏随 APP_BOTTOM_Y 变化。
 */
static uint16_t matrix_detail_vp_lines(void)
{
	uint32_t yrem = (uint32_t)APP_BOTTOM_Y;
	uint16_t n = (uint16_t)(yrem / (uint32_t)APP_MAIN_ROW_PX);

	if (n > MATRIX_DETAIL_LINES_MAX)
	{
		n = MATRIX_DETAIL_LINES_MAX;
	}
	if (n == 0u)
	{
		n = 1u;
	}
	return n;
}

/*
 * 功能：将 GT911 原始 (x,y) 按候选 idx 映射到逻辑屏坐标（与原 8 路顺序一致）。
 */
static void touch_norm_apply(uint8_t idx, int32_t x, int32_t y, int32_t xm,
			     int32_t ym, int32_t *nx, int32_t *ny)
{
	switch (idx)
	{
	case 1u:
		*nx = xm - x;
		*ny = y;
		break;
	case 2u:
		*nx = x;
		*ny = ym - y;
		break;
	case 3u:
		*nx = xm - x;
		*ny = ym - y;
		break;
	case 4u:
		*nx = y;
		*ny = x;
		break;
	case 5u:
		*nx = ym - y;
		*ny = x;
		break;
	case 6u:
		*nx = y;
		*ny = xm - x;
		break;
	case 7u:
		*nx = ym - y;
		*ny = xm - x;
		break;
	case 0u:
	default:
		*nx = x;
		*ny = y;
		break;
	}
}

/*
 * 功能：按下 —— 在合法候选中选 nx，并记录 s_touch_norm_idx。
 */
static uint8_t touch_normalize_bottom_bar_press(int32_t x, int32_t y,
						int32_t *nx_out)
{
	const int32_t ybot = (int32_t)APP_BOTTOM_Y;
	const int32_t ext = (int32_t)APP_BOTTOM_HIT_TOP_EXTEND;
	int32_t       xm;
	int32_t       ym;
	uint8_t       u;
	uint8_t       any = 0u;
	uint8_t       best_idx = 0u;
	int32_t       best_nx = 0;
	int32_t       best_dist = 0x7fffffff;

	if ((LCD_X_LENGTH == 0u) || (LCD_Y_LENGTH == 0u))
	{
		s_touch_norm_idx = -1;
		return 0u;
	}
	xm = (int32_t)LCD_X_LENGTH - 1;
	ym = (int32_t)LCD_Y_LENGTH - 1;

	for (u = 0u; u < TOUCH_NORM_CAND_COUNT; u++)
	{
		int32_t cn;
		int32_t cy;
		int32_t d;

		touch_norm_apply(u, x, y, xm, ym, &cn, &cy);
		if (cy < (ybot - ext))
		{
			continue;
		}
		if ((cn < 0) || (cn >= (int32_t)LCD_X_LENGTH))
		{
			continue;
		}
		if (s_touch_pick_prev_nx < 0)
		{
			best_nx = cn;
			best_idx = u;
			any = 1u;
			break;
		}
		d = cn - s_touch_pick_prev_nx;
		if (d < 0)
		{
			d = -d;
		}
		if ((any == 0u) || (d < best_dist))
		{
			best_dist = d;
			best_nx = cn;
			best_idx = u;
			any = 1u;
		}
	}
	if (any == 0u)
	{
		s_touch_norm_idx = -1;
		return 0u;
	}
	s_touch_norm_idx = (int8_t)best_idx;
	s_touch_pick_prev_nx = best_nx;
	*nx_out = best_nx;
	return 1u;
}

/*
 * 功能：抬起 —— 复用按下选定的归一化序号，计算 nx。
 */
static uint8_t touch_normalize_bottom_bar_release_xy(int32_t x, int32_t y,
						     int32_t *nx_out)
{
	const int32_t ybot = (int32_t)APP_BOTTOM_Y;
	const int32_t ext = (int32_t)APP_BOTTOM_HIT_TOP_EXTEND +
			    (int32_t)APP_BOTTOM_UP_SLOP;
	int32_t xm;
	int32_t ym;
	int32_t cn;
	int32_t cy;

	if ((LCD_X_LENGTH == 0u) || (LCD_Y_LENGTH == 0u) ||
	    (s_touch_norm_idx < 0) ||
	    (s_touch_norm_idx >= (int8_t)TOUCH_NORM_CAND_COUNT))
	{
		return 0u;
	}
	xm = (int32_t)LCD_X_LENGTH - 1;
	ym = (int32_t)LCD_Y_LENGTH - 1;
	touch_norm_apply((uint8_t)s_touch_norm_idx, x, y, xm, ym, &cn, &cy);
	if (cy < (ybot - ext))
	{
		return 0u;
	}
	if ((cn < 0) || (cn >= (int32_t)LCD_X_LENGTH))
	{
		return 0u;
	}
	*nx_out = cn;
	return 1u;
}

/*
 * 功能：主页 Raw 显示带内坐标（不修改 s_touch_norm_idx）；用于半屏翻页 Raw。
 */
static uint8_t touch_normalize_main_raw_zone(int32_t x, int32_t y, int32_t *nx_out)
{
	const int32_t ybot = (int32_t)APP_BOTTOM_Y;
	const int32_t raw_top = (int32_t)MAIN_SUMMARY_ROWS *
				    (int32_t)APP_MAIN_ROW_PX -
				    (int32_t)APP_MAIN_ROW_PX;
	int32_t       xm;
	int32_t       ym;
	uint8_t       u;

	if ((LCD_X_LENGTH == 0u) || (LCD_Y_LENGTH == 0u) || (nx_out == NULL))
	{
		return 0u;
	}
	xm = (int32_t)LCD_X_LENGTH - 1;
	ym = (int32_t)LCD_Y_LENGTH - 1;

	for (u = 0u; u < TOUCH_NORM_CAND_COUNT; u++)
	{
		int32_t cn;
		int32_t cy;

		touch_norm_apply(u, x, y, xm, ym, &cn, &cy);
		if ((cy < raw_top) || (cy >= ybot))
		{
			continue;
		}
		if ((cn < 0) || (cn >= (int32_t)LCD_X_LENGTH))
		{
			continue;
		}
		*nx_out = cn;
		return 1u;
	}
	return 0u;
}

/*
 * 功能：底栏 X 等分分段，与 `ui_paint_bottom_buttons` 绘制一致；钳位 nx、尾格吃余数。
 */
static int8_t touch_bottom_seg_from_x(int32_t nx, uint8_t nseg)
{
	uint32_t w;
	uint32_t segw;
	uint32_t ux;

	if ((nseg == 0u) || (LCD_X_LENGTH == 0u)) {
		return -1;
	}
	w = (uint32_t)LCD_X_LENGTH;
	if (nx < 0) {
		nx = 0;
	}
	ux = (uint32_t)nx;
	if (ux >= w) {
		ux = w - 1u;
	}
	if ((nseg > 1u) && (ux + 3u >= w)) {
		return (int8_t)((int32_t)nseg - 1);
	}
	segw = w / (uint32_t)nseg;
	if (segw == 0u) {
		return 0;
	}
	if (ux >= ((uint32_t)nseg - 1u) * segw) {
		return (int8_t)((int32_t)nseg - 1);
	}
	return (int8_t)(ux / segw);
}

static uint16_t raw_detail_visible_data_lines(void)
{
	uint16_t vp = matrix_detail_vp_lines();

	return (uint16_t)(vp - RAW_DETAIL_HEADER_LINES);
}

static uint16_t final_detail_visible_records(void)
{
	uint16_t vp = matrix_detail_vp_lines();

	return (uint16_t)((vp - FINAL_DETAIL_HEADER_LINES) /
			  FINAL_LINES_PER_RECORD);
}

static uint16_t detail_scroll_max_raw(uint16_t total_lines)
{
	uint16_t vis = raw_detail_visible_data_lines();

	if (total_lines <= vis) {
		return 0u;
	}
	return (uint16_t)(total_lines - vis);
}

static uint16_t detail_scroll_max_final(uint16_t total_records)
{
	uint16_t vis = final_detail_visible_records();

	if (total_records <= vis) {
		return 0u;
	}
	return (uint16_t)(total_records - vis);
}

/*
 * 功能：将冻结 Raw CSV 行格式化为 8 列可读文本（与 app_protocol 生成顺序一致）。
 */
static void format_raw_matrix_row(char *dst, unsigned cap, uint16_t idx,
				  const char *csv)
{
	unsigned tu, co, ro, cl;
	float    cf;
	int      uu, vv, zz;

	if ((dst == NULL) || (cap == 0u)) {
		return;
	}
	if ((csv == NULL) || (csv[0] == '\0')) {
		dst[0] = '\0';
		return;
	}
	if (sscanf(csv, "%u,%u,%u,%u,%f,%d,%d,%d", &tu, &co, &ro, &cl, &cf, &uu,
		   &vv, &zz) == 8) {
		(void)snprintf(dst, cap,
			       "#%u T%u C%u R%u Cl%u Cf%.2f U%d V%d Z%d",
			       (unsigned)idx, tu, co, ro, cl, cf, uu, vv, zz);
		dst[cap - 1u] = '\0';
	} else {
		copy_trim(dst, cap, csv);
	}
}

/*
 * 功能：主页 Raw 带可视行数。
 */
static uint16_t main_raw_visible_lines(void)
{
	uint32_t top = (uint32_t)MAIN_SUMMARY_ROWS * (uint32_t)APP_MAIN_ROW_PX;
	uint32_t yb = (uint32_t)APP_BOTTOM_Y;
	uint32_t h;
	uint16_t n;

	if (top >= yb)
	{
		return 0u;
	}
	h = yb - top - (uint32_t)APP_BOTTOM_H;
	n = (uint16_t)(h / (uint32_t)APP_MAIN_ROW_PX);
	if (n > MAIN_RAW_CAP)
	{
		n = MAIN_RAW_CAP;
	}
	return n;
}

/*
 * 功能：将冻结 Raw 行写入主页 s_scr[MAIN_SUMMARY_ROWS..]。
 */
static void compose_main_raw_into_scr(void)
{
	uint16_t vis = main_raw_visible_lines();
	uint16_t total = AppProtocol_GetRawLastLineCount();
	uint16_t smax;
	unsigned i;

	if (vis > MAIN_RAW_CAP)
	{
		vis = MAIN_RAW_CAP;
	}
	smax = (total > vis) ? (uint16_t)(total - vis) : 0u;
	if (s_main_raw_scroll > smax)
	{
		s_main_raw_scroll = smax;
	}

	for (i = 0u; i < MAIN_RAW_CAP; i++)
	{
		uint16_t row = (uint16_t)(MAIN_SUMMARY_ROWS + i);

		if (row >= APP_LINES)
		{
			break;
		}
		if (i >= vis)
		{
			s_scr[row][0] = '\0';
			continue;
		}
		{
			uint16_t idx = (uint16_t)(s_main_raw_scroll + i);

			if (idx < total)
			{
				char tmp[APP_UI_LINE_CAP];

				if (AppProtocol_GetRawLastLine(
					idx, tmp,
					(uint16_t)sizeof(tmp)) == 0u)
				{
					copy_trim(s_scr[row], sizeof(s_scr[0]),
						  "---");
				}
				else
				{
					format_raw_matrix_row(
					    s_scr[row],
					    (unsigned)sizeof(s_scr[0]), idx,
					    tmp);
				}
			}
			else
			{
				s_scr[row][0] = '\0';
			}
		}
	}
}

/*
 * 功能：底栏按下期间用于高亮某一格（与 GT911 Down/Up 配对）。
 */
static int8_t ui_pressed_idx(void)
{
	if (s_touch_bar_armed == 0u) {
		return -1;
	}
	return s_touch_bar_seg;
}

/*
 * 功能：绘制底部等分按钮条（高对比：浅底 + 黑字；按下黄底；标签 Font16x32）。
 */
static void ui_paint_bottom_buttons(uint8_t nseg, const char *const *labels,
				    int8_t pressed_idx)
{
	uint16_t segw = (uint16_t)LCD_X_LENGTH;
	unsigned u;

	if (nseg == 0u) {
		return;
	}
	segw = (uint16_t)(LCD_X_LENGTH / nseg);

	for (u = 0u; u < (unsigned)nseg; u++) {
		uint16_t     x0 = (uint16_t)(u * segw);
		uint16_t     y = APP_BOTTOM_Y;
		int          is_pr = (pressed_idx == (int8_t)u);
		const char * lb;
		size_t       sl;
		uint16_t     tx;
		uint16_t     fy;
		char         buf[24];

		LCD_SetTextColor(is_pr ? YELLOW : WHITE);
		NT35510_DrawRectangle(x0, y, segw, APP_BOTTOM_H, 1u);

		LCD_SetTextColor(GREY);
		NT35510_DrawRectangle(x0, y, segw, APP_BOTTOM_H, 0u);

		lb = labels[u];
		if (lb == NULL) {
			lb = "";
		}

		LCD_SetFont(&Font16x32);
		sl = strlen(lb);
		if ((uint32_t)(sl * Font16x32.Width) > (uint32_t)segw) {
			sl = (size_t)(segw / Font16x32.Width);
		}
		if (sl >= sizeof(buf)) {
			sl = sizeof(buf) - 1u;
		}
		(void)strncpy(buf, lb, sl);
		buf[sl] = '\0';
		tx = (uint16_t)(x0 + (segw - (uint16_t)(sl * Font16x32.Width)) / 2u);
		fy = (uint16_t)(y + (APP_BOTTOM_H - Font16x32.Height) / 2u);
		LCD_SetColors(BLACK, is_pr ? YELLOW : WHITE);
		NT35510_DispString_EN(tx, fy, buf);
	}
}

/*
 * 功能：主页文本行号 → 起始 Y 像素（800×480，主区统一 16px 行高）。
 * 交互：paint_main_page 绘制。
 */
static uint16_t app_main_row_y(uint16_t row)
{
	return (uint16_t)((uint32_t)row * (uint32_t)APP_MAIN_ROW_PX);
}

/*
 * 功能：主页行高（统一 Font8x16）。
 * 交互：与 app_main_row_y 配套。
 */
static uint16_t app_main_row_height(uint16_t row)
{
	(void)row;
	return APP_MAIN_ROW_PX;
}

/*
 * 功能：识别过短 TX:/RX: 行（示波噪声）以便替换成分界线。
 * 交互：hex_sanitize_copy。
 */
static uint8_t hex_line_should_blank(const char *s)
{
	size_t n;

	if ((s == NULL) || (s[0] == '\0'))
	{
		return 0u;
	}
	if (strncmp(s, "TX:", 3u) == 0)
	{
		return 0u;
	}
	if (strncmp(s, "RX:", 3u) != 0)
	{
		return 0u;
	}
	n = strlen(s);
	if (n >= 28u)
	{
		return 0u;
	}
	return 1u;
}

/*
 * 功能：拷贝 HEX 行前清理噪声短行。
 * 交互：compose_hex_visible_block。
 */
static void hex_sanitize_copy(char *dst, unsigned cap, const char *src)
{
	copy_trim(dst, cap, src);
	if (hex_line_should_blank(dst) != 0u)
	{
		copy_trim(dst, cap, "---------");
	}
}

/*
 * 功能：SER/主页共用的 Font8x16 每行最大字符数（屏宽）。
 */
static uint16_t ser_disp_cols(void)
{
	uint16_t w = Font8x16.Width;

	if (w == 0u)
	{
		w = 8u;
	}
	return (uint16_t)((uint32_t)LCD_X_LENGTH / (uint32_t)w);
}

/*
 * 功能：主页 HEX 区固定 8×16 折行绘制；y 递增到 y_max 前停止（y_max 为行 9 顶，不含）。
 * 交互：paint_main_page。
 */
static uint16_t paint_hex_wrapped_lines(uint16_t x, uint16_t y, uint16_t y_max,
					const char *str)
{
	const uint16_t lh = Font8x16.Height;
	uint16_t       cols = ser_disp_cols();
	const char    *p = str;

	if (cols < 8u)
	{
		cols = 8u;
	}
	if ((p == NULL) || (p[0] == '\0'))
	{
		return y;
	}
	LCD_SetColors(WHITE, BLACK);
	LCD_SetFont(&Font8x16);
	for (;;)
	{
		size_t L;
		size_t take;
		size_t sp;
		size_t cut;
		char   linebuf[APP_UI_LINE_CAP];

		if ((uint32_t)y + (uint32_t)lh > (uint32_t)y_max)
		{
			break;
		}
		L = strlen(p);
		if (L == 0u)
		{
			break;
		}
		take = L;
		if (take > (size_t)cols)
		{
			cut = (size_t)cols;
			sp = cut;
			while ((sp > 0u) && (p[sp - 1u] != ' '))
			{
				sp--;
			}
			if (sp > cut / 4u)
			{
				cut = sp;
			}
			take = cut;
		}
		if (take >= sizeof(linebuf))
		{
			take = sizeof(linebuf) - 1u;
		}
		(void)memcpy(linebuf, p, take);
		linebuf[take] = '\0';
		NT35510_DispString_EN(x, y, linebuf);
		y = (uint16_t)((uint32_t)y + (uint32_t)lh);
		p += take;
		while (p[0] == ' ')
		{
			p++;
		}
	}
	return y;
}

static void serial_diag_append(unsigned *pk, const char *text)
{
	if ((pk == NULL) || ((*pk) >= SERIAL_DIAG_MAX_LINES) || (text == NULL))
	{
		return;
	}
	copy_trim(s_serial_diag_lines[*pk], APP_UI_LINE_CAP, text);
	(*pk)++;
}

/*
 * 功能：prefix + payload 按屏宽折成多条写入 s_serial_diag_lines（可选 SER 长 HEX）。
 */
static void serial_diag_append_wrapped(unsigned *pk, const char *prefix,
				       const char *payload)
{
	uint16_t     cols = ser_disp_cols();
	const char  *pref = (prefix != NULL) ? prefix : "";
	const char  *p = (payload != NULL) ? payload : "";
	size_t       pl = strlen(pref);
	uint8_t      first = 1u;

	if (cols < 8u)
	{
		cols = 8u;
	}
	while ((p[0] != '\0') && ((*pk) < SERIAL_DIAG_MAX_LINES))
	{
		char   line[APP_UI_LINE_CAP];
		size_t room;
		size_t take;
		size_t sp;
		size_t cut;

		if (first != 0u)
		{
			room = (pl < (size_t)cols) ? ((size_t)cols - pl) : 1u;
		}
		else
		{
			room = (size_t)cols;
		}
		if (room == 0u)
		{
			room = 1u;
		}
		take = strlen(p);
		if (take > room)
		{
			cut = room;
			sp = cut;
			while ((sp > 0u) && (p[sp - 1u] != ' '))
			{
				sp--;
			}
			if (sp > cut / 4u)
			{
				cut = sp;
			}
			take = cut;
		}
		if (first != 0u)
		{
			(void)snprintf(line, sizeof(line), "%s%.*s", pref,
				       (int)take, p);
		}
		else
		{
			(void)snprintf(line, sizeof(line), "%.*s", (int)take, p);
		}
		line[sizeof(line) - 1u] = '\0';
		copy_trim(s_serial_diag_lines[*pk], APP_UI_LINE_CAP, line);
		(*pk)++;
		first = 0u;
		p += take;
		while (p[0] == ' ')
		{
			p++;
		}
	}
}

/*
 * 功能：环形缓冲上移一行并把新 HEX 串落在最新槽。
 * 交互：hex_scroll_feed_from_trace。
 */
static void hex_scroll_push_line(const char *line)
{
	unsigned row;

	for (row = 1u; row < HEX_SCROLL_RING_U2; row++)
	{
		copy_trim(s_hex_ring[row - 1u], sizeof(s_hex_ring[0]), s_hex_ring[row]);
	}
	copy_trim(s_hex_ring[HEX_SCROLL_RING_U2 - 1u], sizeof(s_hex_ring[0]), line);
}

static void hex_scroll_push_line_u3(const char *line)
{
	unsigned row;

	for (row = 1u; row < HEX_SCROLL_RING_U3; row++)
	{
		copy_trim(s_hex_ring_u3[row - 1u], sizeof(s_hex_ring_u3[0]),
			  s_hex_ring_u3[row]);
	}
	copy_trim(s_hex_ring_u3[HEX_SCROLL_RING_U3 - 1u], sizeof(s_hex_ring_u3[0]),
		  line);
}

/** 与 `bsp_uart.c` trace：`0`=DATA，`1`=FRAME_END */
#define APP_BS_TR_DATA 0u
#define APP_BS_TR_EOF  1u

/** 当前累加行是否有可输出内容（裸 RX:/TX: 或空串不算） */
static uint8_t hex_trace_line_nonempty(const char *acc)
{
	unsigned L = (unsigned)strlen(acc);

	if (L == 0u)
	{
		return 0u;
	}
	if ((L == 3u) && (strncmp(acc, "RX:", 3u) == 0))
	{
		return 0u;
	}
	if ((L == 3u) && (strncmp(acc, "TX:", 3u) == 0))
	{
		return 0u;
	}
	if ((L >= 3u) && (acc[0] == ' '))
	{
		return 1u;
	}
	return (L > 3u) ? 1u : 0u;
}

/*
 * 功能：排空 BSP_RS485 跟踪队列，拼装 TX:/RX: + HEX，遇帧结束分行压环。
 * 单行溢出时续行为「空格 + HEX」延续同一方向数据，不再重复 TX:/RX:。
 * 拼行状态为 static，跨 Refresh 保持；仅 EOF / 方向切换 / 行长溢出时压环。
 * 交互：compose_hex_visible_block；每圈最多 CFG_HEX_TRACE_DRAIN_MAX evt。
 */
static void hex_scroll_feed_from_trace(void)
{
	unsigned ops;

	for (ops = 0u; ops < CFG_HEX_TRACE_DRAIN_MAX; ops++)
	{
		uint8_t kind;
		uint8_t itx;
		uint8_t bb;

		if (BSP_RS485_TracePopEvt(&kind, &itx, &bb) == 0u)
		{
			break;
		}

		if (kind == APP_BS_TR_EOF)
		{
			if ((s_hex_tr_ok_u2 != 0u) &&
			    hex_trace_line_nonempty(s_hex_tr_acc_u2))
			{
				hex_scroll_push_line(s_hex_tr_acc_u2);
			}
			s_hex_tr_ok_u2 = 0u;
			continue;
		}

		if (s_hex_tr_ok_u2 == 0u)
		{
			s_hex_tr_ok_u2 = 1u;
			s_hex_tr_tx_u2 = itx;
			s_hex_tr_pos_u2 = (unsigned)snprintf(
			    s_hex_tr_acc_u2, sizeof(s_hex_tr_acc_u2), "%s",
			    itx ? "TX:" : "RX:");
		}
		else if (itx != s_hex_tr_tx_u2)
		{
			if (hex_trace_line_nonempty(s_hex_tr_acc_u2))
			{
				hex_scroll_push_line(s_hex_tr_acc_u2);
			}
			s_hex_tr_tx_u2 = itx;
			s_hex_tr_pos_u2 = (unsigned)snprintf(
			    s_hex_tr_acc_u2, sizeof(s_hex_tr_acc_u2), "%s",
			    itx ? "TX:" : "RX:");
		}

		if (s_hex_tr_pos_u2 + 4u < sizeof(s_hex_tr_acc_u2))
		{
			s_hex_tr_pos_u2 += (unsigned)snprintf(
			    s_hex_tr_acc_u2 + s_hex_tr_pos_u2,
			    sizeof(s_hex_tr_acc_u2) - s_hex_tr_pos_u2, " %02X",
			    (unsigned int)bb);
		}
		else
		{
			hex_scroll_push_line(s_hex_tr_acc_u2);
			s_hex_tr_pos_u2 = (unsigned)snprintf(
			    s_hex_tr_acc_u2, sizeof(s_hex_tr_acc_u2), " %02X",
			    (unsigned int)bb);
		}
	}
}

/*
 * 功能：USART3 trace → HEX 环形（逻辑同 hex_scroll_feed_from_trace）。
 * 矩阵长应答拆行时续行可无 RX:，与 RS485 侧行为一致。
 */
static void hex_scroll_feed_from_trace_u3(void)
{
	unsigned ops;

	for (ops = 0u; ops < CFG_HEX_TRACE_DRAIN_MAX; ops++)
	{
		uint8_t kind;
		uint8_t itx;
		uint8_t bb;

		if (BSP_USART3_TracePopEvt(&kind, &itx, &bb) == 0u)
		{
			break;
		}

		if (kind == APP_BS_TR_EOF)
		{
			if ((s_hex_tr_ok_u3 != 0u) &&
			    hex_trace_line_nonempty(s_hex_tr_acc_u3))
			{
				hex_scroll_push_line_u3(s_hex_tr_acc_u3);
			}
			s_hex_tr_ok_u3 = 0u;
			continue;
		}

		if (s_hex_tr_ok_u3 == 0u)
		{
			s_hex_tr_ok_u3 = 1u;
			s_hex_tr_tx_u3 = itx;
			s_hex_tr_pos_u3 = (unsigned)snprintf(
			    s_hex_tr_acc_u3, sizeof(s_hex_tr_acc_u3), "%s",
			    itx ? "TX:" : "RX:");
		}
		else if (itx != s_hex_tr_tx_u3)
		{
			if (hex_trace_line_nonempty(s_hex_tr_acc_u3))
			{
				hex_scroll_push_line_u3(s_hex_tr_acc_u3);
			}
			s_hex_tr_tx_u3 = itx;
			s_hex_tr_pos_u3 = (unsigned)snprintf(
			    s_hex_tr_acc_u3, sizeof(s_hex_tr_acc_u3), "%s",
			    itx ? "TX:" : "RX:");
		}

		if (s_hex_tr_pos_u3 + 4u < sizeof(s_hex_tr_acc_u3))
		{
			s_hex_tr_pos_u3 += (unsigned)snprintf(
			    s_hex_tr_acc_u3 + s_hex_tr_pos_u3,
			    sizeof(s_hex_tr_acc_u3) - s_hex_tr_pos_u3, " %02X",
			    (unsigned int)bb);
		}
		else
		{
			hex_scroll_push_line_u3(s_hex_tr_acc_u3);
			s_hex_tr_pos_u3 = (unsigned)snprintf(
			    s_hex_tr_acc_u3, sizeof(s_hex_tr_acc_u3), " %02X",
			    (unsigned int)bb);
		}
	}
}

/*
 * 功能：刷新 trace 并从环取最近 HEX_VISIBLE_ROWS 行到 s_hex_lines。
 * 交互：compose_screen_main。
 */
static void compose_hex_visible_block(void)
{
	char   line[APP_HEX_LINE_CHARS];

	hex_scroll_feed_from_trace();
	hex_scroll_feed_from_trace_u3();

	(void)snprintf(line, sizeof(line), "U2:%s",
		       s_hex_ring[(unsigned)(HEX_SCROLL_RING_U2 - 1u)]);
	hex_sanitize_copy(s_hex_lines[0], sizeof(s_hex_lines[0]), line);

	(void)snprintf(line, sizeof(line), "U3:%s",
		       s_hex_ring_u3[(unsigned)(HEX_SCROLL_RING_U3 - 2u)]);
	hex_sanitize_copy(s_hex_lines[1], sizeof(s_hex_lines[0]), line);

	(void)snprintf(line, sizeof(line), "U3:%s",
		       s_hex_ring_u3[(unsigned)(HEX_SCROLL_RING_U3 - 1u)]);
	hex_sanitize_copy(s_hex_lines[2], sizeof(s_hex_lines[0]), line);
}

void AppDisplay_ResetU3HexScrollAccum(void)
{
	s_hex_tr_acc_u3[0] = '\0';
	s_hex_tr_pos_u3 = 0u;
	s_hex_tr_ok_u3 = 0u;
}

/*
 * 功能：根据 s_ind 刷新板载 RGB 灯（快闪/常亮等业务语义）。
 * 交互：AppDisplay_Refresh 每圈可调。
 */
static void apply_indicator(uint32_t now_ms)
{
	switch (s_ind)
	{
	case APP_IND_IDLE:
		LED_RGB_ALL_OFF;
		break;
	case APP_IND_ARM_RUN:
		LED_R_OFF;
		LED_G_OFF;
		LED_B_ON;
		break;
	case APP_IND_PAUSE_BLINK:
	case APP_IND_FAIL_BLINK:
		if ((now_ms - s_last_blink_tick) > CFG_LED_BLINK_MS)
		{
			s_last_blink_tick = now_ms;
			s_blink_phase ^= 1u;
			if (s_blink_phase)
			{
				LED_B_ON;
				LED_R_OFF;
			}
			else
			{
				LED_B_OFF;
				LED_R_ON;
			}
		}
		break;
	case APP_IND_OK_HOLD:
		LED_R_OFF;
		LED_G_ON;
		LED_B_OFF;
		break;
	case APP_IND_FAIL_HOLD:
	default:
		LED_G_OFF;
		LED_B_OFF;
		LED_R_ON;
		break;
	}
}

/*
 * 功能：设置指示灯状态枚举（分拣/暂停/故障等）。
 * 交互：app_sort 各分支；Flash 节拍初值重置。
 */
void AppIndicator_SetState(AppIndicatorState_t st)
{
	s_ind = st;
	if (st == APP_IND_FAIL_BLINK || st == APP_IND_PAUSE_BLINK)
	{
		s_last_blink_tick = SysTick_GetMs();
	}
}

/*
 * 功能：切换显示页（主屏 / Raw / Final）；DEBUG 等同 MAIN。
 * 交互：触摸屏底栏按钮。
 */
void AppDisplay_SetPage(AppDisplayPage_te pg)
{
	if (pg == APP_DISPLAY_PAGE_DEBUG)
	{
		pg = APP_DISPLAY_PAGE_MAIN;
	}
	s_page = pg;
}

/*
 * 功能：读取当前页面枚举。
 * 交互：触摸释放逻辑判断是否子页。
 */
AppDisplayPage_te AppDisplay_GetPage(void)
{
	return s_page;
}

/*
 * 功能：写入主页「Alarm/Fault」行缓存。
 * 交互：main、app_sort、app_protocol。
 */
void AppDisplay_SetFaultText(const char *ascii)
{
	copy_trim(s_fault, sizeof(s_fault), ascii);
}

/*
 * 功能：写入 RunFlag 摘要行缓存。
 * 交互：main、AppSort_Poll。
 */
void AppDisplay_SetRunFlagText(const char *ascii)
{
	copy_trim(s_runflg, sizeof(s_runflg), ascii);
}

/*
 * 功能：拼装伺服/状态机简报行（含最近一次 Modbus 长度与溢出计数）。
 * 交互：AppSort_Poll 每圈。
 */
void AppDisplay_SetServoBriefText(const char *main_state, const char *step_state)
{
	const ModbusTxnResult_t *lr = ModbusMaster_GetLastResult();

	copy_trim(s_serv_main, sizeof(s_serv_main), main_state);
	copy_trim(s_serv_step, sizeof(s_serv_step), step_state);

	(void)snprintf(s_fsm_line, sizeof(s_fsm_line),
		       "%s %s Tx/Rx %u/%uB %s/%u ovf=%lu",
		       s_serv_main[0] ? s_serv_main : "-",
		       s_serv_step[0] ? s_serv_step : "-",
		       (unsigned)lr->tx_len, (unsigned)lr->rx_len,
		       ModbusMaster_ErrTag(lr->err), (unsigned)lr->err,
		       (unsigned long)BSP_USART1_RxOverflowCount());

	s_fsm_line[sizeof(s_fsm_line) - 1u] = '\0';
}

/*
 * 功能：Modbus BUSY 钩子（当前占位）。
 * 交互：可接 UI 告警。
 */
void AppDisplay_LogModbusBusy(void)
{
	(void)0;
}

/*
 * 功能：预留 RS485 事务摘要打点（占位）。
 * 交互：可调试扩展。
 */
void AppDisplay_LogRs485Summary(uint8_t is_tx, uint8_t slave, uint8_t fc,
				uint16_t byte_len, uint8_t err_or_ok)
{
	(void)is_tx;
	(void)slave;
	(void)fc;
	(void)byte_len;
	(void)err_or_ok;
}

/*
 * 功能：将托盘枚举码转成屏显词（OK/ERR）。
 * 交互：compose_screen_main Tray 行。
 */
/*
 * 功能：简述矩阵几何是否已 Flush / 有效。
 * 交互：分界线摘要行。
 */
static const char *geom_status_word(void)
{
	uint16_t n = AppMatrix_GetValidCount();

	if (n == 0u)
	{
		return "--";
	}
	if (AppMatrix_IsGeomDirty() != 0u)
	{
		return "WAIT";
	}
	if (AppMatrix_AllRowsGeomValid() != 0u)
	{
		return "OK";
	}
	return "BAD";
}

/*
 * 功能：填充主页 Final 矩阵 5 行滚动窗口摘要。
 * 交互：compose_screen_main；读 AppMatrix_GetRow。
 */
static void compose_final_scroll_block(uint32_t tick_ms)
{
	unsigned         i;
	uint16_t         n = AppMatrix_GetValidCount();
	MatrixFinalRow_t row;
	uint16_t         start;

	if (n > FINAL_SCROLL_ROWS)
	{
		uint16_t lim = (uint16_t)(n - (uint16_t)FINAL_SCROLL_ROWS);

		if ((tick_ms - s_final_autoscroll_tick) >= FINAL_AUTOSCROLL_MS)
		{
			s_final_autoscroll_tick = tick_ms;
			if (s_final_autoscroll_off >= lim)
			{
				s_final_autoscroll_off = 0u;
			}
			else
			{
				s_final_autoscroll_off++;
			}
		}
		start = s_final_autoscroll_off;
	}
	else
	{
		s_final_autoscroll_off = 0u;
		start = 0u;
	}

	for (i = 0u; i < FINAL_SCROLL_ROWS; i++)
	{
		char *dst = s_scr[10u + i];

		if (AppMatrix_GetRow((uint16_t)(start + i), &row))
		{
			(void)snprintf(dst, sizeof(s_scr[0]),
				       "r%u t%u c%u|%u|%u P:%ld|%ld",
				       (unsigned)(start + i),
				       (unsigned)row.tray_id,
				       (unsigned)row.col, (unsigned)row.row,
				       (unsigned)row.class_id,
				       (long)row.pulse_motor1_abs,
				       (long)row.pulse_motor2_abs);
			dst[sizeof(s_scr[0]) - 1u] = '\0';
		}
		else
		{
			copy_trim(dst, sizeof(s_scr[0]), "---------");
		}
	}
}

/*
 * 功能：拼装主页逻辑缓冲（矩阵 Modbus/电机/故障/伺服/状态行 + 大面积 Raw 带）。
 * 交互：AppDisplay_Refresh MAIN 分支。
 */
static void compose_screen_main(uint32_t tick_ms)
{
	AppMatrixTrayStats_t      t1s;
	AppMatrixTrayStats_t      t2s;
	AppMatrixTrayStats_t      t3s;
	const ModbusTxnResult_t  *lr = ModbusMaster_GetLastResult();
	char                      servo_suffix[36];
	char                      tray_line[APP_UI_LINE_CAP];
	AppMatrixModbusDiag_t     md;
	const char               *mb_fin;
	const char               *final_st;
	const char               *rdst;
	uint16_t                  valid_count;

	(void)tick_ms;
	AppMatrixModbus_GetDiag(&md);
	valid_count = AppMatrix_GetValidCount();

	switch (md.read_status)
	{
	case APP_MAT_RD_ST_IDLE:
		rdst = "IDLE";
		break;
	case APP_MAT_RD_ST_READING:
		rdst = "READ";
		break;
	case APP_MAT_RD_ST_OK:
		rdst = "OK";
		break;
	case APP_MAT_RD_ST_ERR:
		rdst = "ERR";
		break;
	default:
		rdst = "?";
		break;
	}

	if (AppSort_GetMainState() == APP_SORT_MAIN_ESTOP)
	{
		int32_t ep1 = 0;
		int32_t ep2 = 0;

		AppSort_GetEstopMonitoredPositions(&ep1, &ep2);
		(void)snprintf(servo_suffix, sizeof(servo_suffix), " EPos:%ld|%ld",
			       (long)ep1, (long)ep2);
	}
	else
	{
		servo_suffix[0] = '\0';
	}

	(void)snprintf(s_scr[0], sizeof(s_scr[0]),
		       "MAT U3 %s st%u rows %u/%u pkt %u/%u busy %u mberr %u "
		       "commits %lu rtu_crc_fail %lu",
		       rdst, (unsigned)md.read_status,
		       (unsigned)md.rows_received,
		       (unsigned)MATRIX_EXPECTED_ROWS, (unsigned)md.chunk_done,
		       (unsigned)CFG_MATRIX_PACKET_COUNT, (unsigned)md.busy,
		       (unsigned)md.last_txn_err, (unsigned long)md.commits_ok,
		       (unsigned long)md.crc_fails);

	(void)snprintf(
	    s_scr[1], sizeof(s_scr[1]),
	    "HDR rdy %u upd %u bat %u seq %u ncell %u tray %u U3_RXovf %lu "
	    "SYS:%s",
	    (unsigned)md.hdr_ready, (unsigned)md.hdr_updating,
	    (unsigned)md.hdr_batch, (unsigned)md.hdr_upd_ctr,
	    (unsigned)md.hdr_count, (unsigned)md.hdr_tray_total,
	    (unsigned long)md.rx_ovf_u3, AppSort_GetSysRunFlagText());

	(void)snprintf(s_scr[2], sizeof(s_scr[2]),
		       "ALARM: %s", s_fault);

	(void)snprintf(
	    s_scr[3], sizeof(s_scr[3]),
	    "MTR RS485 U2 %s %s "
	    "Tx/Rx %u/%uB %s/%u ovf=%lu%s",
	    s_serv_main[0] ? s_serv_main : "-",
	    s_serv_step[0] ? s_serv_step : "-", (unsigned)lr->tx_len,
	    (unsigned)lr->rx_len, ModbusMaster_ErrTag(lr->err),
	    (unsigned)lr->err, (unsigned long)BSP_USART1_RxOverflowCount(),
	    servo_suffix);

	(void)snprintf(s_scr[4], sizeof(s_scr[4]),
		       "RunFlag: %s", s_runflg);

	(void)AppMatrix_GetTrayClassStats(1u, MATRIX_TRAY_COLS,
					  MATRIX_TRAY_ROWS, &t1s);
	(void)AppMatrix_GetTrayClassStats(2u, MATRIX_TRAY_COLS,
					  MATRIX_TRAY_ROWS, &t2s);
	(void)AppMatrix_GetTrayClassStats(3u, MATRIX_TRAY_COLS,
					  MATRIX_TRAY_ROWS, &t3s);
	if (valid_count == 0u)
	{
		copy_trim(tray_line, sizeof(tray_line), "Tray --");
	}
	else if (t1s.complete == 0u || t2s.complete == 0u || t3s.complete == 0u)
	{
		(void)snprintf(tray_line, sizeof(tray_line),
			       "Tray ERR T1=%u/%u T2=%u/%u T3=%u/%u",
			       (unsigned)t1s.total_count,
			       (unsigned)(MATRIX_TRAY_COLS * MATRIX_TRAY_ROWS),
			       (unsigned)t2s.total_count,
			       (unsigned)(MATRIX_TRAY_COLS * MATRIX_TRAY_ROWS),
			       (unsigned)t3s.total_count,
			       (unsigned)(MATRIX_TRAY_COLS * MATRIX_TRAY_ROWS));
	}
	else
	{
		(void)snprintf(tray_line, sizeof(tray_line),
			       "Tray E/W/Y T1=%u/%u/%u T2=%u/%u/%u T3=%u/%u/%u",
			       (unsigned)t1s.empty_count,
			       (unsigned)t1s.white_count,
			       (unsigned)t1s.yellow_count,
			       (unsigned)t2s.empty_count,
			       (unsigned)t2s.white_count,
			       (unsigned)t2s.yellow_count,
			       (unsigned)t3s.empty_count,
			       (unsigned)t3s.white_count,
			       (unsigned)t3s.yellow_count);
	}
	copy_trim(s_scr[5], sizeof(s_scr[5]), tray_line);

	mb_fin = (AppMatrixModbus_LastCommitOk() != 0u) ? "OK" : "--";
	if (valid_count == 0u)
	{
		final_st = "--";
	}
	else if ((valid_count == MATRIX_EXPECTED_ROWS) &&
		 (AppMatrixModbus_LastCommitOk() != 0u))
	{
		final_st = "OK";
	}
	else
	{
		final_st = "BAD";
	}

	(void)snprintf(
	    s_scr[6], sizeof(s_scr[6]),
	    "MAT commit %s valid %u/%u raw_lines %lu geom:%s Fin:%s",
	    mb_fin, (unsigned)valid_count,
	    (unsigned)MATRIX_EXPECTED_ROWS,
	    (unsigned long)AppProtocol_GetRawLastLineCount(),
	    geom_status_word(), final_st);

	compose_main_raw_into_scr();
}

/*
 * 功能：载入 Raw 矩阵文本子页缓冲（带滚动偏移）。
 * 交互：RAW_MATRIX 页。
 */
static void compose_detail_raw(void)
{
	uint16_t total = AppProtocol_GetRawLastLineCount();
	uint16_t smax = detail_scroll_max_raw(total);
	unsigned i;

	if (s_detail_scroll > smax) {
		s_detail_scroll = smax;
	}

	copy_trim(s_detail[0], sizeof(s_detail[0]),
		  "idx tray col row cls conf U V Z");

	for (i = 1u; i < matrix_detail_vp_lines(); i++) {
		uint16_t idx = (uint16_t)(s_detail_scroll + (i - 1u));

		if (idx < total) {
			char tmp[APP_UI_LINE_CAP];

			if (AppProtocol_GetRawLastLine(idx, tmp,
						       (uint16_t)sizeof(tmp)) == 0u) {
				copy_trim(s_detail[i], sizeof(s_detail[0]), "---");
			} else {
				format_raw_matrix_row(s_detail[i],
							(unsigned)sizeof(s_detail[0]), idx,
							tmp);
			}
		} else {
			s_detail[i][0] = '\0';
		}
	}
}

/*
 * 功能：载入 Final 矩阵解析后行到子页缓冲（每条记录两行 + 表头两行）。
 * 交互：FINAL_MATRIX 页。
 */
static void compose_detail_final(void)
{
	uint16_t         total = AppMatrix_GetValidCount();
	uint16_t         smax = detail_scroll_max_final(total);
	unsigned         rec;
	uint16_t         vis = final_detail_visible_records();
	uint16_t         base;
	uint16_t         idx;
	MatrixFinalRow_t row;
	double           cf;

	if (s_detail_scroll > smax) {
		s_detail_scroll = smax;
	}

	copy_trim(
	    s_detail[0], sizeof(s_detail[0]),
	    "idx T C R Cl conf U V Z | geom");
	copy_trim(
	    s_detail[1], sizeof(s_detail[1]),
	    "Xc Yc Xw Yw J1 J2 P1 | P2");

	for (rec = 0u; rec < vis; rec++) {
		base = (uint16_t)(FINAL_DETAIL_HEADER_LINES +
				  rec * FINAL_LINES_PER_RECORD);
		if ((uint16_t)(base + 1u) >= matrix_detail_vp_lines()) {
			break;
		}
		idx = (uint16_t)(s_detail_scroll + rec);

		if ((total != 0u) && AppMatrix_GetRow(idx, &row)) {
			cf = (double)row.confidence / 100.0;
			(void)snprintf(s_detail[base], sizeof(s_detail[0]),
				       "#%u T%u C%u R%u Cl%u %.2f %d %d %d %u",
				       (unsigned)idx, (unsigned)row.tray_id,
				       (unsigned)row.col, (unsigned)row.row,
				       (unsigned)row.class_id, cf, (int)row.u,
				       (int)row.v, (int)row.z_mm,
				       (unsigned)row.geom_ok);
			s_detail[base][sizeof(s_detail[0]) - 1u] = '\0';

			(void)snprintf(s_detail[base + 1u], sizeof(s_detail[0]),
				       "Xc%.0f Yc%.0f Xw%.0f Yw%.0f %.2f %.2f %ld|%ld",
				       (double)row.Xc_mm, (double)row.Yc_mm,
				       (double)row.Xw_mm, (double)row.Yw_mm,
				       (double)row.theta1_deg,
				       (double)row.theta2_deg,
				       (long)row.pulse_motor1_abs,
				       (long)row.pulse_motor2_abs);
			s_detail[base + 1u][sizeof(s_detail[0]) - 1u] = '\0';
		} else {
			s_detail[base][0] = '\0';
			s_detail[base + 1u][0] = '\0';
		}
	}
}

/*
 * 功能：串口诊断页：USART2 电机总线 + USART3 矩阵 Modbus 计数与最近 HEX。
 * 逻辑行写入 s_serial_diag_lines，再按 s_detail_scroll 裁入视口（matrix_detail_vp_lines）。
 */
static void compose_serial_diag(void)
{
	AppMatrixModbusDiag_t     md;
	AppMatrixGeomDiag_t       gd;
	const ModbusTxnResult_t  *lr = ModbusMaster_GetLastResult();
	char                      l1[APP_UI_LINE_CAP];
	unsigned                  ir;
	unsigned                  k;
	unsigned                  di;
	uint16_t                  smax;
	uint16_t                  vp;

	AppMatrixModbus_GetDiag(&md);
	AppMatrix_GetGeomDiag(&gd);

	k = 0u;
	serial_diag_append(&k,
			   "Serial: U2 motor (PA2/3) + U3 matrix (RK3588 Modbus)");

	(void)snprintf(
	    l1, sizeof(l1),
	    "U2 %lubps RXovf %lu %s err%u",
	    (unsigned long)CFG_RS485_BAUD,
	    (unsigned long)BSP_USART1_RxOverflowCount(),
	    ModbusMaster_ErrTag(lr->err), (unsigned)lr->err);
	l1[sizeof(l1) - 1u] = '\0';
	serial_diag_append(&k, l1);

	(void)snprintf(
	    l1, sizeof(l1),
	    "U3 %lubps RXovf %lu rd_st %u ph %u pkt %u/%u rows %u/%u busy %u",
	    (unsigned long)CFG_MATRIX_MODBUS_BAUD,
	    (unsigned long)md.rx_ovf_u3, (unsigned)md.read_status,
	    (unsigned)md.phase, (unsigned)md.chunk_done,
	    (unsigned)CFG_MATRIX_PACKET_COUNT, (unsigned)md.rows_received,
	    (unsigned)MATRIX_EXPECTED_ROWS, (unsigned)md.busy);
	l1[sizeof(l1) - 1u] = '\0';
	serial_diag_append(&k, l1);

	(void)snprintf(
	    l1, sizeof(l1),
	    "MAT CRC32 calc %08lX hdr %08lX last_modbus_err %u",
	    (unsigned long)md.crc32_calc, (unsigned long)md.crc32_expect,
	    (unsigned)md.last_txn_err);
	l1[sizeof(l1) - 1u] = '\0';
	serial_diag_append(&k, l1);

	(void)snprintf(
	    l1, sizeof(l1),
	    "REG0 rdy %u upd %u bat %u seq %u ncell %u tray %u "
	    "CRC hi %04X lo %04X commit_ok %u",
	    (unsigned)md.hdr_ready, (unsigned)md.hdr_updating,
	    (unsigned)md.hdr_batch, (unsigned)md.hdr_upd_ctr,
	    (unsigned)md.hdr_count, (unsigned)md.hdr_tray_total,
	    (unsigned)md.hdr_crc_hi, (unsigned)md.hdr_crc_lo,
	    (unsigned)AppMatrixModbus_LastCommitOk());
	l1[sizeof(l1) - 1u] = '\0';
	serial_diag_append(&k, l1);

	if (gd.first_valid != 0u)
	{
		(void)snprintf(
		    l1, sizeof(l1),
		    "GEOM bad %u first #%u T%u C%u R%u Cl%u ik%ld",
		    (unsigned)gd.bad_count, (unsigned)gd.first_idx,
		    (unsigned)gd.tray_id, (unsigned)gd.col, (unsigned)gd.row,
		    (unsigned)gd.class_id, (long)gd.ik_err);
		l1[sizeof(l1) - 1u] = '\0';
		serial_diag_append(&k, l1);

		(void)snprintf(
		    l1, sizeof(l1),
		    "GEOM uvz %ld %ld %ld xw/yw %.0f %.0f",
		    (long)gd.u, (long)gd.v, (long)gd.z_mm,
		    (double)gd.Xw_mm, (double)gd.Yw_mm);
		l1[sizeof(l1) - 1u] = '\0';
		serial_diag_append(&k, l1);
	}
	else
	{
		serial_diag_append(&k, "GEOM bad 0");
	}

	hex_scroll_feed_from_trace();
	hex_scroll_feed_from_trace_u3();

	(void)snprintf(l1, sizeof(l1),
		       "rtu_crc_fail %lu timeout %lu len_fail %lu commits %lu",
		       (unsigned long)md.crc_fails, (unsigned long)md.to_fails,
		       (unsigned long)md.len_fails,
		       (unsigned long)md.commits_ok);
	l1[sizeof(l1) - 1u] = '\0';
	serial_diag_append(&k, l1);

	for (ir = 0u; ir < HEX_SCROLL_RING_U2; ir++)
	{
		if (k >= SERIAL_DIAG_MAX_LINES)
		{
			break;
		}
		serial_diag_append_wrapped(&k, "U2 MTR ", s_hex_ring[ir]);
	}
	if (k < SERIAL_DIAG_MAX_LINES)
	{
		serial_diag_append(&k, "--- U3 MTX ---");
	}
	for (ir = 0u; ir < HEX_SCROLL_RING_U3; ir++)
	{
		if (k >= SERIAL_DIAG_MAX_LINES)
		{
			break;
		}
		serial_diag_append_wrapped(&k, "U3 MTX ", s_hex_ring_u3[ir]);
	}

	s_serial_diag_nlines = (uint16_t)k;
	vp = matrix_detail_vp_lines();
	if (s_serial_diag_nlines > vp)
	{
		smax = (uint16_t)(s_serial_diag_nlines - vp);
	}
	else
	{
		smax = 0u;
	}
	if (s_detail_scroll > smax)
	{
		s_detail_scroll = smax;
	}

	for (di = 0u; di < vp; di++)
	{
		uint16_t li = (uint16_t)(s_detail_scroll + di);

		if (li < s_serial_diag_nlines)
		{
			copy_trim(s_detail[di], sizeof(s_detail[0]),
				  s_serial_diag_lines[li]);
		}
		else
		{
			s_detail[di][0] = '\0';
		}
	}
}

/*
 * 功能：整页刷新 Raw/Final 明细（Font8x16 全屏列表）。
 * 交互：Refresh 子页分支。
 */
static void paint_detail_page(void)
{
	unsigned i;

	LCD_SetColors(WHITE, BLACK);
	LCD_SetFont(&Font8x16);

	for (i = 0u; i < matrix_detail_vp_lines(); i++)
	{
		uint16_t y = (uint16_t)(i * Font8x16.Height);

		NT35510_Clear(0, y, LCD_X_LENGTH, Font8x16.Height);
		if (s_detail[i][0] != '\0')
		{
			NT35510_DispString_EN(0, y, s_detail[i]);
		}
	}
}

/*
 * 功能：差分刷新主页（顶区状态 + Raw 带）；底栏三键。
 * 交互：MAIN 页。
 */
static void paint_main_page(void)
{
	unsigned i;

	LCD_SetColors(WHITE, BLACK);

	for (i = 0u; i < APP_LINES; i++)
	{
		if (strcmp(s_scr[i], s_scr_prev[i]) != 0)
		{
			uint16_t y = app_main_row_y((uint16_t)i);
			uint16_t h = app_main_row_height((uint16_t)i);

			LCD_SetFont(&Font8x16);
			LCD_SetColors(WHITE, BLACK);
			NT35510_Clear(0, y, LCD_X_LENGTH, h);
			if (s_scr[i][0] != '\0')
			{
				NT35510_DispString_EN(0, y, s_scr[i]);
			}
			copy_trim(s_scr_prev[i], sizeof(s_scr_prev[0]), s_scr[i]);
		}
	}

	ui_paint_bottom_buttons(3u, s_main_btn_lbls, ui_pressed_idx());
}

/*
 * 功能：初始化文本缓存、默认页与字体基线（首帧 Refresh 会做全屏清）。
 * 交互：main 在 LCD 硬件 Init 之后。
 */
void AppDisplay_Init(void)
{
	unsigned i;
	unsigned r;

	memset(s_fault, 0, sizeof(s_fault));
	memset(s_runflg, 0, sizeof(s_runflg));
	memset(s_fsm_line, 0, sizeof(s_fsm_line));
	memset(s_serv_main, 0, sizeof(s_serv_main));
	memset(s_serv_step, 0, sizeof(s_serv_step));
	strncpy(s_fault, "---", sizeof(s_fault) - 1u);
	strncpy(s_runflg, "---", sizeof(s_runflg) - 1u);
	strncpy(s_fsm_line, "FSM init", sizeof(s_fsm_line) - 1u);

	for (i = 0u; i < APP_LINES; i++)
	{
		s_scr[i][0] = '\0';
		s_scr_prev[i][0] = '\0';
	}

	for (r = 0u; r < HEX_VISIBLE_ROWS; r++)
	{
		s_hex_lines[r][0] = '\0';
		s_hex_prev[r][0] = '\0';
	}

	for (r = 0u; r < HEX_SCROLL_RING_U2; r++)
	{
		copy_trim(s_hex_ring[r], sizeof(s_hex_ring[0]), "---");
	}
	for (r = 0u; r < HEX_SCROLL_RING_U3; r++)
	{
		copy_trim(s_hex_ring_u3[r], sizeof(s_hex_ring_u3[0]), "---");
	}

	memset(s_detail, 0, sizeof(s_detail));

	s_ind = APP_IND_IDLE;
	s_last_ui_tick = 0u;
	s_page = APP_DISPLAY_PAGE_MAIN;
	s_last_painted_page = -1;
	s_detail_scroll = 0u;
	s_main_raw_scroll = 0u;
	s_serial_diag_nlines = 0u;
	s_final_autoscroll_off = 0u;
	s_final_autoscroll_tick = 0u;
	s_touch_bar_armed = 0u;
	s_touch_bar_seg = -1;
	s_touch_pg_at_down = APP_DISPLAY_PAGE_MAIN;
	s_touch_norm_idx = -1;
	s_touch_pick_prev_nx = -1;

	LCD_SetFont(&Font8x16);
	LCD_SetColors(WHITE, BLACK);
}

/*
 * 功能：按 CFG_UI_REFRESH_MS 节流整页绘制；处理页切换全清与 RGB 指示。
 * 交互：main 主循环；调用 compose_* / paint_*。
 */
void AppDisplay_Refresh(uint32_t tick_ms)
{
	apply_indicator(tick_ms);

	if ((tick_ms - s_last_ui_tick) < CFG_UI_REFRESH_MS)
	{
		return;
	}

	s_last_ui_tick = tick_ms;

	if ((int32_t)s_page != s_last_painted_page)
	{
		NT35510_Clear(0, 0, LCD_X_LENGTH, LCD_Y_LENGTH);
		memset(s_scr_prev, 0, sizeof(s_scr_prev));
		memset(s_hex_prev, 0, sizeof(s_hex_prev));
		s_last_painted_page = (int32_t)s_page;
	}

	switch (s_page)
	{
	case APP_DISPLAY_PAGE_RAW_MATRIX:
		compose_detail_raw();
		paint_detail_page();
		ui_paint_bottom_buttons(3u, s_mat_btn_lbls, ui_pressed_idx());
		break;

	case APP_DISPLAY_PAGE_FINAL_MATRIX:
		compose_detail_final();
		paint_detail_page();
		ui_paint_bottom_buttons(3u, s_mat_btn_lbls, ui_pressed_idx());
		break;

	case APP_DISPLAY_PAGE_SERIAL_DIAG:
		compose_serial_diag();
		paint_detail_page();
		ui_paint_bottom_buttons(3u, s_mat_btn_lbls, ui_pressed_idx());
		break;

	case APP_DISPLAY_PAGE_MAIN:
	default:
		compose_screen_main(tick_ms);
		paint_main_page();
		break;
	}
}

void AppDisplay_KickRefresh(void)
{
	s_last_ui_tick = 0u;
}

/*
 * 功能：底栏按下立即执行的导航（主页三键 + 子页 Back）。
 */
static void touch_bar_on_down(AppDisplayPage_te pg, int8_t seg)
{
	if (pg == APP_DISPLAY_PAGE_MAIN)
	{
		switch (seg)
		{
		case 0:
			s_detail_scroll = 0u;
			s_page = APP_DISPLAY_PAGE_SERIAL_DIAG;
			break;
		case 1:
			s_main_raw_scroll = 0u;
			s_final_autoscroll_off = 0u;
			s_final_autoscroll_tick = 0u;
			break;
		case 2:
			s_detail_scroll = 0u;
			s_page = APP_DISPLAY_PAGE_FINAL_MATRIX;
			break;
		default:
			break;
		}
	}
	else if ((pg == APP_DISPLAY_PAGE_RAW_MATRIX) ||
		 (pg == APP_DISPLAY_PAGE_FINAL_MATRIX) ||
		 (pg == APP_DISPLAY_PAGE_SERIAL_DIAG))
	{
		if (seg == 1)
		{
			s_page = APP_DISPLAY_PAGE_MAIN;
		}
	}
}

/*
 * 功能：子页底栏 Up/Dn 仅在抬起时滚动（seg 0 / 2）。
 */
static void touch_bar_on_up_sub_scroll(AppDisplayPage_te pg, int8_t down)
{
	uint16_t total;
	uint16_t smax;
	uint16_t vp = matrix_detail_vp_lines();

	if (pg == APP_DISPLAY_PAGE_RAW_MATRIX)
	{
		total = AppProtocol_GetRawLastLineCount();
		smax = detail_scroll_max_raw(total);
	}
	else if (pg == APP_DISPLAY_PAGE_FINAL_MATRIX)
	{
		total = AppMatrix_GetValidCount();
		smax = detail_scroll_max_final(total);
	}
	else
	{
		total = s_serial_diag_nlines;
		if (total > vp)
		{
			smax = (uint16_t)(total - vp);
		}
		else
		{
			smax = 0u;
		}
	}

	if (down == 0)
	{
		if (total == 0u)
		{
			if (pg == APP_DISPLAY_PAGE_SERIAL_DIAG)
			{
				AppDisplay_SetRunFlagText("SER empty");
			}
			else
			{
				AppDisplay_SetRunFlagText(
				    pg == APP_DISPLAY_PAGE_RAW_MATRIX ?
					"RAW empty" :
					"FINAL empty");
			}
			AppDisplay_KickRefresh();
		}
		else if (s_detail_scroll > 0u)
		{
			s_detail_scroll--;
		}
		else
		{
			if (pg == APP_DISPLAY_PAGE_SERIAL_DIAG)
			{
				AppDisplay_SetRunFlagText("SER first");
			}
			else
			{
				AppDisplay_SetRunFlagText(
				    pg == APP_DISPLAY_PAGE_RAW_MATRIX ?
					"RAW first" :
					"FINAL first");
			}
			AppDisplay_KickRefresh();
		}
	}
	else if (down == 1)
	{
		return;
	}
	else
	{
		if (total == 0u)
		{
			if (pg == APP_DISPLAY_PAGE_SERIAL_DIAG)
			{
				AppDisplay_SetRunFlagText("SER empty");
			}
			else
			{
				AppDisplay_SetRunFlagText(
				    pg == APP_DISPLAY_PAGE_RAW_MATRIX ?
					"RAW empty" :
					"FINAL empty");
			}
			AppDisplay_KickRefresh();
		}
		else if (s_detail_scroll < smax)
		{
			s_detail_scroll++;
		}
		else
		{
			if (pg == APP_DISPLAY_PAGE_SERIAL_DIAG)
			{
				AppDisplay_SetRunFlagText("SER last");
			}
			else
			{
				AppDisplay_SetRunFlagText(
				    pg == APP_DISPLAY_PAGE_RAW_MATRIX ?
					"RAW last" :
					"FINAL last");
			}
			AppDisplay_KickRefresh();
		}
	}
}

/*
 * 功能：触摸底栏：Down 归一化后立即导航；Up 仅处理子页滚动。
 * 交互：GT911。
 */
void Touch_Button_Down(int32_t x, int32_t y)
{
	int32_t nx;

	if (touch_normalize_bottom_bar_press(x, y, &nx) == 0u)
	{
		s_touch_bar_armed = 0u;
		s_touch_bar_seg = -1;
		return;
	}

	s_touch_bar_armed = 1u;

	if (s_page == APP_DISPLAY_PAGE_MAIN)
	{
		s_touch_bar_seg = touch_bottom_seg_from_x(nx, 3u);
	}
	else if ((s_page == APP_DISPLAY_PAGE_RAW_MATRIX) ||
		 (s_page == APP_DISPLAY_PAGE_FINAL_MATRIX) ||
		 (s_page == APP_DISPLAY_PAGE_SERIAL_DIAG))
	{
		s_touch_bar_seg = touch_bottom_seg_from_x(nx, 3u);
	}
	else
	{
		s_touch_bar_armed = 0u;
		s_touch_bar_seg = -1;
		return;
	}

	if (s_touch_bar_seg < 0)
	{
		s_touch_bar_armed = 0u;
		s_touch_bar_seg = -1;
		return;
	}

	s_touch_pg_at_down = s_page;
	touch_bar_on_down(s_touch_pg_at_down, s_touch_bar_seg);
	AppDisplay_KickRefresh();
}

void Touch_Button_Up(int32_t x, int32_t y)
{
	int8_t           down = s_touch_bar_seg;
	int32_t          nx;
	int8_t           rel_seg;
	AppDisplayPage_te pg = s_touch_pg_at_down;

	s_touch_bar_armed = 0u;
	s_touch_bar_seg = -1;
	AppDisplay_KickRefresh();

	if (down < 0)
	{
		if ((s_page == APP_DISPLAY_PAGE_RAW_MATRIX) ||
		    (s_page == APP_DISPLAY_PAGE_FINAL_MATRIX) ||
		    (s_page == APP_DISPLAY_PAGE_SERIAL_DIAG))
		{
			if (touch_normalize_bottom_bar_press(x, y, &nx) != 0u)
			{
				int8_t sg = touch_bottom_seg_from_x(nx, 3u);

				if (sg == 1)
				{
					s_page = APP_DISPLAY_PAGE_MAIN;
					s_touch_norm_idx = -1;
					AppDisplay_KickRefresh();
					return;
				}
			}
		}
		if (s_page == APP_DISPLAY_PAGE_MAIN)
		{
			if (touch_normalize_bottom_bar_press(x, y, &nx) != 0u)
			{
				int8_t sg = touch_bottom_seg_from_x(nx, 3u);

				touch_bar_on_down(APP_DISPLAY_PAGE_MAIN, sg);
				s_touch_norm_idx = -1;
				AppDisplay_KickRefresh();
				return;
			}
			if (touch_normalize_main_raw_zone(x, y, &nx) != 0u)
			{
				uint16_t vis = main_raw_visible_lines();
				uint16_t total =
				    AppProtocol_GetRawLastLineCount();
				uint16_t smax;

				if (vis == 0u)
				{
					s_touch_norm_idx = -1;
					return;
				}
				smax = (total > vis) ? (uint16_t)(total - vis) :
						     0u;
				if ((uint32_t)nx <
				    ((uint32_t)LCD_X_LENGTH / 2u))
				{
					if (s_main_raw_scroll > 0u)
					{
						s_main_raw_scroll--;
					}
				}
				else
				{
					if (s_main_raw_scroll < smax)
					{
						s_main_raw_scroll++;
					}
				}
				s_touch_norm_idx = -1;
				AppDisplay_KickRefresh();
				return;
			}
		}
		s_touch_norm_idx = -1;
		return;
	}
	if (touch_normalize_bottom_bar_release_xy(x, y, &nx) == 0u)
	{
		s_touch_norm_idx = -1;
		return;
	}

	rel_seg = -1;
	if (pg == APP_DISPLAY_PAGE_MAIN)
	{
		rel_seg = touch_bottom_seg_from_x(nx, 3u);
	}
	else if ((pg == APP_DISPLAY_PAGE_RAW_MATRIX) ||
		 (pg == APP_DISPLAY_PAGE_FINAL_MATRIX) ||
		 (pg == APP_DISPLAY_PAGE_SERIAL_DIAG))
	{
		rel_seg = touch_bottom_seg_from_x(nx, 3u);
	}
	else
	{
		s_touch_norm_idx = -1;
		return;
	}

	if (rel_seg < 0)
	{
		s_touch_norm_idx = -1;
		return;
	}

	if (rel_seg != down)
	{
		s_touch_norm_idx = -1;
		return;
	}

	if (pg == APP_DISPLAY_PAGE_MAIN)
	{
		s_touch_norm_idx = -1;
		return;
	}

	touch_bar_on_up_sub_scroll(pg, down);
	s_touch_norm_idx = -1;
}
