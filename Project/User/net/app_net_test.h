/**
 * app_net_test.h — 屏内网络测试：ICMP 扫描当前 /24 + 探测 CFG_NET_TEST_TARGET_* 的 ICMP 与 TCP 端口
 */
#ifndef APP_NET_TEST_H
#define APP_NET_TEST_H

#include <stdint.h>

/** 与 app_display 网络页列表区可见行数一致（MATRIX_DETAIL_LINES=27，摘要占 8 行，列表 19 行） */
#define APP_NETTEST_VISIBLE_LIST_LINES 19u

/** 自动全量网测（ICMP /24 + 目标 TCP）周期，与 app_net_test.c 一致 */
#define APP_NETTEST_AUTO_CYCLE_MS 12000u

typedef struct
{
	uint8_t  link_up;
	uint8_t  no_link_at_start;
	uint8_t  scan_busy;
	uint8_t  scan_done;
	uint16_t alive_count;
	uint8_t  icmp_sweep_done;
	uint8_t  icmp154_ok; /* 扫描结束后据 alive 列表判定 */
	uint8_t  tcp5000_ok;
	int      tcp_err; /* lwIP err_t；超时为 ERR_ABRT=-13（依具体 lwip 裁剪） */
	char     tcp_err_tag[12];
	uint8_t  cur_host; /* 正在 ping 的最后一字节；0 表示空闲 */
	uint16_t list_scroll;
	uint16_t list_line_count;
} AppNetTestUi_t;

void AppNetTest_Init(void);
void AppNetTest_Poll(uint32_t tick_ms);
void AppNetTest_Start(uint32_t tick_ms);
void AppNetTest_ListScrollNext(void);
void AppNetTest_GetUi(AppNetTestUi_t *out);
/** vis_line: 0 .. APP_NETTEST_VISIBLE_LIST_LINES-1；输出到 buf */
void AppNetTest_FormatVisibleListLine(uint16_t vis_line, char *buf, unsigned cap);

#endif
