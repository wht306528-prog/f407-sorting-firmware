/**
 * app_net_test.c — lwIP 非阻塞网段 ICMP 扫描 + TCP 连接测试（与 Matrix TCP 服务端共存）
 *
 * 上电后 Poll 中间隔 APP_NETTEST_AUTO_CYCLE_MS 自动启动一轮（全 /24 ICMP + 目标 TCP），
 * 亦可屏上 Start 手动触发；用于开机即见 154:5000 等连通性，无需先点开网络页。
 *
 * 【量产 Keil 目标】未编以太网/lwIP 时勿将本文件加入工程。
 */
#include "app_net_test.h"

#include "global_config.h"
#include "netconf.h"

#include "lwip/tcp.h"
#include "lwip/raw.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/ip.h"
#include "lwip/ip_addr.h"
#include "lwip/def.h"
#include "lwip/err.h"

#include <stdio.h>
#include <string.h>

#define NETTEST_ICMP_ID    0xF407u
#define NETTEST_LINE_CAP   120u


typedef enum {
	NT_IDLE = 0,
	NT_PING_WAIT,
	NT_TCP_CONNECTING,
	NT_DONE
} NetTestPhase_t;

static struct raw_pcb *s_raw;
static struct tcp_pcb *s_tcp_client;

static volatile uint8_t s_icmp_got;
static ip_addr_t        s_icmp_from;

static NetTestPhase_t s_phase;
static uint8_t        s_next_octet; /* 当前正在探测的最后一字节 1..254 */
static ip_addr_t      s_pending_dst;
static uint8_t        s_pending_oct;
static uint32_t       s_deadline_ms;

static uint8_t  s_alive[254];
static uint8_t  s_alive_n;

static uint8_t  s_scan_running;
static uint8_t  s_scan_done;
static uint8_t  s_no_link_at_start;
static uint8_t  s_icmp_sweep_done;
static uint8_t  s_icmp154_ok;
static uint8_t  s_tcp_ok;
static int      s_tcp_last_err;

static uint16_t s_list_scroll;
static char     s_disp_lines[40][NETTEST_LINE_CAP];
static uint8_t  s_disp_n;
static uint8_t  s_disp_dirty;

/** 上电后周期性全量 ICMP /24 + 目标 TCP 探测，不阻塞主循环 */
static uint8_t  s_auto_enabled = 1u;
static uint32_t s_auto_next_ms;

static void nettest_finish_all(void)
{
	if (s_scan_done != 0u) {
		return;
	}
	s_phase = NT_DONE;
	s_scan_running = 0u;
	s_scan_done = 1u;
	s_tcp_client = NULL;
}

static void copy_trim_local(char *dst, unsigned cap, const char *src)
{
	if ((dst == NULL) || (cap == 0u)) {
		return;
	}
	if (src == NULL) {
		dst[0] = '\0';
		return;
	}
	(void)strncpy(dst, src, cap - 1u);
	dst[cap - 1u] = '\0';
}

static const char *lwip_err_tag(int e)
{
	switch (e) {
	case ERR_OK:
		return "OK";
	case ERR_MEM:
		return "MEM";
	case ERR_BUF:
		return "BUF";
	case ERR_TIMEOUT:
		return "TMO";
	case ERR_RTE:
		return "RTE";
	case ERR_INPROGRESS:
		return "BUSY";
	case ERR_VAL:
		return "VAL";
	case ERR_WOULDBLOCK:
		return "WBLK";
	case ERR_USE:
		return "USE";
	case ERR_IF:
		return "IF";
	case ERR_ISCONN:
		return "ISCON";
	case ERR_ABRT:
		return "ABRT";
	case ERR_RST:
		return "RST";
	case ERR_CLSD:
		return "CLSD";
	case ERR_CONN:
		return "CONN";
	default:
		return "?";
	}
}

static void nettest_make_ip(uint8_t lo, ip_addr_t *o)
{
	ip_addr_t na;

	ip_addr_get_network(&na, &gnetif.ip_addr, &gnetif.netmask);
	o->addr = na.addr | ((u32_t)lo << 24);
}

static void alive_add(uint8_t lo)
{
	uint16_t i;
	uint16_t j;

	if (s_alive_n >= (uint8_t)sizeof(s_alive)) {
		return;
	}
	for (i = 0u; i < (uint16_t)s_alive_n; i++) {
		if (s_alive[i] == lo) {
			return;
		}
		if (s_alive[i] > lo) {
			break;
		}
	}
	for (j = (uint16_t)s_alive_n; j > i; j--) {
		s_alive[j] = s_alive[j - 1u];
	}
	s_alive[i] = lo;
	s_alive_n++;
	s_disp_dirty = 1u;
}

static void finish_icmp_phase(void)
{
	unsigned i;

	s_icmp_sweep_done = 1u;
	s_icmp154_ok = 0u;
	for (i = 0u; i < (unsigned)s_alive_n; i++) {
		if (s_alive[i] == (uint8_t)CFG_NET_TEST_TARGET_IP3) {
			s_icmp154_ok = 1u;
			break;
		}
	}
}

static void rebuild_disp_lines(void)
{
	unsigned i;
	unsigned col;
	char     line[NETTEST_LINE_CAP];

	s_disp_n = 0u;
	line[0] = '\0';
	col = 0u;

	for (i = 0u; i < (unsigned)s_alive_n; i++) {
		char  piece[32];
		size_t plen;
		size_t ll;

		(void)snprintf(piece, sizeof(piece), "%u.%u.%u.%u ",
			       (unsigned)ip4_addr1_16(&gnetif.ip_addr),
			       (unsigned)ip4_addr2_16(&gnetif.ip_addr),
			       (unsigned)ip4_addr3_16(&gnetif.ip_addr),
			       (unsigned)s_alive[i]);
		piece[sizeof(piece) - 1u] = '\0';
		plen = strlen(piece);
		ll = strlen(line);
		if ((ll + plen + 4u) >= (size_t)NETTEST_LINE_CAP) {
			copy_trim_local(s_disp_lines[s_disp_n], (unsigned)sizeof(s_disp_lines[0]),
					line);
			s_disp_n = (uint8_t)(s_disp_n + 1u);
			if (s_disp_n >= (uint8_t)(sizeof(s_disp_lines) / sizeof(s_disp_lines[0]))) {
				return;
			}
			line[0] = '\0';
			col = 0u;
		}
		(void)strncat(line, piece, sizeof(line) - strlen(line) - 1u);
		col++;
		(void)col;
	}
	if (line[0] != '\0') {
		copy_trim_local(s_disp_lines[s_disp_n], (unsigned)sizeof(s_disp_lines[0]), line);
		s_disp_n = (uint8_t)(s_disp_n + 1u);
	}
}

static err_t nettest_send_ping(ip_addr_t *dst)
{
	struct pbuf *p;
	struct icmp_echo_hdr *iecho;
	err_t      er;

	p = pbuf_alloc(PBUF_IP, sizeof(struct icmp_echo_hdr), PBUF_RAM);
	if (p == NULL) {
		return ERR_MEM;
	}
	iecho = (struct icmp_echo_hdr *)p->payload;
	ICMPH_TYPE_SET(iecho, ICMP_ECHO);
	ICMPH_CODE_SET(iecho, 0);
	iecho->chksum = 0;
	iecho->id = PP_HTONS(NETTEST_ICMP_ID);
	iecho->seqno = PP_HTONS((u16_t)s_next_octet);
	iecho->chksum = inet_chksum(iecho, sizeof(struct icmp_echo_hdr));

	ip_addr_copy(s_pending_dst, *dst);
	s_pending_oct = (uint8_t)s_next_octet;
	s_icmp_got = 0u;

	er = raw_sendto(s_raw, p, dst);
	pbuf_free(p);
	return er;
}

static u8_t nettest_raw_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p,
			     ip_addr_t *addr)
{
	struct ip_hdr          *iph;
	u16_t                   iphdrlen;
	struct icmp_echo_hdr *iecho;

	(void)arg;
	(void)pcb;
	if (p == NULL) {
		return 0u;
	}
	iph = (struct ip_hdr *)p->payload;
	iphdrlen = IPH_HL(iph) * 4u;
	if (pbuf_header(p, -(s16_t)iphdrlen)) {
		pbuf_free(p);
		return 1u;
	}
	iecho = (struct icmp_echo_hdr *)p->payload;
	if (p->tot_len < (s16_t)sizeof(struct icmp_echo_hdr)) {
		pbuf_header(p, (s16_t)iphdrlen);
		pbuf_free(p);
		return 0u;
	}
	if (ICMPH_TYPE(iecho) != ICMP_ER) {
		pbuf_header(p, (s16_t)iphdrlen);
		return 0u;
	}
	if (iecho->id != PP_HTONS(NETTEST_ICMP_ID)) {
		pbuf_header(p, (s16_t)iphdrlen);
		return 0u;
	}
	if (iecho->seqno != PP_HTONS((u16_t)s_pending_oct)) {
		pbuf_header(p, (s16_t)iphdrlen);
		return 0u;
	}
	if (!ip_addr_cmp(addr, &s_pending_dst)) {
		pbuf_header(p, (s16_t)iphdrlen);
		return 0u;
	}
	s_icmp_got = 1u;
	ip_addr_copy(s_icmp_from, *addr);
	pbuf_header(p, (s16_t)iphdrlen);
	pbuf_free(p);
	return 1u;
}

static err_t nettest_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void nettest_tcp_err(void *arg, err_t err);
static err_t nettest_tcp_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

static err_t nettest_tcp_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	(void)arg;
	if (err != ERR_OK) {
		if (p != NULL) {
			pbuf_free(p);
		}
		return err;
	}
	if (p == NULL) {
		return ERR_OK;
	}
	tcp_recved(tpcb, p->tot_len);
	pbuf_free(p);
	return ERR_OK;
}

static void nettest_tcp_err(void *arg, err_t err)
{
	(void)arg;
	s_tcp_client = NULL;
	if (s_phase == NT_TCP_CONNECTING) {
		s_tcp_ok = 0u;
		s_tcp_last_err = (int)err;
		nettest_finish_all();
	}
	(void)err;
}

static err_t nettest_tcp_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	(void)arg;
	s_tcp_client = NULL;
	if ((tpcb != NULL) && (err == ERR_OK)) {
		s_tcp_ok = 1u;
		s_tcp_last_err = (int)ERR_OK;
		tcp_close(tpcb);
	} else {
		s_tcp_ok = 0u;
		s_tcp_last_err = (int)err;
		if (tpcb != NULL) {
			tcp_close(tpcb);
		}
	}
	nettest_finish_all();
	return ERR_OK;
}

static void nettest_begin_tcp(uint32_t tick_ms)
{
	ip_addr_t r;

	if ((gnetif.flags & NETIF_FLAG_LINK_UP) == 0u) {
		s_tcp_ok = 0u;
		s_tcp_last_err = (int)ERR_CONN;
		nettest_finish_all();
		return;
	}

	IP4_ADDR(&r, CFG_NET_TEST_TARGET_IP0, CFG_NET_TEST_TARGET_IP1,
		 CFG_NET_TEST_TARGET_IP2, CFG_NET_TEST_TARGET_IP3);

	s_tcp_client = tcp_new();
	if (s_tcp_client == NULL) {
		s_tcp_ok = 0u;
		s_tcp_last_err = (int)ERR_MEM;
		nettest_finish_all();
		return;
	}
	tcp_arg(s_tcp_client, NULL);
	tcp_recv(s_tcp_client, nettest_tcp_recv);
	tcp_err(s_tcp_client, nettest_tcp_err);

	{
		err_t e = tcp_connect(s_tcp_client, &r,
				      (u16_t)CFG_NET_TEST_TARGET_TCP_PORT,
				      nettest_tcp_connected);
		if (e != ERR_OK) {
			tcp_close(s_tcp_client);
			s_tcp_client = NULL;
			s_tcp_ok = 0u;
			s_tcp_last_err = (int)e;
			nettest_finish_all();
			return;
		}
	}

	s_deadline_ms = tick_ms + 3000u;
	s_phase = NT_TCP_CONNECTING;
}

static void send_next_ping_octet(uint32_t tick_ms)
{
	while (s_next_octet <= 254u) {
		if (s_next_octet == (uint8_t)ip4_addr4(&gnetif.ip_addr)) {
			s_next_octet++;
			continue;
		}
		{
			ip_addr_t d;

			nettest_make_ip((uint8_t)s_next_octet, &d);
			(void)nettest_send_ping(&d);
			s_deadline_ms = tick_ms + 300u;
			return;
		}
	}
	finish_icmp_phase();
	if (s_disp_dirty != 0u) {
		rebuild_disp_lines();
		s_disp_dirty = 0u;
	}
	nettest_begin_tcp(tick_ms);
}

void AppNetTest_Init(void)
{
	s_raw = raw_new(IP_PROTO_ICMP);
	if (s_raw != NULL) {
		(void)raw_bind(s_raw, IP_ADDR_ANY);
		raw_recv(s_raw, nettest_raw_recv, NULL);
	}
	s_tcp_client = NULL;
	s_phase = NT_IDLE;
	s_scan_running = 0u;
	s_scan_done = 0u;
	s_icmp_sweep_done = 0u;
	s_alive_n = 0u;
	s_list_scroll = 0u;
	s_disp_n = 0u;
	s_auto_next_ms = 0u;
}

void AppNetTest_Start(uint32_t tick_ms)
{
	s_no_link_at_start = 0u;
	s_tcp_ok = 0u;
	s_tcp_last_err = (int)ERR_OK;
	s_icmp154_ok = 0u;
	s_list_scroll = 0u;

	if (s_auto_enabled != 0u) {
		s_auto_next_ms = tick_ms + APP_NETTEST_AUTO_CYCLE_MS;
	}

	if (s_raw == NULL) {
		s_scan_done = 1u;
		s_scan_running = 0u;
		s_no_link_at_start = 1u;
		return;
	}

	if ((gnetif.flags & NETIF_FLAG_LINK_UP) == 0u) {
		s_no_link_at_start = 1u;
		s_scan_done = 1u;
		s_scan_running = 0u;
		return;
	}

	s_alive_n = 0u;
	s_disp_n = 0u;
	s_disp_dirty = 0u;
	s_icmp_sweep_done = 0u;
	s_next_octet = 1u;
	s_scan_running = 1u;
	s_scan_done = 0u;
	s_phase = NT_PING_WAIT;
	send_next_ping_octet(tick_ms);
}

void AppNetTest_Poll(uint32_t tick_ms)
{
	if (s_disp_dirty != 0u) {
		rebuild_disp_lines();
		s_disp_dirty = 0u;
	}

	if (s_auto_enabled != 0u) {
		uint8_t busy = (uint8_t)(((s_scan_running != 0u) ||
					  (s_phase == NT_TCP_CONNECTING)) ?
					     1u :
					     0u);

		if ((busy == 0u) && ((int32_t)(tick_ms - s_auto_next_ms) >= 0)) {
			AppNetTest_Start(tick_ms);
		}
	}

	if ((s_phase == NT_TCP_CONNECTING) && (s_tcp_client != NULL)) {
		if ((int32_t)(tick_ms - s_deadline_ms) >= 0) {
			struct tcp_pcb *p;

			p = s_tcp_client;
			s_tcp_client = NULL;
			tcp_abort(p);
			s_tcp_ok = 0u;
			s_tcp_last_err = (int)ERR_TIMEOUT;
			nettest_finish_all();
		}
	}

	if (s_phase != NT_PING_WAIT) {
		return;
	}
	if (s_scan_running == 0u) {
		return;
	}

	{
		uint8_t advance = 0u;

		if (s_icmp_got != 0u) {
			if (ip_addr_cmp(&s_icmp_from, &s_pending_dst)) {
				alive_add(s_pending_oct);
			}
			advance = 1u;
		} else if ((int32_t)(tick_ms - s_deadline_ms) >= 0) {
			advance = 1u;
		}

		if (advance == 0u) {
			return;
		}

		s_next_octet++;
		send_next_ping_octet(tick_ms);
	}
}

void AppNetTest_ListScrollNext(void)
{
	uint16_t maxs = 0u;

	s_disp_dirty = 1u;
	rebuild_disp_lines();
	if (s_disp_n > APP_NETTEST_VISIBLE_LIST_LINES) {
		maxs = (uint16_t)(s_disp_n - APP_NETTEST_VISIBLE_LIST_LINES);
	}
	if (maxs == 0u) {
		s_list_scroll = 0u;
		return;
	}
	s_list_scroll = (uint16_t)(s_list_scroll + 8u);
	if (s_list_scroll > maxs) {
		s_list_scroll = 0u;
	}
}

void AppNetTest_GetUi(AppNetTestUi_t *out)
{
	if (out == NULL) {
		return;
	}
	(void)memset(out, 0, sizeof(*out));
	out->link_up =
	    (uint8_t)(((gnetif.flags & NETIF_FLAG_LINK_UP) != 0u) ? 1u : 0u);
	out->no_link_at_start = s_no_link_at_start;
	out->scan_busy =
	    (uint8_t)(((s_scan_running != 0u) || (s_phase == NT_TCP_CONNECTING)) ?
			     1u :
			     0u);
	out->scan_done = s_scan_done;
	out->alive_count = (uint16_t)s_alive_n;
	out->icmp_sweep_done = s_icmp_sweep_done;
	out->icmp154_ok = s_icmp154_ok;
	out->tcp5000_ok = s_tcp_ok;
	out->tcp_err = s_tcp_last_err;
	copy_trim_local(out->tcp_err_tag, sizeof(out->tcp_err_tag),
			lwip_err_tag(s_tcp_last_err));
	out->cur_host = (s_phase == NT_PING_WAIT) ? s_pending_oct : 0u;
	out->list_scroll = s_list_scroll;
	out->list_line_count = (uint16_t)s_disp_n;
}

void AppNetTest_FormatVisibleListLine(uint16_t vis_line, char *buf, unsigned cap)
{
	uint16_t idx;

	if ((buf == NULL) || (cap == 0u)) {
		return;
	}
	buf[0] = '\0';
	if (vis_line >= APP_NETTEST_VISIBLE_LIST_LINES) {
		return;
	}
	if (s_disp_dirty != 0u) {
		rebuild_disp_lines();
		s_disp_dirty = 0u;
	}
	idx = (uint16_t)(s_list_scroll + vis_line);
	if (idx < (uint16_t)s_disp_n) {
		copy_trim_local(buf, cap, s_disp_lines[idx]);
	}
}
