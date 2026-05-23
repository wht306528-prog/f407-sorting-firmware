/**
 * app_tcp_server.c — lwIP raw TCP 服务端（Matrix 文本帧入口）
 *
 * 职责：bind 端口，accept，把 pbuf 流水按序喂给 AppProtocol_OnStream（采样关闸时跳过，仍确认收包）。
 * 注意：与 RS485/Modbus 不使用同一 RAM 缓冲；断线清矩阵见 app_protocol。
 *
 * 【量产 Keil 目标】若为双路 RS485、未编 lwIP，请勿把本文件加入工程以免误链 lwip。
 * 来源/丢弃：RJ45 TCP 载荷先经 `AppProtocol_ShouldAcceptStream`（关闸则不入解析），再经
 * `AppSort_ShouldDiscardTcpMatrixStream()`；机械臂分拣子步（FL2/FL3，`app_sort.c`）为 1 时静默丢弃载荷
 * （仍 `tcp_recved`），避免分拣中快照被新一帧改写。新连接注册 `tcp_err`，RST/异常与 FIN 均清 drv_network。
 */
#include "app_tcp_server.h"
#include "app_protocol.h"
#include "app_matrix.h"
#include "app_sort.h"

#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
#include "netconf.h"

static struct tcp_pcb *s_listen_pcb; /* 监听 PCB：整个进程就一个 */

/*
 * 功能：清除屏用远端占位（FIN / 异常 / RST 共用）。
 */
static void matrix_tcp_clear_client_net(void)
{
	drv_network.net_connect = 0u;
	drv_network.net_remote_port = 0u;
	drv_network.net_remote_ip1 = 0u;
	drv_network.net_remote_ip2 = 0u;
	drv_network.net_remote_ip3 = 0u;
	drv_network.net_remote_ip4 = 0u;
}

/*
 * 功能：lwIP tcp_err：对端 RST、超时等；此回调到达时连接 PCB 已由栈回收，勿再 tcp_close。
 */
static void matrix_tcp_err(void *arg, err_t err)
{
	(void)arg;
	(void)err;
	matrix_tcp_clear_client_net();
	AppProtocol_Init();
	AppMatrix_Clear();
}

/*
 * 功能：lwIP TCP recv 回调；消费 pbuf 链中的矩阵文本流（或 FIN 时复位协议与矩阵）。
 * 交互：外部由 lwIP 在 Matrix TCP 连接上调用；依 AppProtocol_ShouldAcceptStream 与
 * AppSort_ShouldDiscardTcpMatrixStream 决定是否喂 AppProtocol_OnStream；FIN 分支调用 AppProtocol_Init、AppMatrix_Clear、tcp_close 并刷新 drv_network。
 */
static err_t matrix_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
	LWIP_UNUSED_ARG(arg); /* raw 服务端未用回调透传指针 */

	if (pcb == NULL) { /* PCB 空： lwIP 异常路径 */
		return ERR_ARG;
	}
	if (err != ERR_OK) { /* 驱动/栈错误：释放 pbuf 后把错误往上抛 */
		if (p != NULL) {
			pbuf_free(p);
		}
		matrix_tcp_clear_client_net();
		AppProtocol_Init();
		AppMatrix_Clear();
		return err;
	}
	if (p == NULL) { /* 对端 FIN：会话结束；清协议与矩阵，避免断线后继续用上一帧分拣 */
		AppProtocol_Init();
		AppMatrix_Clear();
		tcp_close(pcb); /* 跟上位机关闭会话 */
		matrix_tcp_clear_client_net();
		return ERR_OK;
	}

	{
		struct pbuf *q; /* 遍历链表 */

		for (q = p; q != NULL; q = q->next) { /* 每片 payload 都可能是半行 CSV */
			if (AppProtocol_ShouldAcceptStream() == 0u) {
				continue; /* 采样关闸：仍由 lwIP 收包，不喂解析器 */
			}
			if (AppSort_ShouldDiscardTcpMatrixStream() != 0u) {
				continue; /* 机械臂分拣子步：不喂解析器 */
			}
			AppProtocol_OnStream((const uint8_t *)q->payload,
					     q->len); /* 流式解析，不假设一次一行 */
		}
	}
	tcp_recved(pcb, p->tot_len); /* 告诉 lwIP 应用层已消费多少 */
	pbuf_free(p); /* 释放整链 */
	return ERR_OK;
}

/*
 * 功能：新 TCP 接入时绑定 matrix_recv，记录远端 IP/端口并复位协议解析与矩阵缓冲。
 * 交互：外部由 lwIP tcp_accept 调用；写入全局 drv_network、调用 tcp_recv/tcp_arg/AppProtocol_Init/AppMatrix_Clear。
 */
static err_t matrix_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
	LWIP_UNUSED_ARG(arg);

	if ((newpcb == NULL) || (err != ERR_OK)) { /* 非法握手直接失败 */
		return ERR_VAL;
	}

	tcp_arg(newpcb, NULL); /* 无附加上下文 */
	tcp_recv(newpcb, matrix_recv); /* 绑定收包 */
	tcp_err(newpcb, matrix_tcp_err);
	drv_network.net_connect = 1u; /* 已连接标志 */
	drv_network.net_remote_port = newpcb->remote_port; /* 远端端口 */
	drv_network.net_remote_ip1 = ip4_addr1(&newpcb->remote_ip); /* IPv4 逐字节 */
	drv_network.net_remote_ip2 = ip4_addr2(&newpcb->remote_ip);
	drv_network.net_remote_ip3 = ip4_addr3(&newpcb->remote_ip);
	drv_network.net_remote_ip4 = ip4_addr4(&newpcb->remote_ip);
	/* AppProtocol_Init：s_gate_open=1，采样窗口清零；新 TCP 从 START 起收完整帧 */
	AppProtocol_Init(); /* 新连接重新建帧状态机 */
	AppMatrix_Clear(); /* 换客户端默认清矩阵，防串数据 */
	return ERR_OK; /* 接受连接 */
}

/*
 * 功能：绑定并监听 MATRIX_TCP_SERVER_PORT，供上位机 ASCII 矩阵帧接入。
 * 交互：外部由 AppRos2Bridge_Init 链式调用一次；调用 tcp_* lwIP API、AppProtocol_Init、tcp_accept 注册 matrix_accept。
 */
void AppTcpMatrix_Init(void) {
	struct tcp_pcb *pcb; /* 临时新 PCB */
	err_t            er; /* lwIP 错误码 */
	ip_addr_t        local_ip;

	drv_network.net_listen = 0u; /* listen 未建立或未成功 */
	AppProtocol_Init(); /* 保险：服务起之前 parser 归零 */

	pcb = tcp_new(); /* 申请 PCB */
	if (pcb == NULL) { /* 内存不够：静默失败，屏上会显示未监听 */
		return;
	}
	IP4_ADDR(&local_ip, LOCAL_IP_ADDR0, LOCAL_IP_ADDR1, LOCAL_IP_ADDR2,
		 LOCAL_IP_ADDR3);
	er = tcp_bind(pcb, &local_ip, MATRIX_TCP_SERVER_PORT); /* 绑定本机静态 IP */
	if (er != ERR_OK) { /* 端口占用或参数错 */
		tcp_close(pcb);
		return;
	}
	s_listen_pcb = tcp_listen(pcb); /* 转监听态 */
	if (s_listen_pcb == NULL) { /* listen 失败 */
		tcp_close(pcb);
		return;
	}
	tcp_accept(s_listen_pcb, matrix_accept); /* 等上位机连入 */
	drv_network.net_listen = 1u; /* tcp_listen + accept 注册成功，LISTEN 就绪 */
}
