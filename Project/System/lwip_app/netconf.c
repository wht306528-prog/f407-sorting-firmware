/**
 * =============================================================================
 * 模块职责：
 *   初始化 EtherNet + lwIP(TCP/IP) 协议栈骨架，在主循环调用收包与时基任务。
 * =============================================================================
 */
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/tcp_impl.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "LAN8742A.h"
#include "netconf.h"

#include <string.h>

struct netif gnetif;

DRV_NETWORK drv_network;

static uint32_t s_TCPTimer;
static uint32_t s_ARPTimer;
static uint32_t s_LinkTimer;

extern __IO uint32_t EthStatus;

/*
 * 功能：初始化 PHY/MAC、lwIP、默认 netif、gnetif IP/掩码/网关与 TCP 定时器初值。
 * 交互：main 网络启动；回调 ETH_link_callback；读写 drv_network 屏显占位。
 */
void LwIP_Init(void)
{
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;

	(void)memset(&drv_network, 0, sizeof(drv_network));

	(void)ETH_BSP_Config();

	lwip_init();

	IP4_ADDR(&ipaddr, LOCAL_IP_ADDR0, LOCAL_IP_ADDR1, LOCAL_IP_ADDR2,
		 LOCAL_IP_ADDR3);
	IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2,
		 NETMASK_ADDR3);
	IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

	netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init,
		  &ethernet_input);
	netif_set_default(&gnetif);

	if (EthStatus == (ETH_INIT_FLAG | ETH_LINK_FLAG))
	{
		/* PHY 已 link-up 且 MAC 初始化完毕：接口可直接承载 ARP/IP */
		gnetif.flags |= NETIF_FLAG_LINK_UP;
		netif_set_up(&gnetif);
	}
	else
	{
		/* 网线未插或协商失败：置 DOWN，避免 lwIP 误以为以太网可用 */
		netif_set_down(&gnetif);
	}

	netif_set_link_callback(&gnetif, ETH_link_callback);

	s_TCPTimer = 0u;
	s_ARPTimer = 0u;
	s_LinkTimer = 0u;

	drv_network.net_init = 1u;
	drv_network.net_local_ip1 = LOCAL_IP_ADDR0;
	drv_network.net_local_ip2 = LOCAL_IP_ADDR1;
	drv_network.net_local_ip3 = LOCAL_IP_ADDR2;
	drv_network.net_local_ip4 = LOCAL_IP_ADDR3;
	drv_network.net_local_port = MATRIX_TCP_SERVER_PORT;
}

/*
 * 功能：link 就绪时从 MAC DMA 收包并送入 lwIP（ethernetif_input）。
 * 交互：main 主循环高频调用。
 */
void LwIP_Pkt_Handle(void)
{
	if (EthStatus != (ETH_INIT_FLAG | ETH_LINK_FLAG))
	{
		/* link 未就绪：跳过 DMA 收包，防止读到无效描述符 */
		return;
	}
	(void)ethernetif_input(&gnetif);
}

/*
 * 功能：lwIP 周期任务：tcp_tmr、ARP 超时、PHY 链路巡检。
 * 交互：main 传入 SysTick_ms；调用 ETH_CheckLinkStatus。
 */
void LwIP_Periodic_Handle(uint32_t localtime)
{
	if (localtime - s_TCPTimer >= TCP_TMR_INTERVAL)
	{
		s_TCPTimer = localtime;
		tcp_tmr();
	}
	if ((localtime - s_ARPTimer) >= ARP_TMR_INTERVAL)
	{
		s_ARPTimer = localtime;
		etharp_tmr();
	}
	if ((localtime - s_LinkTimer) >= LINK_TIMER_INTERVAL)
	{
		s_LinkTimer = localtime;
		/* 周期性读 PHY BSR，刷新 EthStatus 并在回调里联动 netif */
		ETH_CheckLinkStatus(ETHERNET_PHY_ADDRESS);
	}
}
