/**
 * netconf.h — lwIP 网络初始化与 `DRV_NETWORK` LCD 占位状态（实现 netconf.c）
 *
 * LOCAL_IP_* / MAC 等宏优先定义在 global_config.h；本头仅保留 lwIP 示例兼容别名。
 */
#ifndef __NETCONF_H
#define __NETCONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/** IP/端口/MAC 等以 global_config.h 为**唯一**真源；本头文件只做别名，方便 lwIP 旧代码名不改 */
#include "global_config.h"

#include "lwip/netif.h"

extern struct netif gnetif;

/*
 * DRV_NETWORK：
 * - 本项目用于在 LCD 或其它模块上展示远端/近端端口与连接状态占位。
 */

typedef struct
{
	uint8_t  net_init;
	uint8_t  net_type;
	uint8_t  net_connect;
	uint8_t  net_local_ip1;
	uint8_t  net_local_ip2;
	uint8_t  net_local_ip3;
	uint8_t  net_local_ip4;
	uint16_t net_local_port;
	uint8_t  net_remote_ip1;
	uint8_t  net_remote_ip2;
	uint8_t  net_remote_ip3;
	uint8_t  net_remote_ip4;
	uint16_t net_remote_port;
	/** Matrix TCP listen 已建立（lwIP tcp_listen 成功） */
	uint8_t  net_listen;
} DRV_NETWORK;

extern DRV_NETWORK drv_network;

/* 关闭 DHCP：使用静态 IP（鲁班猫 3588 网段常为 192.168.137.x）*/
/* #define USE_DHCP */

#ifndef LINK_TIMER_INTERVAL
#define LINK_TIMER_INTERVAL        CFG_NET_LINK_TIMER_INTERVAL_MS
#endif

#define RMII_MODE

/*
 * PHY 硬件地址：
 * LAN8720 常见为 0x00（由 PHYAD[2:0] 引脚上拉/下拉决定），若网线协商失败请先修改此宏。
 */

#define LOCAL_IP_ADDR0              CFG_NET_LOCAL_IP0
#define LOCAL_IP_ADDR1              CFG_NET_LOCAL_IP1
#define LOCAL_IP_ADDR2              CFG_NET_LOCAL_IP2
#define LOCAL_IP_ADDR3              CFG_NET_LOCAL_IP3

#define NETMASK_ADDR0               CFG_NET_NETMASK0
#define NETMASK_ADDR1               CFG_NET_NETMASK1
#define NETMASK_ADDR2               CFG_NET_NETMASK2
#define NETMASK_ADDR3               CFG_NET_NETMASK3

/*
 * 网关：PC 主机共享网卡时常见为 .1；若局域网无网关可填与 IP 同一网段的任意路由。
 */

#define GW_ADDR0                    CFG_NET_GW0
#define GW_ADDR1                    CFG_NET_GW1
#define GW_ADDR2                    CFG_NET_GW2
#define GW_ADDR3                    CFG_NET_GW3

/*
 * STM32 ETH 二层地址（局域网内需唯一）。
 */

#define MAC_ADDR0                   CFG_NET_MAC0
#define MAC_ADDR1                   CFG_NET_MAC1
#define MAC_ADDR2                   CFG_NET_MAC2
#define MAC_ADDR3                   CFG_NET_MAC3
#define MAC_ADDR4                   CFG_NET_MAC4
#define MAC_ADDR5                   CFG_NET_MAC5

/*
 * ROS2 TCP 桥接服务端端口。
 */

#ifndef MATRIX_TCP_SERVER_PORT
#define MATRIX_TCP_SERVER_PORT      CFG_NET_MATRIX_TCP_SERVER_PORT
#endif

void LwIP_Init(void);
void LwIP_Pkt_Handle(void);
void LwIP_Periodic_Handle(uint32_t localtime);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */
