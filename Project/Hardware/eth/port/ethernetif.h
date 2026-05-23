/**
 * ethernetif.h — lwIP netif 与 STM32 ETH MAC/DMA 之间的移植层接口
 *
 * 实现：Hardware/eth/port/ethernetif.c；由 netconf.c 注册 netif。
 */
#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

err_t ethernetif_init(struct netif *netif);
err_t ethernetif_input(struct netif *netif);

#endif 
