/*
 * [LIB_ANNOT_ZH_V1]
 * STM32\CMSIS\SPL / 第三方库源码；非分拣业务逻辑。
 * 说明：寄存器与外设语义以芯片手册与原库为准；工程业务见 F407/docs/F407_GUIDE.md。
 * 此块仅追加中文导读，不改变版权与编译行为；升级库时可按 marker 检索后整文件替换。
 */
#ifndef __ETHERNETIF_H__
#define __ETHERNETIF_H__


#include "lwip/err.h"
#include "lwip/netif.h"

err_t ethernetif_init(struct netif *netif);

#endif 
