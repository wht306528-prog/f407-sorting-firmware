/**
 * bsp_uart2.h — 【兼容占位】转发到 bsp_uart（USART2 PA2/PA3 + PC0 DE）
 *
 * 历史 AlarmLight 等曾用 BSP_USART2_* 符号；现电机/灯 Modbus 与 `bsp_uart.c` 共用同一 USART2 硬件。
 * `BSP_USART2_SendBlocking` 仍经 ModbusMaster_SendRawBlocking 以保持互斥。
 */
#ifndef BSP_UART2_H
#define BSP_UART2_H

#include "stm32f4xx.h"
#include <stdint.h>

/** @deprecated 等价于 NOP；由 BSP_Uart_HW_Init(CFG_RS485_BAUD) 完成 USART2 初始化 */
void BSP_USART2_Init(uint32_t baud);

/** @deprecated 经 Modbus 互斥走 USART2 RS485 发送 */
void BSP_USART2_SendBlocking(const uint8_t *data, uint16_t len);

/** @deprecated 转发到 BSP_USART1_RxFlush（USART2 接收环清） */
void BSP_USART2_RxDrain(void);

#endif /* BSP_UART2_H */
