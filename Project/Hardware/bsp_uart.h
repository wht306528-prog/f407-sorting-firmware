/**
 * bsp_uart.h — 统一 RS485：USART2 PA2/PA3 + PC0 方向（USART2 为 APB1 外设，见 RM0090）
 *
 * - 物理层：8N1，波特率由 BSP_Uart_HW_Init(baud) 设置，须与总线上所有从机一致。
 * - 命名：保留 BSP_USART1_* 前缀以兼容既有 Modbus Master 代码（实现为 USART2）；
 *       新增 BSP_RS485_* 语义别名，表示「本条总线」，便于阅读。
 * - 链路：USART2_IRQHandler → BSP_USART1_Rx_IrqService 环形缓冲 → modbus_master master_recv_adu。
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx.h"
#include <stdint.h>

#define BSP_USART1_TX_ENABLE 1u
#define BSP_USART1_RX_ENABLE 1u

void BSP_Uart_HW_Init(uint32_t baud);

void BSP_USART1_SendBlocking(const uint8_t *data, uint16_t len);

#define BSP_USART1_SendBytes(data, len) BSP_USART1_SendBlocking((data), (len))

void BSP_USART1_Rx_IrqService(void);

uint8_t BSP_USART1_ReadByte(uint8_t *out);

void BSP_USART1_RxFlush(void);

uint32_t BSP_Irq_USART1_RxCount(void);

/* ---------- RS485 语义别名（同上 USART2 硬件）---------- */
void BSP_RS485_SendBlocking(const uint8_t *data, uint16_t len);

#define BSP_RS485_SendBytes(data, len) BSP_RS485_SendBlocking((data), (len))

uint8_t BSP_RS485_ReadByte(uint8_t *out);

void BSP_RS485_RxFlush(void);

/**
 * @brief 取走一条软硬件 trace（主循环调用）。
 * @param[out] is_tx 非零为 MCU 发出的 TX 字节，否则为 RX。
 * @param[out] out_b 原始字节。
 * @return 0 空；非 0 成功弹出一字节。
 *
 * trace 的来源：BSP_USART1_SendBlocking 发送时逐字节记入；USART2 Rx 中断里每收到一字节记入。
 */
uint8_t BSP_RS485_TracePop(uint8_t *is_tx, uint8_t *out_b);

/** Pop trace event：`kind` 0=DATA 1=FRAME_END（帧结束，用于 LCD 分行） */
uint8_t BSP_RS485_TracePopEvt(uint8_t *kind, uint8_t *is_tx, uint8_t *out_b);

/** Modbus 收到完整 RX ADU 后调用，写入 RX 帧结束标记 */
void BSP_RS485_TracePushRxFrameEnd(void);

uint32_t BSP_USART1_RxOverflowCount(void);
void BSP_USART1_RxOverflowClear(void);

#endif /* BSP_UART_H */
