/**
 * bsp_uart3.h — USART3（PB10 TX / PB11 RX）用于鲁班猫矩阵 Modbus RTU
 *
 * 与 USART2 电机 RS485 主站总线物理隔离；提供阻塞发送、RX 环形缓冲、可选 RS485 DE、HEX trace。
 */
#ifndef BSP_UART3_H
#define BSP_UART3_H

#include <stdint.h>

void BSP_Uart3_HW_Init(uint32_t baud);

void BSP_USART3_SendBlocking(const uint8_t *data, uint16_t len);

uint8_t BSP_USART3_ReadByte(uint8_t *out);

void BSP_USART3_RxFlush(void);

uint32_t BSP_USART3_RxOverflowCount(void);

void BSP_USART3_RxOverflowClear(void);

/**
 * trace：`kind` 0=DATA 1=FRAME_END；与 bsp_uart / app_display HEX 滚动一致。
 */
uint8_t BSP_USART3_TracePopEvt(uint8_t *kind, uint8_t *is_tx, uint8_t *out_b);

void BSP_USART3_TracePushRxFrameEnd(void);

/** KEY2 矩阵 Modbus 会话内才写 trace；默认关，避免空闲总线毛刺刷屏 */
void BSP_USART3_TraceSetEnabled(uint8_t on);

/** 清空 trace 环（KEY2 本轮开始前调用） */
void BSP_USART3_TraceReset(void);

void BSP_USART3_Rx_IrqService(void);

#endif
