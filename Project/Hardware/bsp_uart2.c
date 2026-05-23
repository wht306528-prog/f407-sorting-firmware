/*
 * bsp_uart2.c — 兼容层：`BSP_USART2_*` 转发到 bsp_uart / modbus_master（硬件 USART2）
 */

#include "bsp_uart2.h"

#include "bsp_uart.h"
#include "modbus_master.h"

#include <stddef.h>

/*
 * 功能：兼容桩：USART2 由 BSP_Uart_HW_Init 初始化；此处不重复配置。
 * 交互：旧代码链防链接失败。
 */
void BSP_USART2_Init(uint32_t baud)
{
	(void)baud;
	/* 真实初始化在 main 中 BSP_Uart_HW_Init(CFG_RS485_BAUD) 已完成 */
}

/*
 * 功能：经 Modbus 互斥在 USART2 RS485 上阻塞发送。
 * 交互：遗留调用。
 */
void BSP_USART2_SendBlocking(const uint8_t *data, uint16_t len)
{
	(void)ModbusMaster_SendRawBlocking(data, len, MB_MUTEX_WAIT_MS);
}

/*
 * 功能：别名 RxFlush，与历史「发前清 RX」行为一致。
 * 交互：遗留调用。
 */
void BSP_USART2_RxDrain(void)
{
	/* 与历史行为一致：发送后清空残留；具体见 bsp_uart.c */
	BSP_RS485_RxFlush();
}
