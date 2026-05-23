/*
 * bsp_uart3.c — USART3 PB10/PB11：矩阵侧 Modbus RTU（F407 主站）
 *
 * 物理：常经 TTL↔RS485 接 RK3588 等从站；8N1，波特率 `BSP_Uart3_HW_Init(baud)`，默认见 CFG_MATRIX_MODBUS_BAUD。
 * 诊断：`BSP_USART3_RxOverflowCount` 在 AppMatrixModbusDiag / 串口页展示，用于区分「MCU 收缓冲溢出」与「线无字节」。
 */

#include "bsp_uart3.h"
#include "global_config.h"

#include "delay.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

#include <stddef.h>

#define UART3_PERIPH     USART3
#define UART3_PERIPH_CLK RCC_APB1Periph_USART3

#define UART3_TX_PIN     GPIO_Pin_10
#define UART3_RX_PIN     GPIO_Pin_11
#define UART3_GPIO_PORT  GPIOB
#define UART3_GPIO_CLK   RCC_AHB1Periph_GPIOB
#define UART3_TX_PS      GPIO_PinSource10
#define UART3_RX_PS      GPIO_PinSource11

#define U3_URX_RB_SZ   2048u
#define U3_URX_RB_MASK (U3_URX_RB_SZ - 1u)

static volatile uint8_t  s_u3_urx_rb[U3_URX_RB_SZ];
static volatile uint32_t s_u3_urx_head;
static volatile uint32_t s_u3_urx_tail;

static volatile uint32_t s_u3_rx_overflow_cnt;
static uint32_t          s_u3_uart_baud = 115200u;

#define U3_TRACE_CAP 384u
#define U3_TR_DATA     0u
#define U3_TR_EOF      1u

typedef struct
{
	uint8_t kind;
	uint8_t is_tx;
	uint8_t data;
} u3_tr_ent_t;

static volatile u3_tr_ent_t s_u3_tr[U3_TRACE_CAP];
static volatile uint16_t    s_u3_tr_w;
static volatile uint16_t    s_u3_tr_r;

static volatile uint8_t     s_u3_trace_en;

static void u3_trace_push_evt(uint8_t kind, uint8_t is_tx, uint8_t bb)
{
	uint16_t w;
	uint16_t nw;

	if (s_u3_trace_en == 0u)
	{
		return;
	}
	__disable_irq();
	w = s_u3_tr_w;
	nw = (uint16_t)((w + 1u) % U3_TRACE_CAP);
	if (nw == s_u3_tr_r)
	{
		s_u3_tr_r = (uint16_t)((s_u3_tr_r + 1u) % U3_TRACE_CAP);
	}
	s_u3_tr[w].kind = kind;
	s_u3_tr[w].is_tx = is_tx;
	s_u3_tr[w].data = bb;
	s_u3_tr_w = nw;
	__enable_irq();
}

uint8_t BSP_USART3_TracePopEvt(uint8_t *kind, uint8_t *is_tx, uint8_t *out_b)
{
	uint16_t r;
	uint16_t w;

	if ((kind == NULL) || (is_tx == NULL) || (out_b == NULL))
	{
		return 0u;
	}
	__disable_irq();
	w = s_u3_tr_w;
	r = s_u3_tr_r;
	if (w == r)
	{
		__enable_irq();
		return 0u;
	}
	*kind = s_u3_tr[r].kind;
	*is_tx = s_u3_tr[r].is_tx;
	*out_b = s_u3_tr[r].data;
	s_u3_tr_r = (uint16_t)((r + 1u) % U3_TRACE_CAP);
	__enable_irq();
	return 1u;
}

void BSP_USART3_TracePushRxFrameEnd(void)
{
	u3_trace_push_evt(U3_TR_EOF, 0u, 0u);
}

void BSP_USART3_TraceSetEnabled(uint8_t on)
{
	__disable_irq();
	s_u3_trace_en = (on != 0u) ? 1u : 0u;
	__enable_irq();
}

void BSP_USART3_TraceReset(void)
{
	__disable_irq();
	s_u3_tr_w = 0u;
	s_u3_tr_r = 0u;
	__enable_irq();
}

uint32_t BSP_USART3_RxOverflowCount(void)
{
	uint32_t v;

	__disable_irq();
	v = s_u3_rx_overflow_cnt;
	__enable_irq();
	return v;
}

void BSP_USART3_RxOverflowClear(void)
{
	__disable_irq();
	s_u3_rx_overflow_cnt = 0u;
	__enable_irq();
}

static void Matrix_RS485_BusDelay(__IO uint32_t n)
{
	for (; n != 0u; n--)
	{
	}
}

#if CFG_MATRIX_RS485_USE_DE

static void Matrix_RS485_EnterTx(void)
{
	Matrix_RS485_BusDelay(2000u);
	GPIO_SetBits(CFG_MATRIX_RS485_DE_PORT, CFG_MATRIX_RS485_DE_PIN);
	Matrix_RS485_BusDelay(2000u);
}

static void Matrix_RS485_EnterRx(void)
{
	Matrix_RS485_BusDelay(800u);
	GPIO_ResetBits(CFG_MATRIX_RS485_DE_PORT, CFG_MATRIX_RS485_DE_PIN);
	Matrix_RS485_BusDelay(800u);
}

#else

static void Matrix_RS485_EnterTx(void)
{
}

static void Matrix_RS485_EnterRx(void)
{
}

#endif

static uint32_t Matrix_RS485_TailHoldMs(void)
{
	uint32_t baud = s_u3_uart_baud;

	if (baud == 0u)
	{
		baud = 9600u;
	}
	return (uint32_t)((11000u + baud - 1u) / baud);
}

void BSP_USART3_SendBlocking(const uint8_t *data, uint16_t len)
{
	uint16_t i;

	if ((data == NULL) || (len == 0u))
	{
		return;
	}

	Matrix_RS485_EnterTx();
	USART_ClearFlag(UART3_PERIPH, USART_FLAG_TC);

	for (i = 0u; i < len; i++)
	{
		while (USART_GetFlagStatus(UART3_PERIPH, USART_FLAG_TXE) == RESET)
		{
		}
		USART_SendData(UART3_PERIPH, (uint16_t)data[i]);
		u3_trace_push_evt(U3_TR_DATA, 1u, data[i]);
	}

	while (USART_GetFlagStatus(UART3_PERIPH, USART_FLAG_TC) == RESET)
	{
	}

	u3_trace_push_evt(U3_TR_EOF, 1u, 0u);
	Delay_ms(Matrix_RS485_TailHoldMs());
	Matrix_RS485_EnterRx();
}

void BSP_Uart3_HW_Init(uint32_t baud)
{
	GPIO_InitTypeDef   g;
	USART_InitTypeDef  u;
	NVIC_InitTypeDef   n;

	s_u3_urx_head = 0u;
	s_u3_urx_tail = 0u;
	s_u3_rx_overflow_cnt = 0u;
	s_u3_uart_baud = baud;
	s_u3_tr_w = 0u;
	s_u3_tr_r = 0u;
	s_u3_trace_en = 0u;

	RCC_AHB1PeriphClockCmd(UART3_GPIO_CLK, ENABLE);
#if CFG_MATRIX_RS485_USE_DE
	RCC_AHB1PeriphClockCmd(CFG_MATRIX_RS485_DE_CLK, ENABLE);
#endif
	RCC_APB1PeriphClockCmd(UART3_PERIPH_CLK, ENABLE);

	GPIO_PinAFConfig(UART3_GPIO_PORT, UART3_TX_PS, GPIO_AF_USART3);
	GPIO_PinAFConfig(UART3_GPIO_PORT, UART3_RX_PS, GPIO_AF_USART3);

	g.GPIO_Pin = UART3_TX_PIN | UART3_RX_PIN;
	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART3_GPIO_PORT, &g);

#if CFG_MATRIX_RS485_USE_DE
	g.GPIO_Pin = CFG_MATRIX_RS485_DE_PIN;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CFG_MATRIX_RS485_DE_PORT, &g);
	GPIO_ResetBits(CFG_MATRIX_RS485_DE_PORT, CFG_MATRIX_RS485_DE_PIN);
#endif

	USART_StructInit(&u);
	u.USART_BaudRate = baud;
	u.USART_WordLength = USART_WordLength_8b;
	u.USART_StopBits = USART_StopBits_1;
	u.USART_Parity = USART_Parity_No;
	u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART3_PERIPH, &u);
	USART_ITConfig(UART3_PERIPH, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART3_PERIPH, ENABLE);

	n.NVIC_IRQChannel = USART3_IRQn;
	n.NVIC_IRQChannelPreemptionPriority = 6;
	n.NVIC_IRQChannelSubPriority = 0;
	n.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&n);
}

void BSP_USART3_Rx_IrqService(void)
{
	if (USART_GetITStatus(UART3_PERIPH, USART_IT_RXNE) != RESET)
	{
		uint8_t  bb = (uint8_t)(USART_ReceiveData(UART3_PERIPH) & 0xFFu);
		uint32_t h = s_u3_urx_head;
		uint32_t nx = (h + 1u) & U3_URX_RB_MASK;

		if (nx != s_u3_urx_tail)
		{
			s_u3_urx_rb[h] = bb;
			s_u3_urx_head = nx;
		}
		else
		{
			s_u3_rx_overflow_cnt++;
		}
		u3_trace_push_evt(U3_TR_DATA, 0u, bb);
		USART_ClearITPendingBit(UART3_PERIPH, USART_IT_RXNE);
	}
}

void USART3_IRQHandler(void)
{
	BSP_USART3_Rx_IrqService();
}

uint8_t BSP_USART3_ReadByte(uint8_t *out)
{
	uint32_t h = s_u3_urx_head;
	uint32_t t = s_u3_urx_tail;

	if (h == t)
	{
		return 0u;
	}
	*out = s_u3_urx_rb[t];
	s_u3_urx_tail = (t + 1u) & U3_URX_RB_MASK;
	return 1u;
}

void BSP_USART3_RxFlush(void)
{
	__disable_irq();
	s_u3_urx_tail = s_u3_urx_head;
	__enable_irq();

	while (USART_GetFlagStatus(UART3_PERIPH, USART_FLAG_RXNE) != RESET)
	{
		(void)USART_ReceiveData(UART3_PERIPH);
	}
	if (USART_GetFlagStatus(UART3_PERIPH, USART_FLAG_ORE) != RESET)
	{
		(void)UART3_PERIPH->SR;
		(void)UART3_PERIPH->DR;
	}
}
