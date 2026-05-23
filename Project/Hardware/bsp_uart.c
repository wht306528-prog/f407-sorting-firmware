/*
 * bsp_uart.c — USART2（PA2/PA3）+ RS485 收发方向（DE）+ 接收中断环形缓冲
 *
 * 【零基础：485 为什么要有 DE】
 * RS485 半双工：同一对线上**同一时间只能有一方讲话**。MCU TX 接到收发器 DI，**PC0=DE** 拉高时
 * 才把 MCU 的发送推到 A/B 总线；拉低时听别人说话。若 DE 始终为低，MCU“发了字节”外部总线也看不到。
 *
 * 【引脚（电机/灯 RS485 = KEY1 所在总线）】
 * - PA2 = USART2_TX，PA3 = USART2_RX（接到 MAX485 类芯片的 DI / RO）
 * - PC0 = DE/~RE（高=发送，低=接收）
 *
 * 【Trace 是干什么的】
 * 每收到/发送一字节记一条 DATA；一帧结束记 FRAME_END。`app_display` 把这些拼成屏上 TX:/RX: 的 HEX 行，
 * 老奶奶不用示波器也能看到“板子刚才有没有讲话”。
 *
 * 【外部 USB/串口助手为何常看到 0x00】
 * 须接在同一 RS485 总线（A/B）且 8N1 波特一致；用 TTL 怼 PA3、或总线空闲/收发器 RO 在发送期的电平，
 * 都可能让 PC 侧长期像「只收到 0x00」。以 MCU trace 的 TX: 行或示波看差分为准，不等同于未发出的帧。
 * DE=1 发送时部分 MAX485 电路 RO 不稳，USART 仍可能进 RX 中断：本模块在发送窗口内丢弃此类入环与 trace，
 * 避免屏上出现误导性的 RX: 00。
 *
 * 【接收】RXNE 中断把字节推进环形缓冲；应用用 `ReadByte` 取。若取太慢会丢字节并 `overflow` 计数。
 */

#include "bsp_uart.h"
#include "delay.h"
#include "misc.h"
#include <stddef.h>

#define UART_PERIPH     USART2
#define UART_PERIPH_CLK RCC_APB1Periph_USART2

#define UART_TX_PIN     GPIO_Pin_2
#define UART_RX_PIN     GPIO_Pin_3
#define UART_GPIO_PORT  GPIOA
#define UART_GPIO_CLK   RCC_AHB1Periph_GPIOA
#define UART_TX_PS      GPIO_PinSource2
#define UART_RX_PS      GPIO_PinSource3

#define RS485_DE_PORT  GPIOC
#define RS485_DE_PIN   GPIO_Pin_0
#define RS485_DE_CLK   RCC_AHB1Periph_GPIOC

#define URX_RB_SZ   2048u
#define URX_RB_MASK (URX_RB_SZ - 1u)

static volatile uint8_t  s_urx_rb[URX_RB_SZ];
static volatile uint32_t s_urx_head;
static volatile uint32_t s_urx_tail;

static volatile uint32_t s_irq_rx_cnt;
static volatile uint32_t s_rx_overflow_cnt;
static uint32_t          s_uart_baud = 9600u;
/** DE=高、本机正在占用总线发送：RX 字节视为回声/毛刺，不入环、不记 trace（仍读 DR 清标志） */
static volatile uint8_t  s_rs485_tx_active;

#define BS_TRACE_CAP 384u
#define BS_TR_DATA   0u
#define BS_TR_EOF    1u

typedef struct
{
	uint8_t kind; /* BS_TR_DATA / BS_TR_EOF */
	uint8_t is_tx;
	uint8_t data;
} bs_tr_ent_t;

static volatile bs_tr_ent_t s_tr[BS_TRACE_CAP];
static volatile uint16_t    s_tr_w;
static volatile uint16_t    s_tr_r;

/*
 * 功能：向 RS485 跟踪环形缓冲写入一字节或帧结束事件（中断/发送路径均可能调）。
 * 交互：内部 ISR 与 SendBlocking；读侧 app_display。
 */
static void bs_trace_push_evt(uint8_t kind, uint8_t is_tx, uint8_t bb)
{
	uint16_t w;
	uint16_t nw;

	__disable_irq();
	w = s_tr_w;
	nw = (uint16_t)((w + 1u) % BS_TRACE_CAP);
	if (nw == s_tr_r)
	{
		/* 环形缓冲满：丢弃最旧一条，避免阻塞中断上下文 */
		s_tr_r = (uint16_t)((s_tr_r + 1u) % BS_TRACE_CAP);
	}
	s_tr[w].kind = kind;
	s_tr[w].is_tx = is_tx;
	s_tr[w].data = bb;
	s_tr_w = nw;
	__enable_irq();
}

/*
 * 功能：弹出一条完整跟踪事件（含 FRAME_END）。
 * 交互：低级调试；BSP_RS485_TracePop 包装。
 */
uint8_t BSP_RS485_TracePopEvt(uint8_t *kind, uint8_t *is_tx, uint8_t *out_b)
{
	uint16_t r;
	uint16_t w;

	if ((kind == NULL) || (is_tx == NULL) || (out_b == NULL))
	{
		return 0u;
	}
	__disable_irq();
	w = s_tr_w;
	r = s_tr_r;
	if (w == r)
	{
		/* 读指针追上写指针：空队列 */
		__enable_irq();
		return 0u;
	}
	*kind = s_tr[r].kind;
	*is_tx = s_tr[r].is_tx;
	*out_b = s_tr[r].data;
	s_tr_r = (uint16_t)((r + 1u) % BS_TRACE_CAP);
	__enable_irq();
	return 1u;
}

/*
 * 功能：兼容接口：跳过 EOF，只吐出 DATA 字节。
 * 交互：旧屏显路径。
 */
uint8_t BSP_RS485_TracePop(uint8_t *is_tx, uint8_t *out_b)
{
	uint8_t k;

	while (BSP_RS485_TracePopEvt(&k, is_tx, out_b) != 0u)
	{
		if (k == BS_TR_DATA)
		{
			/* 兼容旧接口：只吐出 DATA，跳过 EOF 分隔符 */
			return 1u;
		}
	}
	return 0u;
}

/*
 * 功能：Modbus 收齐一帧时由主机层打点 EOF，便于 HEX 分行。
 * 交互：modbus_master.master_recv_adu。
 */
void BSP_RS485_TracePushRxFrameEnd(void)
{
	bs_trace_push_evt(BS_TR_EOF, 0u, 0u);
}

/*
 * 功能：返回 RX 环形缓冲溢出丢弃次数。
 * 交互：诊断。
 */
uint32_t BSP_USART1_RxOverflowCount(void)
{
	uint32_t v;

	__disable_irq();
	v = s_rx_overflow_cnt;
	__enable_irq();
	return v;
}

/*
 * 功能：清零 RX 溢出计数。
 * 交互：初始化或调试重置。
 */
void BSP_USART1_RxOverflowClear(void)
{
	__disable_irq();
	s_rx_overflow_cnt = 0u;
	__enable_irq();
}

/*
 * 功能：忙等_nop 粗延时用于 DE 切换毛刺空隙。
 * 交互：RS485_EnterTx/Rx 内部。
 */
static void RS485_BusDelay(__IO uint32_t n)
{
	for (; n != 0u; n--)
	{
	}
}

/*
 * 功能：拉高 DE（RS485 进入发送）。
 * 交互：SendBlocking 发前调用。
 */
static void RS485_EnterTx(void)
{
	s_rs485_tx_active = 1u;
	/* DE 拉高前先插空操作：等待收发器完成方向翻转，避免首字节毛刺 */
	RS485_BusDelay(2000u);
	GPIO_SetBits(RS485_DE_PORT, RS485_DE_PIN);
	RS485_BusDelay(2000u);
}

/*
 * 功能：拉低 DE（RS485 进入接收）。
 * 交互：SendBlocking 尾 Hold 后调用。
 */
static void RS485_EnterRx(void)
{
	RS485_BusDelay(800u);
	/* ~RE/DE 拉低：芯片回到接收态，A/B 总线交由外部驱动 */
	GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);
	RS485_BusDelay(800u);
	s_rs485_tx_active = 0u;
}

/*
 * 功能：估算发送末尾再保持 ~1 字符时间的毫秒数。
 * 交互：SendBlocking TailHold Delay_ms。
 */
static uint32_t RS485_TailHoldMs(void)
{
	uint32_t baud = s_uart_baud;

	if (baud == 0u)
	{
		baud = 9600u;
	}
	/* 发送完成后额外 Hold ~1 字符时间，确保最后一位彻底离开线缆再切接收 */
	return (uint32_t)((11000u + baud - 1u) / baud);
}

/*
 * 功能：阻塞发送一串字节：切 TX→写 USART→跟踪→TailHold→切 RX。
 * 交互：modbus_master BSP_USART1_SendBlocking（硬件 USART2）；占满主线程。
 */
void BSP_USART1_SendBlocking(const uint8_t *data, uint16_t len)
{
	uint16_t i;

	if ((data == NULL) || (len == 0u))
	{
		return;
	}

	RS485_EnterTx();

	USART_ClearFlag(UART_PERIPH, USART_FLAG_TC);

	for (i = 0u; i < len; i++)
	{
		while (USART_GetFlagStatus(UART_PERIPH, USART_FLAG_TXE) == RESET)
		{
			/* 发送寄存器满则 spin：Blocking 语义，避免破坏帧顺序 */
		}
		USART_SendData(UART_PERIPH, (uint16_t)data[i]);
		bs_trace_push_evt(BS_TR_DATA, 1u, data[i]);
	}

	while (USART_GetFlagStatus(UART_PERIPH, USART_FLAG_TC) == RESET)
	{
		/* 等到移位寄存器也空，才算最后一个 STOP bit 真正出门 */
	}

	bs_trace_push_evt(BS_TR_EOF, 1u, 0u);

	Delay_ms(RS485_TailHoldMs());
	RS485_EnterRx();
}

/*
 * 功能：USART2 PA2/PA3 + DE GPIO + RXNE 中断 + NVIC；记录波特率用于 TailHold。
 * 交互：main BSP_Uart_HW_Init。
 */
void BSP_Uart_HW_Init(uint32_t baud)
{
	GPIO_InitTypeDef g;
	USART_InitTypeDef u;
	NVIC_InitTypeDef n;

	s_urx_head = 0u;
	s_urx_tail = 0u;
	s_irq_rx_cnt = 0u;
	s_rx_overflow_cnt = 0u;
	s_uart_baud = baud;
	s_tr_w = 0u;
	s_tr_r = 0u;
	s_rs485_tx_active = 0u;

	RCC_AHB1PeriphClockCmd(UART_GPIO_CLK | RS485_DE_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(UART_PERIPH_CLK, ENABLE);

	GPIO_PinAFConfig(UART_GPIO_PORT, UART_RX_PS, GPIO_AF_USART2);
	GPIO_PinAFConfig(UART_GPIO_PORT, UART_TX_PS, GPIO_AF_USART2);

	g.GPIO_Pin = UART_TX_PIN | UART_RX_PIN;
	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(UART_GPIO_PORT, &g);

	g.GPIO_Pin = RS485_DE_PIN;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(RS485_DE_PORT, &g);
	GPIO_ResetBits(RS485_DE_PORT, RS485_DE_PIN);

	USART_StructInit(&u);
	u.USART_BaudRate = baud;
	u.USART_WordLength = USART_WordLength_8b;
	u.USART_StopBits = USART_StopBits_1;
	u.USART_Parity = USART_Parity_No;
	u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART_PERIPH, &u);
	USART_ITConfig(UART_PERIPH, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART_PERIPH, ENABLE);

	n.NVIC_IRQChannel = USART2_IRQn;
	n.NVIC_IRQChannelPreemptionPriority = 7;
	n.NVIC_IRQChannelSubPriority = 0;
	n.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&n);
}

/*
 * 功能：RXNE ISR 核心：弹字节入环、超限计数、打点 trace。
 * 交互：USART2_IRQHandler 调用。
 */
void BSP_USART1_Rx_IrqService(void)
{
	if (USART_GetITStatus(UART_PERIPH, USART_IT_RXNE) != RESET)
	{
		uint8_t  bb = (uint8_t)(USART_ReceiveData(UART_PERIPH) & 0xFFu);
		uint32_t h;
		uint32_t nx;

		if (s_rs485_tx_active != 0u)
		{
			USART_ClearITPendingBit(UART_PERIPH, USART_IT_RXNE);
			return;
		}

		h = s_urx_head;
		nx = (h + 1u) & URX_RB_MASK;

		if (nx != s_urx_tail)
		{
			s_urx_rb[h] = bb;
			s_urx_head = nx;
		}
		else
		{
			/* head 追上 tail：应用未及时 ReadByte，硬件字节只能丢弃并计数 */
			s_rx_overflow_cnt++;
		}
		s_irq_rx_cnt++;
		bs_trace_push_evt(BS_TR_DATA, 0u, bb);
		USART_ClearITPendingBit(UART_PERIPH, USART_IT_RXNE);
	}
}

/*
 * 功能：应用软件从环形缓冲取一字节。
 * 交互：modbus_master 轮询收。
 */
uint8_t BSP_USART1_ReadByte(uint8_t *out)
{
	uint32_t h = s_urx_head;
	uint32_t t = s_urx_tail;

	if (h == t)
	{
		/* 空环形缓冲 */
		return 0u;
	}
	*out = s_urx_rb[t];
	s_urx_tail = (t + 1u) & URX_RB_MASK;
	return 1u;
}

/*
 * 功能：对齐环读写指针并读空硬件 FIFO（新事务前清残留）。
 * 交互：modbus txn 发送前 RxFlush。
 */
void BSP_USART1_RxFlush(void)
{
	__disable_irq();
	s_urx_tail = s_urx_head;
	__enable_irq();

	while (USART_GetFlagStatus(UART_PERIPH, USART_FLAG_RXNE) != RESET)
	{
		(void)USART_ReceiveData(UART_PERIPH);
	}
	if (USART_GetFlagStatus(UART_PERIPH, USART_FLAG_ORE) != RESET)
	{
		/*  Overrun：读 SR 再读 DR 序列清标志（参考 RM0090） */
		(void)UART_PERIPH->SR;
		(void)UART_PERIPH->DR;
	}
}

/*
 * 功能：ISR 已累计接收字节总数（调试）。
 * 交互：可选统计。
 */
uint32_t BSP_Irq_USART1_RxCount(void)
{
	return s_irq_rx_cnt;
}

/*
 * 功能：USART2 向量入口；API 仍名 BSP_USART1_* 以兼容 modbus_master。
 * 交互：内核中断表。
 */
void USART2_IRQHandler(void)
{
	BSP_USART1_Rx_IrqService();
}

/*
 * 功能：别名：RS485 SendBlocking。
 * 交互：对外 API 语义。
 */
void BSP_RS485_SendBlocking(const uint8_t *data, uint16_t len)
{
	BSP_USART1_SendBlocking(data, len);
}

/*
 * 功能：别名：ReadByte。
 * 交互：对外封装。
 */
uint8_t BSP_RS485_ReadByte(uint8_t *out)
{
	return BSP_USART1_ReadByte(out);
}

/*
 * 功能：别名：电机口（BSP_USART1_*）RxFlush。
 * 交互：对外封装。
 */
void BSP_RS485_RxFlush(void)
{
	BSP_USART1_RxFlush();
}
