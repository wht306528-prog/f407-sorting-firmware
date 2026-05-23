/*
 * bsp_exti.c — 把 KEY1/KEY2 引脚挂到 EXTI 外部中断线
 *
 * KEY1 使用 PA0 → EXTI_Line0；KEY2 使用 PC13 → EXTI_Line13。
 * 触发方式选**下降沿**：因为我们的按键电路是“平时上拉、按下接地”，按下瞬间电平从高变低。
 *
 * 注意：**真正业务（回零、发 485 演示帧等）不在这里做**，只在中断服务程序里记一个标志（见 stm32f4xx_it.c），
 * main 循环里再弹出来处理——否则中断里阻塞发串口会把系统拖死。
 */
#include "bsp_exti.h"
#include "stm32f4xx.h"
#include "misc.h"

volatile uint8_t g_key1_rising;
volatile uint8_t g_key2_rising;

/*
 * 功能：把 KEY1/KEY2 映射到 EXTI0/EXTI13，下降沿中断， NVIC 优先级组内子优先级错位。
 * 交互：ISR 位于 stm32f4xx_it；main 在 BSP_Key_Init 后调用。
 */
void BSP_EXTI_Keys_Init(void)
{
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nv;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

	exti.EXTI_Line = EXTI_Line0;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 按下接地：下降沿触发；上升沿会在松手触发，不符合“按下即发” */
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);

	exti.EXTI_Line = EXTI_Line13;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&exti);

	nv.NVIC_IRQChannel = EXTI0_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 6;
	nv.NVIC_IRQChannelSubPriority = 0;
	nv.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nv);

	nv.NVIC_IRQChannel = EXTI15_10_IRQn;
	nv.NVIC_IRQChannelPreemptionPriority = 6;
	nv.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nv);
}
