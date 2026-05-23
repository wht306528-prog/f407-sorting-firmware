/**
 * sys.c — NVIC 优先级分组配置（抢占 2bit + 子优先级 2bit）
 *
 * 以太网 / USART / EXTI 的相对优先级由此决定；具体向量服务程序见 stm32f4xx_it.c。
 */
#include "stm32f4xx.h"
#include "misc.h"
#include "sys.h"

/*
 * NVIC：抢占=2bit + 子优先级=2bit。
 * 数值越小优先级越高；USART1/Touch EXTI 与以太网 DMA 中断的相对顺序由此分组决定。
 */

/*
 * 功能：配置 NVIC 优先级分组（抢占 2bit + 子优先级 2bit）。
 * 交互：main 早于各外设中断使能前调用。
 */
void SYS_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
