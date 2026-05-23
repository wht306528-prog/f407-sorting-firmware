#include "bsp_buzzer.h"
#include "stm32f4xx.h"
#include "delay.h"

/**
 * bsp_buzzer.c — PG7 蜂鸣器 GPIO；BeepMs 使用 delay 阻塞
 */

#define BUZZER_PIN        GPIO_Pin_7
#define BUZZER_GPIO       GPIOG
#define BUZZER_GPIO_CLK   RCC_AHB1Periph_GPIOG

/*
 * 功能：PG7 蜂鸣器 GPIO 输出初始化并静音。
 * 交互：main 初始化。
 */
void BSP_Buzzer_Init(void)
{
	GPIO_InitTypeDef g;

	RCC_AHB1PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);

	g.GPIO_Pin = BUZZER_PIN;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_DOWN;
	g.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(BUZZER_GPIO, &g);
	BSP_Buzzer_Off();
}

/*
 * 功能：蜂鸣器开（PG7 高有效按硬件）。
 * 交互：故障提示短时鸣叫。
 */
void BSP_Buzzer_On(void)
{
	GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
}

/*
 * 功能：蜂鸣器关。
 * 交互：与 On 配对。
 */
void BSP_Buzzer_Off(void)
{
	GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
}

/*
 * 功能：阻塞鸣叫给定毫秒（Delay_ms）。
 * 交互：告警简单提示；慎用长 ms。
 */
void BSP_Buzzer_BeepMs(uint32_t ms)
{
	BSP_Buzzer_On();
	Delay_ms(ms);
	BSP_Buzzer_Off();
}
