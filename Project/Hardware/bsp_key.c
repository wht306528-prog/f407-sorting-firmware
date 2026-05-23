#include "bsp_key.h"

/*
 * bsp_key.c — 板载按键 GPIO（输入 + 内部上拉）
 *
 * KEY1/KEY2 在硬件上是“按下=接地、松开=被上拉拉高”，与 `bsp_exti.c` 的下降沿触发一致。
 * 本文件的 `BSP_Key_Init` 负责把 PA0、PC13 配成上拉输入；外部中断连线在别处初始化。
 *
 * `BSP_Key_Scan` 是轮询去抖的一种写法：部分老代码可能还在用；KEY1/KEY2 主路径走 EXTI + main。
 */

/*
 * 功能：配置 KEY1/KEY2 为上拉输入（中断挂接见 bsp_exti）。
 * 交互：main 初始化早于 EXTI。
 */
void BSP_Key_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(KEY1_GPIO_CLK | KEY2_GPIO_CLK, ENABLE);

	gpio.GPIO_Pin = KEY1_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	/* 独立按键常接 GND：释放为高需上拉，按下为低 */
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(KEY1_GPIO_PORT, &gpio);

	gpio.GPIO_Pin = KEY2_PIN;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(KEY2_GPIO_PORT, &gpio);
}

/*
 * 功能：简易阻塞读取一次按键闭合（按住直到松开）。
 * 交互：老式轮询可选；主干用 EXTI。
 */
uint8_t BSP_Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	if (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == KEY_ON)
	{
		/* 简易阻塞去抖：等到松开才返回，避免一次长按触发多次业务 */
		while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == KEY_ON)
		{
		}
		return KEY_ON;
	}
	return KEY_OFF;
}
