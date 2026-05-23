#include "bsp_led.h"

/**
 * bsp_led.c — RGB + LED4 GPIO 初始化与默认熄灭状态
 *
 * 模式：推挽输出；熄灭电平由 LED_ACTIVE_LEVEL / INACTIVE 宏定义。
 */

/* RGB + LED4：推挽输出；默认全部熄灭（共阳/共阴宏定义见头文件） */

/*
 * 功能：初始化板载 RGB 与 LED4 GPIO 并置默认熄灭。
 * 交互：外部 main 初始化；寄存器时钟 GPIO_Init。
 */
void BSP_LED_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_AHB1PeriphClockCmd(LED_R_GPIO_CLK | LED_G_GPIO_CLK | LED_B_GPIO_CLK | LED4_GPIO_CLK, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_25MHz;

	gpio.GPIO_Pin = LED_R_PIN;
	GPIO_Init(LED_R_GPIO_PORT, &gpio);

	gpio.GPIO_Pin = LED_G_PIN;
	GPIO_Init(LED_G_GPIO_PORT, &gpio);

	gpio.GPIO_Pin = LED_B_PIN;
	GPIO_Init(LED_B_GPIO_PORT, &gpio);

	gpio.GPIO_Pin = LED4_PIN;
	GPIO_Init(LED4_GPIO_PORT, &gpio);

	LED_RGB_ALL_OFF;
	LED4_OFF;
}
