/**
 * bsp_key.h — KEY1/KEY2 引脚与软件轮询扫描 API；边沿检测另见 bsp_exti / NVIC EXTI
 */
#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "stm32f4xx.h"

#define KEY1_PIN                  GPIO_Pin_0
#define KEY1_GPIO_PORT            GPIOA
#define KEY1_GPIO_CLK             RCC_AHB1Periph_GPIOA

#define KEY2_PIN                  GPIO_Pin_13
#define KEY2_GPIO_PORT            GPIOC
#define KEY2_GPIO_CLK             RCC_AHB1Periph_GPIOC

#define KEY_ON                    1
#define KEY_OFF                   0

void BSP_Key_Init(void);
uint8_t BSP_Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* BSP_KEY_H */
