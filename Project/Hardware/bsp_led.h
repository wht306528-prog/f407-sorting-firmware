/**
 * bsp_led.h — 板载 RGB + LED4 的 GPIO 宏封装（低电平点亮，见 LED_ACTIVE_LEVEL）
 */
#ifndef BSP_LED_H
#define BSP_LED_H

#include "stm32f4xx.h"

#define LED_R_PIN                  GPIO_Pin_6
#define LED_R_GPIO_PORT            GPIOF
#define LED_R_GPIO_CLK             RCC_AHB1Periph_GPIOF

#define LED_G_PIN                  GPIO_Pin_7
#define LED_G_GPIO_PORT            GPIOF
#define LED_G_GPIO_CLK             RCC_AHB1Periph_GPIOF

#define LED_B_PIN                  GPIO_Pin_8
#define LED_B_GPIO_PORT            GPIOF
#define LED_B_GPIO_CLK             RCC_AHB1Periph_GPIOF

#define LED4_PIN                   GPIO_Pin_3
#define LED4_GPIO_PORT             GPIOC
#define LED4_GPIO_CLK              RCC_AHB1Periph_GPIOC

#define LED_ACTIVE_LEVEL           0
#define LED_INACTIVE_LEVEL         1

#define digitalHi(p, i)            ((p)->BSRRL = (i))
#define digitalLo(p, i)            ((p)->BSRRH = (i))
#define digitalToggle(p, i)        ((p)->ODR ^= (i))

#define LED_R_ON                   digitalLo(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_OFF                  digitalHi(LED_R_GPIO_PORT, LED_R_PIN)
#define LED_R_TOGGLE               digitalToggle(LED_R_GPIO_PORT, LED_R_PIN)

#define LED_G_ON                   digitalLo(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_OFF                  digitalHi(LED_G_GPIO_PORT, LED_G_PIN)
#define LED_G_TOGGLE               digitalToggle(LED_G_GPIO_PORT, LED_G_PIN)

#define LED_B_ON                   digitalLo(LED_B_GPIO_PORT, LED_B_PIN)
#define LED_B_OFF                  digitalHi(LED_B_GPIO_PORT, LED_B_PIN)
#define LED_B_TOGGLE               digitalToggle(LED_B_GPIO_PORT, LED_B_PIN)

#define LED4_ON                    digitalLo(LED4_GPIO_PORT, LED4_PIN)
#define LED4_OFF                   digitalHi(LED4_GPIO_PORT, LED4_PIN)
#define LED4_TOGGLE                digitalToggle(LED4_GPIO_PORT, LED4_PIN)

#define LED_RGB_ALL_OFF            do { LED_R_OFF; LED_G_OFF; LED_B_OFF; } while (0)

void BSP_LED_Init(void);

#endif /* BSP_LED_H */
