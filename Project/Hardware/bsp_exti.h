/**
 * bsp_exti.h — KEY1/KEY2 外部中断初始化与 volatile 上升沿标志
 *
 * ISR 在 stm32f4xx_it.c；业务层用 EXTI_PopKey1/PopKey2 消费，避免在中断里跑 Modbus。
 */
#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include <stdint.h>

void BSP_EXTI_Keys_Init(void);

extern volatile uint8_t g_key1_rising;
extern volatile uint8_t g_key2_rising;

#endif /* BSP_EXTI_H */
