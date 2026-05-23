/**
 * delay.h — SysTick 毫秒计数与阻塞延时声明（实现见 delay.c；SysTick_Handler 调 Delay_Inc）
 */
#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

void Delay_Init(void);
void Delay_Inc(void);
uint32_t Delay_GetTick(void);
void Delay_ms(uint32_t ms);

uint32_t SysTick_GetMs(void);

#endif /* DELAY_H */
