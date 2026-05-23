/**
 * bsp_buzzer.h — 蜂鸣器 GPIO 开关与定时 beep（实现里可能用阻塞延时）
 */
#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

void BSP_Buzzer_Init(void);
void BSP_Buzzer_On(void);
void BSP_Buzzer_Off(void);
void BSP_Buzzer_BeepMs(uint32_t ms);

#endif /* BSP_BUZZER_H */
