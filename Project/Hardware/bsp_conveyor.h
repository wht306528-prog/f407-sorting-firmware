/**
 * =============================================================================
 * bsp_conveyor.h — 传送带电磁阀光电 GPIO
 * =============================================================================
 */
#ifndef __BSP_CONVEYOR_H__
#define __BSP_CONVEYOR_H__

#include "stm32f4xx.h"
#include <stdint.h>

#define BSP_CONV_ID_0   0u
#define BSP_CONV_ID_1   1u
#define BSP_CONV_ID_2   2u

void BSP_Conveyor_Init(void);

void BSP_Valve_SetZ(uint8_t on);
void BSP_Valve_SetGrip(uint8_t on);

void BSP_Conveyor_SetMotor(uint8_t id, uint8_t on);

uint8_t BSP_Photo_IsTriggered(uint8_t id); /* id 同传送带索引 */

/**
 * @brief 故障 / 紧急停止时兜底：三路传送带熄灭、气缸/阀门回到安全。
 * DE/~RE **不在本函数**：仅 GPIO 工况侧执行器。
 */
void BSP_Actuators_AllSafe(void);

#endif /* __BSP_CONVEYOR_H__ */
