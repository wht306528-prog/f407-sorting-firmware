/**
 * app_conveyor.h — 传送带业务：沿指定 conveyor_id 运行直到光电或超时
 *
 * 下层由 bsp_conveyor 驱动电机方向/使能；本函数在长等待中轮询急停（见 AppMotionAbort_PollEscalate）。
 */
#ifndef __APP_CONVEYOR_APP_H__
#define __APP_CONVEYOR_APP_H__

#include <stdint.h>

/** @return 0 光电触发停；1 超时/非法 conveyor_id；2 急停闭锁在长等待中中止 */
uint8_t AppConveyor_RunUntilSensor(uint8_t conveyor_id, uint32_t timeout_ms);

#endif
