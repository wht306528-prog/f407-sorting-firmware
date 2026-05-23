/**
 * app_motion_abort.h — 阻塞运动中的「急停/按键」升格为分拣 ESTOP
 *
 * 由 app_motor、app_arm 阀延时、app_conveyor 等长等待循环调用，避免卡死在 while。
 */
#ifndef APP_MOTION_ABORT_H__
#define APP_MOTION_ABORT_H__

#include <stdint.h>

/**
 * 在运动阻塞路径（伺服等待、阀延时、传送带轮询）中间隔调用。
 * 若已急停闭锁（AppSort_IsEstopLatched）：返回 1，否则 0。
 */
uint8_t AppMotionAbort_PollEscalate(void);

#endif
