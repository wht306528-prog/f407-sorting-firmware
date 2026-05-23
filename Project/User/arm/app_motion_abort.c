/**
 * app_motion_abort.c — 运动是否立刻刹停的快判
 *
 * 职责：把急停已锁存整理成 0/1，供 app_arm、app_motor、传送带轮询。
 * 不做 Modbus，不做 PWM，只读 AppSort 急停标志。
 */
#include "app_motion_abort.h"

#include "app_sort.h"

/*
 * 功能：运动中轮询是否需要立即刹停（急停已锁存）。
 * 交互：外部由 app_arm 延时切片、app_motor 等待环、传送带阻塞环调用。
 */
uint8_t AppMotionAbort_PollEscalate(void)
{
	if (AppSort_IsEstopLatched() != 0u) {
		return 1u;
	}
	return 0u;
}
