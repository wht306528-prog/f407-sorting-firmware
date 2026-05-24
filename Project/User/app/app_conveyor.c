/**
 * app_conveyor.c — 三条传送带：继电器/晶体管驱动电机 + 光电停限
 *
 * 使用者：app_sort 中系统标志 FL4 FL5 FL6 各调一次 AppConveyor_RunUntilSensor。
 *
 * ID（与 bsp_conveyor.h 一致）：BSP_CONV_ID_0 对应物理带一与 FL4；BSP_CONV_ID_1 带二 FL5；BSP_CONV_ID_2 带三 FL6。
 *
 * 一次调用：先关另外两条带，再启指定带；每 10ms 查光电；触发则停返回 0；超时返回 1；等待中急停返回 2，上层文案 CNV:ABORT。
 *
 * 重要：本函数内为 while 阻塞 loop，占用 main 循环；不要在中断里调用，否则摸屏与 RS485/矩阵轮询被饿死。
 */
#include "app_conveyor.h"
#include "bsp_conveyor.h"
#include "delay.h"
#include "global_config.h"

#include "app_motion_abort.h"

/*
 * 功能：驱动指定传送带电机直到对应光电触发或超时；等待中可被急停打断。
 * 交互：外部由 app_sort FL4～FL6 调用；封装 BSP_Conveyor/BSP_Photo、AppMotionAbort_PollEscalate；阻塞主循环。
 */
uint8_t AppConveyor_RunUntilSensor(uint8_t conveyor_id, uint32_t timeout_ms)
{
#if CFG_CONVEYOR_SIM_MODE
	/*
	 * 传送带仿真：当前现场还没有确认真实传送带和光电输入。
	 * 这里不打开任何传送带电机，也不等待光电，直接当作“光电已触发”返回成功。
	 * 这样 AppSort 可以继续跑 FL4/FL5/FL6 逻辑，但不会误驱动硬件。
	 */
	(void)timeout_ms;
	if (conveyor_id > BSP_CONV_ID_2)
	{
		return 1u;
	}
	return 0u;
#else
	uint32_t t0;

	if (conveyor_id > BSP_CONV_ID_2)
	{
		return 1u;
	}

	BSP_Conveyor_SetMotor(BSP_CONV_ID_0, 0u);
	BSP_Conveyor_SetMotor(BSP_CONV_ID_1, 0u);
	BSP_Conveyor_SetMotor(BSP_CONV_ID_2, 0u);

	BSP_Conveyor_SetMotor(conveyor_id, 1u);

	t0 = SysTick_GetMs();
	while ((SysTick_GetMs() - t0) < timeout_ms)
	{
		if (AppMotionAbort_PollEscalate())
		{
			BSP_Conveyor_SetMotor(conveyor_id, 0u);
			return 2u;
		}
		if (BSP_Photo_IsTriggered(conveyor_id))
		{
			BSP_Conveyor_SetMotor(conveyor_id, 0u);
			return 0u;
		}
		Delay_ms(10);
	}

	BSP_Conveyor_SetMotor(conveyor_id, 0u);
	return 1u;
#endif
}
