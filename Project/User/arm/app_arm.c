/**
 * app_arm.c — 机械臂运动 + 气缸/电磁阀时序（取试管、放试管）
 *
 * 安全：geom_ok 为 0 禁止下发角度；长阻塞里 AppMotionAbort_PollEscalate 可打断。
 * Modbus 返回 MB_MASTER_ERR_ABORT 时 AppMotor 侧会尝试 Disable。
 *
 * 返回码文字：AppArm_ResultText() 与 app_sort 屏显 ARM FLx eNN 配套。
 *
 * 执行节拍：goto src → valve_pick → goto dst → valve_place → AppMatrix_ApplyTransfer；
 * 任一步可被 AppMotionAbort_PollEscalate 拆分。
 */
#include "app_arm.h"

#include "app_calibration_params.h"
#include "app_matrix.h"
#include "app_motor.h"
#include "app_motion_abort.h"

#include "bsp_conveyor.h"
#include "delay.h"
#include "global_config.h"
#include "modbus_master.h"

/*
 * 功能：将 AppArm_PickPlace 等数字返回码映射为短 ASCII 标签（屏显/日志）。
 * 交互：外部被 app_sort snprintf 等调用；无内部状态。
 */
const char *AppArm_ResultText(uint8_t code)
{
	switch (code) {
	case 0u:
		return "OK";
	case 1u:
		return "row_bad";
	case 2u:
		return "src_m1";
	case 3u:
		return "src_m2";
	case 5u:
		return "dst_m1";
	case 6u:
		return "dst_m2";
	case 7u:
		return "src_geom";
	case 8u:
		return "dst_geom";
	case 9u:
		return "xfr_fail";
	case 10u:
		return "dst_3p7";
	case 11u:
		return "dst_3p8";
	case 86u:
		return "pick_stop";
	case 87u:
		return "place_stop";
	case 88u:
		return "m1_abort_s";
	case 89u:
		return "m2_abort_s";
	case 91u:
		return "m1_abort_d";
	case 92u:
		return "m2_abort_d";
	default:
		return "?";
	}
}

/*
 * 功能：可中止的毫秒延时，25ms 切片内轮询急停升级。
 * 交互：内部被 valve_*_sequence 调用；调用 AppMotionAbort_PollEscalate、Delay_ms；返回 1 表示被中断。
 */
static uint8_t Delay_ms_abortable_chunked(uint32_t ms)
{
	uint32_t left = ms;

	while (left > 0u)
	{
		uint32_t step = left;

		if (step > 25u)
		{
			step = 25u;
		}
		if (AppMotionAbort_PollEscalate())
		{
			return 1u;
		}
		Delay_ms(step);
		left -= step;
	}
	return 0u;
}

/*
 * 功能：读矩阵行 θ1/θ2，换算脉冲后经 Modbus 驱动两轴到该位姿。
 * 交互：内部被 AppArm_PickPlace；调用 AppMatrix_GetRow、AppMotor_GotoAbsTargetAsRelative；
 * is_dst_move 区分源/目标段错误码。
 */
static uint8_t move_joints_to_row(uint16_t idx, uint8_t is_dst_move)
{
	MatrixFinalRow_t r;
	uint8_t          e;

	if (!AppMatrix_GetRow(idx, &r))
	{
		return 1u;
	}

	if (r.geom_ok == 0u)
	{
		return is_dst_move ? 8u : 7u;
	}

	e = AppMotor_GotoAbsTargetAsRelative(CFG_MODBUS_SLAVE_MOTOR1,
					     r.pulse_motor1_abs);
	if (e != MB_MASTER_OK)
	{
		if (e == MB_MASTER_ERR_ABORT)
		{
			return 88u;
		}
		return 2u;
	}

	e = AppMotor_GotoAbsTargetAsRelative(CFG_MODBUS_SLAVE_MOTOR2,
					     r.pulse_motor2_abs);
	if (e != MB_MASTER_OK)
	{
		if (e == MB_MASTER_ERR_ABORT)
		{
			return 89u;
		}
		return 3u;
	}

	return 0u;
}

/*
 * 功能：取料阀时序：Z 下降 → 夹爪闭合 → Z 抬升，各步可.abort。
 * 交互：内部被 AppArm_PickPlace；调用 BSP_Valve_*、Delay_ms_abortable_chunked、AppMotionAbort_PollEscalate。
 *
 * 气缸节拍待实测（宏真源在 global_config.h）：
 * - CFG_ARM_PICK_LOWER_MS：气缸下降后、夹持电磁阀输出前的等待；
 * - CFG_ARM_PICK_HOLD_MS：夹爪夹紧输出后的内部保持（随后抬 Z）；
 * - CFG_ARM_PICK_TO_MOVE_MS：整段 pick 结束后、下发 move_joints_to_row(目标) 前的等待（见 PickPlace）。
 */
static uint8_t valve_pick_sequence(void)
{
	if (AppMotionAbort_PollEscalate())
	{
		return 1u;
	}
	BSP_Valve_SetZ(1u);
	/* CFG_ARM_PICK_LOWER_MS：下降到位后再开夹持 */
	if (Delay_ms_abortable_chunked(CFG_ARM_PICK_LOWER_MS) != 0u)
	{
		return 1u;
	}
	BSP_Valve_SetGrip(1u);
	/* CFG_ARM_PICK_HOLD_MS：夹紧电磁阀 ON 后的短时保持 */
	if (Delay_ms_abortable_chunked(CFG_ARM_PICK_HOLD_MS) != 0u)
	{
		return 1u;
	}
	BSP_Valve_SetZ(0u);
	if (Delay_ms_abortable_chunked(CFG_ARM_PICK_RAISE_MS) != 0u)
	{
		return 1u;
	}
	return 0u;
}

/*
 * 功能：放料阀时序：Z 下降 → 夹爪松开 → Z 抬升。
 * 交互：内部被 AppArm_PickPlace；同 pick 的路径与 BSP。
 */
static uint8_t valve_place_sequence(void)
{
	if (AppMotionAbort_PollEscalate())
	{
		return 1u;
	}
	BSP_Valve_SetZ(1u);
	if (Delay_ms_abortable_chunked(CFG_ARM_PLACE_LOWER_MS) != 0u)
	{
		return 1u;
	}
	BSP_Valve_SetGrip(0u);
	if (Delay_ms_abortable_chunked(CFG_ARM_PLACE_RELEASE_MS) != 0u)
	{
		return 1u;
	}
	BSP_Valve_SetZ(0u);
	if (Delay_ms_abortable_chunked(CFG_ARM_PLACE_RAISE_MS) != 0u)
	{
		return 1u;
	}
	return 0u;
}

/*
 * 功能：两轴回到 MCU 标定的零点绝对脉冲（app_calibration_params.h 中 CALIB_JOINT*_ZERO_PULSE）。
 * 交互：外部被 app_sort 44/传送带前、FL45 等调用；走 AppMotor_GotoPulse 绝对移动流水线。
 */
void AppArm_GoHome(void)
{
	(void)AppMotor_GotoPulse(CFG_MODBUS_SLAVE_MOTOR1, (int32_t)CALIB_JOINT1_ZERO_PULSE);
	(void)AppMotor_GotoPulse(CFG_MODBUS_SLAVE_MOTOR2, (int32_t)CALIB_JOINT2_ZERO_PULSE);
}

/*
 * 功能：完整取放：源位姿→夹取→目标位姿→放置→更新矩阵 class 交换。
 * 交互：外部由 app_sort FL2/FL3 调用；内部串联 move_joints_*、valve_*、AppMatrix_ApplyTransfer；可被急停链路打断返回码。
 */
uint8_t AppArm_PickPlace(uint16_t src_idx, uint16_t dst_idx)
{
	uint8_t e;

	e = move_joints_to_row(src_idx, 0u);
	if (e != 0u)
	{
		return e;
	}

	if (valve_pick_sequence() != 0u)
	{
		return 86u;
	}

	/* CFG_ARM_PICK_TO_MOVE_MS：夹持完成且抬 Z 后，再发下一关节绝对移动的延时（现场可调） */
	if (Delay_ms_abortable_chunked(CFG_ARM_PICK_TO_MOVE_MS) != 0u)
	{
		return 86u;
	}

	e = move_joints_to_row(dst_idx, 1u);
	if (e != 0u)
	{
		/* 目标段失败：夹爪仍夹着料；急停/电机类错误码 +3 形成目标段族 */
		if (e == 88u || e == 89u)
		{
			return (uint8_t)(e + 3u);
		}
		return (uint8_t)(3u + e);
	}

	if (valve_place_sequence() != 0u)
	{
		return 87u;
	}

	if (!AppMatrix_ApplyTransfer(src_idx, dst_idx))
	{
		return 9u;
	}

	return 0u;
}
