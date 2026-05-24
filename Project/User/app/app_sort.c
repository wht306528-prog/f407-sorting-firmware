/**
 * app_sort.c — 主编排：触摸屏大状态 + 内层步进 + 系统运行标志机
 *
 * 给零基础读者：三件事叠在一起
 * 1) 大状态 s_main：IDLE / RUN / PAUSE / FAULT / ESTOP（启停可从上位机或其它路径 RequestStart/Pause/Clear）。
 * 2) 内层步进 s_step：仅 RUN 有意义。先 APP_SORT_STEP_WAIT_MATRIX（等 KEY2 Modbus 矩阵提交 + 几何/IK 就绪），再 SORTING（运行 sorting_tick）。
 * 3) 系统运行标志 s_sys_run_flag：FL2、FL3 等，决定本轮调用 AppArm_PickPlace、AppConveyor_RunUntilSensor 等。
 *
 * AppSort_Poll 开头先做 AppMatrix_FlushPendingGeometry：矩阵逆解费 CPU，故统一在主循环算。
 *
 * 安全：RequestEstop → sort_safe_stop_outputs（阀/带安全 + P0D-18=511）→ 两轴 P0D-05=1；
 * Poll 在 ESTOP 下轮询 P0B-07；ClearFault 写 P0D-05=0 并清闭锁。
 *
 * 数据流简图（与 main 模块总览一致）：
 *   KEY2 Modbus 矩阵落盘 + IK → WAIT 步满足 → SORTING → sorting_tick 按 s_sys_run_flag 调用
 *   AppArm_PickPlace / AppConveyor_RunUntilSensor；失败 enter_fault_mode；44 为短时告警分支。
 *
 * 【矩阵关闸】FL2/FL3 期间 `AppSort_ShouldDiscardTcpMatrixStream` 可丢弃 OnStream 字节流；Modbus 不经 ASCII 闸。
 * 采样关闸：`AppProtocol_ShouldAcceptStream`（ASCII 路径择优后关窗；Modbus 不经此闸）。
 *
 * 状态层次（读屏时对照）：
 *   s_main：人机大状态；s_step：RUN 内等矩阵还是分拣中；s_sys_run_flag：业务子步 FL2..。
 */

#include "app_sort.h"
#include "app_alarm_light.h"
#include "app_arm.h"
#include "app_conveyor.h"
#include "app_display.h"
#include "app_matrix.h"
#include "app_matrix_modbus.h"
#include "app_motor.h"
#include "app_protocol.h"
#include "bsp_conveyor.h"
#include "delay.h"
#include "global_config.h"
#include <stdio.h>
#include <string.h>

/* 外层人机看到的 IDLE/RUN/PAUSE/FAULT/ESTOP；它和触摸屏按键语义更接近 */

static AppSortMainState_e s_main;

/*
 * 内层节拍：WAIT_MATRIX 只是在「等新矩阵」，进了 SORTING 才真正跑标志位机。

 * 这样 Pause 时可以停在 SORTING 里冻结标志位推进。

 */

static AppSortStep_e      s_step;

/* 软件急停闭锁标志：只要它为 1，Start 按钮都会被挡住（必须先 Clear） */

static uint8_t            s_esop_hold;

/*
 * 业务「分拣标志」：不是触摸屏 RUN，而是「这一轮分拣想去干嘛」。

 * 取值详见 AppSortSysFlag_e（0=空闲等待；44=特殊告警流程；45=回家待机）。

 */

static AppSortSysFlag_e   s_sys_run_flag;

/*
 * 标志 44 专用：进入 44 后要有一段时间红灯快闪提示。

 * s_flag44_active 为 1 时，sorting_tick() 会优先处理这段超时逻辑并「立刻 return」，防止误入 switch 里空洞分支。

 */

static uint8_t            s_flag44_active;

static uint32_t           s_flag44_t0_ms;

/*
 * 待机回家请求可能是 ISR/触摸在 IDLE 态发来的：为了不阻塞 ISR，只置位，下一次 Poll 再真正 GoHome。

 */

static uint8_t            s_req_standby45;

/*
 * ESTOP 大状态下 RGB 指示策略：先快闪 CFG_ESTOP_BLINK_MS，到期切成 FAIL_HOLD（红灯常亮）。

 * s_estop_ind_hold_armed 用来记住「还在快闪计时窗口内」。

 */

static uint8_t            s_estop_ind_hold_armed;

static uint32_t           s_estop_blink_t0_ms;

/** ESTOP 闭锁期间缓存的两轴绝对脉冲（FC03 P0B-07），供屏显 AppSort_GetEstopMonitoredPositions */
static int32_t            s_estop_fb_m1;
static int32_t            s_estop_fb_m2;
static uint32_t           s_estop_mon_last_ms;
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
static uint32_t           s_debug_last_sort_step_ms;
#endif

/*
 * 功能：停机安全侧：阀/带输出置安全态并软件禁用两台伺服。
 * 交互：内部被 enter_fault_mode、RequestEstop、FL45、Poll 待机回家等调用；调用 BSP_Actuators_AllSafe、AppMotor_Disable。
 */
static void sort_safe_stop_outputs(void)
{
	BSP_Actuators_AllSafe();
	(void)AppMotor_Disable(CFG_MODBUS_SLAVE_MOTOR1);
	(void)AppMotor_Disable(CFG_MODBUS_SLAVE_MOTOR2);

}

/*
 * 功能：ESTOP 闭锁态下按 SERVO_POS_POLL_MS 读两轴 P0B-07 绝对脉冲并缓存。
 */
static void sort_poll_estop_position_monitor(uint32_t tick_ms)
{
	if ((tick_ms - s_estop_mon_last_ms) < (uint32_t)SERVO_POS_POLL_MS)
	{
		return;
	}
	s_estop_mon_last_ms = tick_ms;
	(void)AppMotor_ReadFeedbackPosition32(CFG_MODBUS_SLAVE_MOTOR1, &s_estop_fb_m1);
	(void)AppMotor_ReadFeedbackPosition32(CFG_MODBUS_SLAVE_MOTOR2, &s_estop_fb_m2);
}

/*
 * 功能：在盘 1 再盘 2 上查找首个 class=2 且几何有效的行索引。
 * 交互：内部被 run_flag2_once 调用；调用 AppMatrix_FindFirstRowByTrayClass。
 */
static uint8_t pick_src_row_class2(uint16_t *src)
{
	if (src == NULL)
	{
		return 0u;
	}
	if (AppMatrix_FindFirstRowByTrayClass(1u, 2u, src))
	{
		return 1u;
	}
	if (AppMatrix_FindFirstRowByTrayClass(2u, 2u, src))
	{
		return 1u;
	}
	return 0u;

}

/*
 * 功能：若上位声明行数不足，在 fault 行给出软提示（不锁死状态机）。
 * 交互：内部被 AppSort_Poll 每圈调用；读 AppMatrix_LastFrameUnderflow、AppDisplay_SetFaultText。
 */
static void notify_matrix_warn(void)
{
	if (AppMatrix_LastFrameUnderflow())
	{
		AppDisplay_SetFaultText("MAT<EXPECT rows");
	}

}

/*
 * 功能：判断「可开始分拣」的严格矩阵条件：`AppMatrixModbus_LastCommitOk`、有行、几何已 Flush 且有效。
 * 交互：内部被 run_step_machine_once、sorting_tick IDLE 调用；依赖 AppMatrixModbus_LastCommitOk、AppMatrix_*。
 */
static uint8_t sort_matrix_ready_strict(void)
{
	if (AppMatrixModbus_LastCommitOk() == 0u)
	{
		return 0u;
	}
	if (AppMatrix_GetValidCount() == 0u)
	{
		return 0u;
	}
	return AppMatrix_IsSortGeometryReady();

}

/*
 * 功能：检测「矩阵已提交但逆解未就绪」的不一致，用于进故障。
 * 交互：内部被 WAIT_MATRIX、SORTING IDLE 分支调用。
 */
static uint8_t sort_geom_inconsistent_fault(void)
{
	return (AppMatrixModbus_LastCommitOk() != 0u &&
		AppMatrix_GetValidCount() != 0u &&
		!AppMatrix_IsSortGeometryReady());

}

/*
 * 功能：进入 FAULT 大状态：复位步进与标志、安全停机、声光报警、写故障文案。
 * 交互：内部被各业务失败路径调用；调用 sort_safe_stop_outputs、AppIndicator_*、AlarmLight_*、AppDisplay_SetFaultText。
 */
static void enter_fault_mode(const char *txt)
{
	s_main = APP_SORT_MAIN_FAULT;
	s_step = APP_SORT_STEP_WAIT_MATRIX;
	s_sys_run_flag = APP_SORT_SYS_IDLE;
	s_flag44_active = 0u;
	sort_safe_stop_outputs();
	AppIndicator_SetState(APP_IND_FAIL_HOLD);
	(void)AlarmLight_Set(ALARM_LIGHT_COLOR_RED, ALARM_LIGHT_MODE_STEADY,
			     ALARM_LIGHT_BUZZER_ON);
	AppDisplay_SetFaultText(txt);

}

/*
 * 功能：系统标志 44：臂回家、红灯快闪计时窗口、非 FAULT 锁死式告警。
 * 交互：内部被 FL2 三盘不齐路径调用；调用 sort_safe_stop_outputs、AppArm_GoHome、灯与屏。
 */
static void enter_sys_flag44(uint32_t tick_ms)
{
	s_sys_run_flag = APP_SORT_SYS_FL44;
	s_flag44_active = 1u;
	s_flag44_t0_ms = tick_ms;
	sort_safe_stop_outputs();
	AppArm_GoHome();
	AppIndicator_SetState(APP_IND_FAIL_BLINK);
	(void)AlarmLight_Set(ALARM_LIGHT_COLOR_RED, ALARM_LIGHT_MODE_FAST,
			     ALARM_LIGHT_BUZZER_OFF);
	AppDisplay_SetFaultText("SYS44:tray/arm");

}

/*
 * 功能：执行 FL2 一步：二类苗优先归盘 3，失败或异常时切换 FL3/FL6 或进故障。
 * 交互：内部被 sorting_tick；调用 AppMatrix_*、AppArm_PickPlace、enter_fault_mode。
 */
static void run_flag2_once(void)
{
	uint16_t src;
	uint16_t dst;
	uint8_t  r3;
	/* 若盘 3 已经「满二类」，直接进入传送带 3 流程（标志 6） */
	if (AppMatrix_CheckTrayFullOrEmpty(3u, MATRIX_TRAY_COLS, MATRIX_TRAY_ROWS) == 2u)
	{
		s_sys_run_flag = APP_SORT_SYS_FL6;
		return;
	}
	/* 若盘 1、盘 2 都找不到二类苗，策略转到标志 3（处理一类苗） */
	if (!AppMatrix_TrayHasClassWithGeom(1u, 2u) && !AppMatrix_TrayHasClassWithGeom(2u, 2u))
	{
		s_sys_run_flag = APP_SORT_SYS_FL3;
		return;
	}
	if (!pick_src_row_class2(&src))
	{
		s_sys_run_flag = APP_SORT_SYS_FL3;
		return;
	}
	/* 盘 3 没有空穴却还停留在标志 2：判定异常转传送带 3 */
	if (!AppMatrix_FindFirstEmptyOnTray(3u, &dst))
	{
		s_sys_run_flag = APP_SORT_SYS_FL6;
		return;
	}
	r3 = AppArm_PickPlace(src, dst);
	if (r3 != 0u)
	{
		char b[48];
		(void)snprintf(b, sizeof(b), "ARM FL2 e%02u %s", (unsigned)r3,
			       AppArm_ResultText(r3));
		enter_fault_mode(b);
		return;
	}
	/*
	 * 搬完后再次确认「二类苗是否已经耗尽」。
	 * 若耗尽且盘 3 并未「满二类」，则可以退回标志 3 继续做一类苗整理。
	 */
	if (!AppMatrix_TrayHasClassWithGeom(1u, 2u) && !AppMatrix_TrayHasClassWithGeom(2u, 2u))
	{
		if (AppMatrix_CheckTrayFullOrEmpty(3u, MATRIX_TRAY_COLS,
						  MATRIX_TRAY_ROWS) != 2u)
		{
			s_sys_run_flag = APP_SORT_SYS_FL3;
		}
	}

}

/*
 * 功能：执行 FL3 一步：一类苗从盘 2 补盘 1，含传送带跳转与臂取放失败处理。
 * 交互：内部被 sorting_tick；调用 AppMatrix_*、AppArm_PickPlace、enter_fault_mode。
 */
static void run_flag3_once(void)
{
	uint16_t src;
	uint16_t dst;
	uint8_t  e;
	if (AppMatrix_CheckTrayFullOrEmpty(1u, MATRIX_TRAY_COLS, MATRIX_TRAY_ROWS) == 1u)
	{
		s_sys_run_flag = APP_SORT_SYS_FL4;
		return;
	}
	if (!AppMatrix_TrayHasClassWithGeom(2u, 1u))
	{
		s_sys_run_flag = APP_SORT_SYS_FL5;
		return;
	}
	if (!AppMatrix_FindFirstRowByTrayClass(2u, 1u, &src))
	{
		s_sys_run_flag = APP_SORT_SYS_FL5;
		return;
	}
	if (!AppMatrix_FindFirstEmptyOnTray(1u, &dst))
	{
		enter_fault_mode("FL3:no dst");
		return;
	}
	e = AppArm_PickPlace(src, dst);
	if (e != 0u)
	{
		char b[48];
		(void)snprintf(b, sizeof(b), "ARM FL3 e%02u %s", (unsigned)e,
			       AppArm_ResultText(e));
		enter_fault_mode(b);
	}

}

/*
 * 功能：FL4/5/6：臂回家后置指定传送带跑到光电停，超时/中止进故障。
 * 交互：内部被 sorting_tick；调用 AppArm_GoHome、AppConveyor_RunUntilSensor、AlarmLight_*。
 */
static void run_conveyor_flag(uint8_t conv_id)
{
	uint8_t cr;

	AppProtocol_ArmSampleWindow();
	AppArm_GoHome();
	cr = AppConveyor_RunUntilSensor(conv_id, CFG_CONVEYOR_TIMEOUT_MS);
	if (cr == 2u)
	{
		enter_fault_mode("CNV:ABORT");
		return;
	}
	if (cr != 0u)
	{
		enter_fault_mode("CNV TMOUT");
		return;
	}
	(void)AlarmLight_Set(ALARM_LIGHT_COLOR_GREEN, ALARM_LIGHT_MODE_STEADY,
			     ALARM_LIGHT_BUZZER_OFF);
	AppIndicator_SetState(APP_IND_OK_HOLD);
	s_sys_run_flag = APP_SORT_SYS_IDLE;

}

/*
 * 功能：RUN 内分拣内核一步：处理待机 45 请求、44 计时早退、按 s_sys_run_flag 执行各子策略。
 * 交互：内部被 run_step_machine_once、PAUSE 特殊路径调用；调用大量 AppMatrix/AppArm/AppConveyor 等。
 */
static void sorting_tick(uint32_t tick_ms)
{
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
	/*
	 * 联调观察用节拍：SIM 模式下机械臂/传送带都会瞬间返回成功。
	 * 如果不降速，FL2/FL3/FL4 等标志位可能在 LCD 刷新前就跑完。
	 * 这里把排序子步骤人为放慢，方便现场拍照确认状态机是否按预期推进。
	 */
	if ((tick_ms - s_debug_last_sort_step_ms) < (uint32_t)CFG_SORT_DEBUG_STEP_HOLD_MS)
	{
		return;
	}
	s_debug_last_sort_step_ms = tick_ms;
#endif
	if (s_req_standby45 != 0u)
	{
		s_req_standby45 = 0u;
		s_sys_run_flag = APP_SORT_SYS_FL45;
	}
	if (s_flag44_active != 0u)
	{
		if ((tick_ms - s_flag44_t0_ms) >= (uint32_t)CFG_SYS_FLAG44_ALARM_MS)
		{
			(void)AlarmLight_Off();
			AppIndicator_SetState(APP_IND_IDLE);
			s_flag44_active = 0u;
			s_sys_run_flag = APP_SORT_SYS_IDLE;
		}
		/* 只要还在 44 计时窗口，就别继续跑下面的 switch，避免同一周期重复 Enter */
		return;
	}
	switch (s_sys_run_flag)
	{
	case APP_SORT_SYS_IDLE:
		/*
		 * 「矩阵就绪」= Modbus 整表已成功写入 + 行数足够 + Flush 完成 +
		 * 全域 geom_ok（见 AppMatrix_IsSortGeometryReady）。
		 */
		if (sort_geom_inconsistent_fault())
		{
			enter_fault_mode("MAT:IK BAD");
			break;
		}
		if (sort_matrix_ready_strict())
		{
			s_sys_run_flag = APP_SORT_SYS_FL2;
		}
		break;
	case APP_SORT_SYS_FL2:
		if (!AppMatrix_AllThreeTraysPresent())
		{
			enter_sys_flag44(tick_ms);
			break;
		}
		run_flag2_once();
		break;
	case APP_SORT_SYS_FL3:
		run_flag3_once();
		break;
	case APP_SORT_SYS_FL4:
		run_conveyor_flag(BSP_CONV_ID_0);
		break;
	case APP_SORT_SYS_FL5:
		run_conveyor_flag(BSP_CONV_ID_1);
		break;
	case APP_SORT_SYS_FL6:
		run_conveyor_flag(BSP_CONV_ID_2);
		break;
	case APP_SORT_SYS_FL44:
		/*
		 * 理论上进了这里却没有 s_flag44_active 只能说明外部强行写了枚举。
		 * 为避免跑飞，当作 noop（真正的 44 超时在上面早退处理）。
		 */
		break;
	case APP_SORT_SYS_FL45:
		sort_safe_stop_outputs();
		AppArm_GoHome();
		s_sys_run_flag = APP_SORT_SYS_IDLE;
		break;
	default:
		/* 非法枚举：回落 IDLE，防止卡在莫名其妙数值 */
		s_sys_run_flag = APP_SORT_SYS_IDLE;
		break;
	}

}

/*
 * 功能：RUN 子状态机：WAIT_MATRIX 等矩阵；SORTING 调 sorting_tick 并维护臂运行指示灯。
 * 交互：内部被 AppSort_Poll RUN 分支调用。
 */
static void run_step_machine_once(uint32_t tick_ms)
{
	switch (s_step)
	{
	case APP_SORT_STEP_WAIT_MATRIX:
		AppIndicator_SetState(APP_IND_ARM_RUN);
		if (sort_geom_inconsistent_fault())
		{
			enter_fault_mode("MAT:IK FAIL");
			break;
		}
		if (sort_matrix_ready_strict())
		{
			s_step = APP_SORT_STEP_SORTING;
			s_sys_run_flag = APP_SORT_SYS_IDLE;
		}
		break;
	case APP_SORT_STEP_SORTING:
	default:
		sorting_tick(tick_ms);
		/*
		 * 只有在非 44 告警计时期间，才让面板灯表达「手臂正在跑分拣」。
		 * 否则 FAIL_BLINK / IDLE 会被这里覆盖掉，用户体验会乱。
		 */
		if (s_flag44_active == 0u &&
		    (s_sys_run_flag == APP_SORT_SYS_FL2 ||
		     s_sys_run_flag == APP_SORT_SYS_FL3))
		{
			AppIndicator_SetState(APP_IND_ARM_RUN);
		}
		break;
	}

}

/*
 * 功能：初始化分拣状态变量、指示与执行器安全态。
 * 交互：外部由 main 上电调用一次；写屏、BSP_Actuators_AllSafe。
 */
void AppSort_Init(void)
{
	s_esop_hold = 0u;
	s_main = APP_SORT_MAIN_IDLE;
	s_step = APP_SORT_STEP_WAIT_MATRIX;
	s_sys_run_flag = APP_SORT_SYS_IDLE;
	s_flag44_active = 0u;
	s_req_standby45 = 0u;
	s_estop_ind_hold_armed = 0u;
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
	s_debug_last_sort_step_ms = 0u;
#endif
	AppIndicator_SetState(APP_IND_IDLE);
	AppDisplay_SetRunFlagText("IDLE");
	AppDisplay_SetFaultText("idle");
	BSP_Actuators_AllSafe();
	s_estop_fb_m1 = 0;
	s_estop_fb_m2 = 0;
	s_estop_mon_last_ms = 0u;

}

/*
 * 功能：急停请求：闭锁 Start、切 ESTOP、安全停机、声光与屏显。
 * 交互：保留供将来上位机/触摸等调用；板载 KEY2 与 app_motion_abort 不再触发本函数。
 */
void AppSort_RequestEstop(void)
{
	s_esop_hold = 1u;
	s_main = APP_SORT_MAIN_ESTOP;
	s_step = APP_SORT_STEP_WAIT_MATRIX;
	s_sys_run_flag = APP_SORT_SYS_IDLE;
	s_flag44_active = 0u;
	sort_safe_stop_outputs();
	(void)AppMotor_SetEmergencyMode(CFG_MODBUS_SLAVE_MOTOR1, 1u);
	(void)AppMotor_SetEmergencyMode(CFG_MODBUS_SLAVE_MOTOR2, 1u);
	s_estop_mon_last_ms = 0u;
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
	s_debug_last_sort_step_ms = 0u;
#endif
	sort_poll_estop_position_monitor(SysTick_GetMs());
	s_estop_blink_t0_ms = SysTick_GetMs();
	s_estop_ind_hold_armed = 1u;
	AppIndicator_SetState(APP_IND_FAIL_BLINK);
	(void)AlarmLight_Set(ALARM_LIGHT_COLOR_RED, ALARM_LIGHT_MODE_FAST,
			     ALARM_LIGHT_BUZZER_OFF);
	AppDisplay_SetFaultText("ESTOP(latch)");
	AppDisplay_SetRunFlagText("ESTOP");

}

/*
 * 功能：清除故障/急停闭锁，回到 IDLE 等待矩阵，关灯复位指示。
 * 交互：外部由 UI「清除」类操作调用。
 */
void AppSort_RequestClearFault(void)
{
	s_esop_hold = 0u;
	s_estop_ind_hold_armed = 0u;
	(void)AppMotor_ClearTrigger(CFG_MODBUS_SLAVE_MOTOR1);
	(void)AppMotor_ClearTrigger(CFG_MODBUS_SLAVE_MOTOR2);
	(void)AppMotor_SetEmergencyMode(CFG_MODBUS_SLAVE_MOTOR1, 0u);
	(void)AppMotor_SetEmergencyMode(CFG_MODBUS_SLAVE_MOTOR2, 0u);
	BSP_Actuators_AllSafe();
	(void)AlarmLight_Off();
	s_main = APP_SORT_MAIN_IDLE;
	s_step = APP_SORT_STEP_WAIT_MATRIX;
	s_sys_run_flag = APP_SORT_SYS_IDLE;
	s_flag44_active = 0u;
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
	s_debug_last_sort_step_ms = 0u;
#endif
	AppIndicator_SetState(APP_IND_IDLE);
	AppDisplay_SetFaultText("cleared,IDLE");
	AppDisplay_SetRunFlagText("CLR");

}

/*
 * 功能：启动或恢复 RUN；若急停闭锁则拒绝。
 * 交互：外部由触摸/上位机路径调用。
 */
void AppSort_RequestStart(void)
{
	if (s_esop_hold != 0u)
	{
		AppDisplay_SetFaultText("blocked: ESTOP clr");
		return;
	}
	if (s_main == APP_SORT_MAIN_PAUSE)
	{
		s_main = APP_SORT_MAIN_RUN;
		AppDisplay_SetRunFlagText("RESUME");
		return;
	}
	if ((s_main == APP_SORT_MAIN_IDLE) || (s_main == APP_SORT_MAIN_FAULT))
	{
		s_main = APP_SORT_MAIN_RUN;
		s_step = APP_SORT_STEP_WAIT_MATRIX;
		s_sys_run_flag = APP_SORT_SYS_IDLE;
		s_flag44_active = 0u;
#if CFG_SORT_DEBUG_STEP_HOLD_MS > 0u
		s_debug_last_sort_step_ms = 0u;
#endif
		(void)AlarmLight_Off();
		AppDisplay_SetFaultText("RUN");
		AppDisplay_SetRunFlagText("RUN");
	}

}

/*
 * 功能：将大状态切 PAUSE 并安全停机（若当前为 RUN）。
 * 交互：外部由 UI 暂停调用。
 */
void AppSort_RequestPause(void)
{
	if (s_main == APP_SORT_MAIN_RUN)
	{
		s_main = APP_SORT_MAIN_PAUSE;
		BSP_Actuators_AllSafe();
		AppIndicator_SetState(APP_IND_PAUSE_BLINK);
		AppDisplay_SetRunFlagText("PAUSE");
	}

}

/*
 * 功能：请求臂回待机位：RUN+SORTING 时切 FL45；IDLE/PAUSE 时仅置位由 Poll 执行 GoHome。
 * 交互：外部由触摸调用；与 AppArm_GoHome、Modbus 阻塞边界相关。
 */
void AppSort_RequestStandbyHome(void)
{
	if (s_esop_hold != 0u)
	{
		return;
	}
	if (s_main == APP_SORT_MAIN_RUN &&
	    s_step == APP_SORT_STEP_SORTING)
	{
		s_sys_run_flag = APP_SORT_SYS_FL45;
		return;
	}
	if (s_main == APP_SORT_MAIN_IDLE || s_main == APP_SORT_MAIN_PAUSE)
	{
		s_req_standby45 = 1u;
	}

}

/*
 * 功能：查询急停闭锁标志 s_esop_hold。
 * 交互：外部由 main KEY2 去抖、UI 查询。
 */
uint8_t AppSort_IsEstopLatched(void)
{
	return s_esop_hold;

}

/*
 * 功能：返回人机大状态 s_main。
 * 交互：外部显示/调试。
 */
AppSortMainState_e AppSort_GetMainState(void)
{
	return s_main;

}

/*
 * 功能：返回 RUN 内子步 s_step（WAIT_MATRIX / SORTING）。
 * 交互：外部显示/调试。
 */
AppSortStep_e AppSort_GetStep(void)
{
	return s_step;

}

/*
 * 功能：返回业务分拣子标志枚举值。
 * 交互：外部调试；与触摸屏 FL 文本拼装配合。
 */
uint8_t AppSort_GetSysRunFlag(void)
{
	return (uint8_t)s_sys_run_flag;

}

/*
 * 功能：将 s_sys_run_flag 译为短字符串。
 * 交互：外部屏显 app_display。
 */
const char *AppSort_GetSysRunFlagText(void)
{
	switch (s_sys_run_flag)
	{
	case APP_SORT_SYS_IDLE:
		return "IDLE";
	case APP_SORT_SYS_FL2:
		return "FL2";
	case APP_SORT_SYS_FL3:
		return "FL3";
	case APP_SORT_SYS_FL4:
		return "FL4";
	case APP_SORT_SYS_FL5:
		return "FL5";
	case APP_SORT_SYS_FL6:
		return "FL6";
	case APP_SORT_SYS_FL44:
		return "F44";
	case APP_SORT_SYS_FL45:
		return "H45";
	default:
		return "?";
	}

}

/*
 * 功能：将 s_main 译为短字符串。
 * 交互：外部屏显。
 */
const char *AppSort_GetMainStateText(void)
{
	switch (s_main)
	{
	case APP_SORT_MAIN_RUN:
		return "RUN ";
	case APP_SORT_MAIN_PAUSE:
		return "PAUS";
	case APP_SORT_MAIN_FAULT:
		return "FAULT";
	case APP_SORT_MAIN_ESTOP:
		return "ESTOP";
	case APP_SORT_MAIN_IDLE:
	default:
		return "IDLE ";
	}

}

/*
 * 功能：将子步枚举译为固定宽字符串。
 * 交互：仅 AppSort_Poll 拼装 run flag 文本。
 */
static const char *step_text(AppSortStep_e st)
{
	switch (st)
	{
	case APP_SORT_STEP_WAIT_MATRIX:
		return "W_MTX";
	case APP_SORT_STEP_SORTING:
		return "SORT ";
	default:
		return "---- ";
	}

}

/*
 * 功能：FL2/FL3 忙时用于丢弃 ASCII OnStream 调试字节流（避免干扰解析窗口）。量产 Modbus 不经此路径。
 * 交互：可由 lwIP 打开时的调试字节流回调查询（双 RS485 量产镜像通常无此入口）；名称保留以兼容旧调用点。
 */
uint8_t AppSort_ShouldDiscardTcpMatrixStream(void)
{
	if (s_main != APP_SORT_MAIN_RUN)
	{
		return 0u;
	}
	if (s_step != APP_SORT_STEP_SORTING)
	{
		return 0u;
	}
	if (s_sys_run_flag != APP_SORT_SYS_FL2 &&
	    s_sys_run_flag != APP_SORT_SYS_FL3)
	{
		return 0u;
	}
	return 1u;
}

/*
 * 功能：读取 ESTOP 监视缓存的两轴绝对脉冲（Poll 内刷新）。
 */
void AppSort_GetEstopMonitoredPositions(int32_t *m1, int32_t *m2)
{
	if (m1 != NULL)
	{
		*m1 = s_estop_fb_m1;
	}
	if (m2 != NULL)
	{
		*m2 = s_estop_fb_m2;
	}
}

/*
 * 功能：分拣调度主入口：Flush 矩阵几何、急停指示灯超时、待机回家、按大状态走一步并刷新 UI 摘要行。
 * 交互：外部由 main 主循环周期性调用；内部调用 AppMatrix_FlushPendingGeometry、run_step_machine_once/sorting_tick、AppDisplay_*。
 */
void AppSort_Poll(uint32_t tick_ms)
{
	char rfline[48];
	/*
	 * 每圈必做：把矩阵解析后缓存的「待算几何」在这里算完。
	 * 好处：主循环可控；坏处：若您从这里单步调试，会看到 Poll 偶尔多花时间——属于正常。
	 */
	AppMatrix_FlushPendingGeometry();
	if ((s_main == APP_SORT_MAIN_ESTOP) && (s_estop_ind_hold_armed != 0u))
	{
		if ((tick_ms - s_estop_blink_t0_ms) >= (uint32_t)CFG_ESTOP_BLINK_MS)
		{
			AppIndicator_SetState(APP_IND_FAIL_HOLD);
			s_estop_ind_hold_armed = 0u;
		}
	}
	notify_matrix_warn();
	/*
	 * IDLE/PAUSE 下的「请求回家」：真正 GoHome 放在 Poll，而不是 EXTI ISR。
	 * 原因是 GoHome 里会有 Modbus 阻塞调用。
	 */
	if (s_req_standby45 != 0u &&
	    (s_main == APP_SORT_MAIN_IDLE || s_main == APP_SORT_MAIN_PAUSE))
	{
		s_req_standby45 = 0u;
		BSP_Actuators_AllSafe();
		AppArm_GoHome();
	}
	switch (s_main)
	{
	case APP_SORT_MAIN_IDLE:
		AppIndicator_SetState(APP_IND_IDLE);
		break;
	case APP_SORT_MAIN_PAUSE:
		AppIndicator_SetState(APP_IND_PAUSE_BLINK);
		/*
		 * Pause 仍允许 44 超时演进（用户体验：暂停并不等于冻结安全告警计时）。
		 * 若不希望如此，可改成在外层直接挡住 sorting_tick。
		 */
		if (s_flag44_active != 0u)
		{
			sorting_tick(tick_ms);
		}
		break;
	case APP_SORT_MAIN_FAULT:
		/*
		 * 故障：不再调度分拣内核。
		 */
		break;
	case APP_SORT_MAIN_ESTOP:
		sort_poll_estop_position_monitor(tick_ms);
		break;
	case APP_SORT_MAIN_RUN:
		run_step_machine_once(tick_ms);
		break;
	default:
		break;
	}
	(void)snprintf(rfline, sizeof(rfline), "FL%u:%s %s",
		       (unsigned)AppSort_GetSysRunFlag(),
		       AppSort_GetSysRunFlagText(), step_text(s_step));
	rfline[sizeof(rfline) - 1u] = '\0';
	AppDisplay_SetRunFlagText(rfline);
	AppDisplay_SetServoBriefText(AppSort_GetMainStateText(), step_text(s_step));

}
