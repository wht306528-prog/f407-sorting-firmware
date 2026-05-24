/**
 * main.c — STM32F407 分拣控制固件入口（裸机单线程，无 FreeRTOS）
 *
 * 文件角色：整机入口。上电做外设初始化，然后 for(;;) 轮询触摸、按键、矩阵 Modbus、分拣状态机、屏幕。
 *
 * IO 与真源（引脚/波特率以 global_config.h 为准，文字说明见 F407/docs/F407_IO引脚与逻辑表.txt）：
 *   RS485 Modbus 单总线（电机/报警灯/KEY1）：USART2 PA2=TX, PA3=RX, PC0=DE（高=发送占线，低=接收）。波特率 CFG_RS485_BAUD。
 *   矩阵视觉：USART3 PB10=TX、PB11=RX，Modbus RTU 读保持寄存器 → app_matrix_modbus → app_matrix（量产无 lwIP 网口矩阵，仅 RS485）。
 *   执行器与传感器（电磁阀/三带电机/光电/备用输入）：
 *     VALVE_ACTUATOR_Z  PA8；VALVE_GRIP PB2；
 *     CONV_MOTOR_0 PB0，CONV_MOTOR_1 PB12，CONV_MOTOR_2 PE5；
 *     PHOTO_0 PB13，PHOTO_1 PB15，PHOTO_2 PE3；
 *     LIMIT_SOFT1 PE4，ESTOP PE6（bsp 急停占位脚；软件急停 API 仍为 AppSort_RequestEstop，当前不由板载 KEY 触发）。
 *   人机：NT35510 并口屏（FSMC）；GT911 触摸（软件 I2C，初始化在 GTP_Init_Panel）。
 *   按键：KEY1 PA0 EXTI0 → USART2 RS485 单帧 FC06 写七色灯（站 0x03 寄存 0xC2=0x0013）仅发不等应答；KEY2 PC13 → USART3 矩阵 Modbus 单轮。
 *
 * 伺服寄存器（Modbus 从站号 CFG_MODBUS_SLAVE_MOTOR1/2；FC06 写单寄存器、FC10 写多寄存器、FC03 读保持寄存器，见 app_motor.c / modbus_master）：
 *   SERVO_REG_ENABLE 0x0D12：上电使能，SERVO_VAL_ENABLE_ON/OFF。
 *   SERVO_REG_TRIGGER 0x0D08：绝对移动触发 SERVO_VAL_TRIGGER_ABS_MOVE(3)，结束写 CLEAR(0)。
 *   SERVO_REG_POS32_START 0x100E：32bit 绝对目标（FC16，低 16 位寄存器在前）。
 *   SERVO_REG_FB_POS32_START 0x0B07：反馈绝对脉冲 FC03 读 2 寄存器，低字在前合成 32 位。
 *   SERVO_REG_REACH_STATUS 0x0B04：到位 Bit0；与反馈误差≤SERVO_TARGET_TOL_PULSE 二选一。
 *   SERVO_REG_ESTOP_MODE 0x0D05：急停模式寄存器。
 * 报警灯：站号 CFG_ALARM_MODBUS_SLAVE，寄存器 CFG_ALARM_REG_DIRECT，FC06 写色与模式（app_alarm_light.c）。
 *
 * 返回码入口（LCD/ fault 行常显简写；代码里用数字）：
 *   Modbus 主机：MB_MASTER_OK=0 至 MB_MASTER_ERR_ABORT=10（modbus_master.h）。
 *   机械臂取放：AppArm_PickPlace 非 0 见 AppArm_ResultText()（app_arm.c）。
 *   传送带：AppConveyor_RunUntilSensor 0 成功，1 超时，2 急停中止（app_conveyor.c）。
 *   矩阵满盘判定：AppMatrix_CheckTrayFullOrEmpty 返回 0/1/2 或 44 非法（与系统标志 44 不同层，见 app_matrix.h）。
 *   分拣大状态与 SYS 标志：AppSortMainState_e、AppSortSysFlag_e（app_sort.h），如 FL2..FL6、44 告警计时、45 回位。
 *
 * 启动顺序：SYS_Init 与 Delay_Init；LED/蜂鸣器/传送带 BSP；USART2+DE；USART3 矩阵口；按键与 EXTI；报警灯 Init；LCD 与 AppDisplay；触摸；AppMatrixModbus_Init；AppSort_Init。
 * 主循环顺序：触摸轮询；KEY1/KEY2；AppMatrixModbus_Poll；AppSort_Poll（含矩阵几何 Flush）；AppDisplay_Refresh。
 *
 * 按键：KEY1 仅发灯 Modbus RAW+CRC（ModbusMaster_SendRawNoResponse）；KEY2 AppMatrixModbus_Key2Request + 蓝灯指示。
 *
 * 游戏逻辑与功能模块（main 负责调度，业务在对应 .c）：
 *   1) 矩阵：USART3 Modbus RTU → app_matrix_modbus → app_matrix；逆解在 AppSort_Poll 内 Flush。
 *   2) 分拣状态机：app_sort（IDLE/RUN/PAUSE/FAULT/ESTOP + RUN 内 WAIT_MATRIX + SORTING + FL2..FL6/44/45）。
 *   3) 机械臂与伺服：app_arm、app_motor、app_kinematics（逆解结果在 matrix 行内）。
 *   4) 传送带与 IO：app_conveyor、bsp_conveyor（光电、阀、带）。
 *   5) RS485 主站：modbus_master、app_alarm_light；KEY1 灯单帧亦走 modbus_master。
 *   6) 人机：app_display、GT911、KEY/EXTI；运动中Abort 仅认急停闭锁（AppMotionAbort_PollEscalate）。
 */

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <stdio.h>

#include "delay.h"
#include "global_config.h"
#include "sys.h"

#include "app_alarm_light.h"
#include "app_display.h"
#include "app_matrix_modbus.h"
#include "app_sort.h"
#include "bsp_buzzer.h"
#include "bsp_conveyor.h"
#include "bsp_exti.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_nt35510_lcd.h"
#include "bsp_uart.h"
#include "bsp_uart3.h"
#include "gt9xx.h"
#include "modbus_crc.h"
#include "modbus_master.h"
#include "stm32f4xx_it.h"

/*
 * 功能：程序入口；完成系统与各外设初始化，循环处理触摸、按键、矩阵 Modbus、分拣调度与刷屏。
 * 交互：矩阵 USART3 Modbus；伺服/灯经 USART2（BSP_USART1_*）+ modbus_master；人机经 AppDisplay/GTP_Key/EXTI。
 */
int main(void)
{
	uint32_t tick;

	SYS_Init(); /* RCC、SysTick 等系统级初始化（sys.c）*/
	Delay_Init(); /* Delay_Inc 与 SysTick_Handler 对齐的毫秒节拍 */

	BSP_LED_Init(); /* RGB/板载指示灯 */
	BSP_Buzzer_Init();
	BSP_Conveyor_Init(); /* 阀、带、光电、急停备用脚 GPIO */

	BSP_Uart_HW_Init(CFG_RS485_BAUD); /* USART2 PA2/PA3 + PC0 DE，RS485 主站底层 */

	BSP_Uart3_HW_Init(CFG_MATRIX_MODBUS_BAUD); /* USART3 PB10/11 矩阵 Modbus */
	AppMatrixModbus_Init();

	BSP_Key_Init(); /* GPIO 读取 KEY 电平 */
	BSP_EXTI_Keys_Init(); /* KEY1 EXTI0 / KEY2 EXTI13 下降沿，只置中断标志 */

	AlarmLight_Init();
	(void)AlarmLight_Off(); /* 开机默认关灯，分拣里再设色 */

	NT35510_Init();
	AppDisplay_Init(); /* LCD 文本页缓冲与指示灯状态 */

	(void)GTP_Init_Panel(); /* GT911 + 面板触摸校准入口 */

	AppSort_Init(); /* 分拣变量、s_esop_hold=0、执行器 AllSafe */

	for (;;) {
		tick = SysTick_GetMs();

		GTP_TouchPoll(); /* 触摸坐标与页面切换 */

		{
			uint32_t dum_ms;

			if (EXTI_PopKey1(&dum_ms) != 0u) {
				uint8_t adu[8];

				/*
				 * 【KEY1】USART2 RS485：仅发 03 06 00 C2 00 13 + Modbus CRC16，
				 * 不等从站应答（NO_RESPONSE）。不与 AppArm_GoHome 连用。
				 */
				adu[0] = CFG_KEY1_LAMP_SLAVE;
				adu[1] = 0x06u;
				adu[2] = (uint8_t)((CFG_KEY1_LAMP_REG >> 8) & 0xFFu);
				adu[3] = (uint8_t)(CFG_KEY1_LAMP_REG & 0xFFu);
				adu[4] = (uint8_t)((CFG_KEY1_LAMP_VALUE >> 8) & 0xFFu);
				adu[5] = (uint8_t)(CFG_KEY1_LAMP_VALUE & 0xFFu);
				Modbus_AppendCRC(adu, 6u);
				(void)ModbusMaster_SendRawNoResponse(adu, 8u,
					  MB_MUTEX_WAIT_MS);
				AppDisplay_SetRunFlagText("KEY1 LAMP TX");
			}

			if (EXTI_PopKey2() != 0u) {
				uint8_t mr;

				/*
				 * 【KEY2】USART3 矩阵 Modbus：单轮读头→15 包→再读头→CRC32→重建矩阵（忙时拒绝）。
				 */
				mr = AppMatrixModbus_Key2Request();
				if (mr == 0u) {
					AppIndicator_SetState(APP_IND_ARM_RUN);
					AppDisplay_SetRunFlagText("MAT READ...");
					/*
					 * 整轮内连续 Poll：每步先 TX FC03 再收齐一帧 RX；通过后组矩阵并 FreezeRaw 子页。
					 * 循环内 AppDisplay_Refresh（受 CFG_UI_REFRESH_MS 节流），便于观察 rows/pkt 与 RunFlag。
					 */
					while (AppMatrixModbus_IsBusy() != 0u) {
						AppMatrixModbus_Poll(SysTick_GetMs());
						GTP_TouchPoll();
						AppDisplay_Refresh(SysTick_GetMs());
					}
#if CFG_SORT_DEBUG_AUTO_START_AFTER_KEY2
					/*
					 * 联调入口：KEY2 读矩阵成功后自动启动 AppSort。
					 * 当前没有可靠触摸/上位机 Start 入口；这个开关只用于 SIM 阶段，
					 * 目的是观察 RunFlag/FL 是否能从 IDLE 进入 WAIT/SORTING/FL2/FL3。
					 */
					AppSort_RequestStart();
#endif
				} else {
					AppDisplay_SetRunFlagText("MAT READ BUSY");
				}
			}
		}

		AppMatrixModbus_Poll(tick);
		AppSort_Poll(tick); /* 含矩阵几何 Flush + 状态机一步 */
		AppDisplay_Refresh(tick); /* 按 CFG_UI_REFRESH_MS 节流整页刷屏 */
	}
}
