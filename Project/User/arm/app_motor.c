/**
 * app_motor.c — 两台关节伺服：经 RS485 Modbus 下发目标与读反馈
 *
 * 电机挂在 USART1 RS485（脚位见 main.c、global_config.h）。从站号 CFG_MODBUS_SLAVE_MOTOR1/2。
 *
 * 32 位脉冲在 FC16 / FC03 载荷中为「低 16 位寄存器在前、高 16 位寄存器在后」，
 * 每个寄存器内线序为大端（高字节在前）。
 *
 * 点到点绝对流程（AppMotor_GotoPulse / GotoAbsTargetAsRelative）：
 * FC16 写 SERVO_REG_POS32_START → FC06 SERVO_REG_ENABLE=ON → FC06 SERVO_REG_TRIGGER=ABS_MOVE(3)
 * → WaitPosition（P0B-04.Bit0 或 |P0B-07−目标|≤SERVO_TARGET_TOL_PULSE）→ FC06 TRIGGER=CLEAR(0)。
 *
 * AppMotor_GotoAbsTargetAsRelative：先读反馈，在 MCU 内算 Δ；默认仍向寄存器写绝对终点。
 *
 * AppMotor_ServoAngleToPulse：关节角经 CFG_ICHUANDONG_RATIO 与 SERVO_PULSE_PER_360_REV 换脉冲。
 */
#include "app_motor.h"
#include "modbus_master.h"
#include "global_config.h"
#include "delay.h"

#include "app_motion_abort.h"

#include <string.h>

#define APP_MOTOR_RXCAP 128u

/*
 * 功能：解析 FC03 读 2 个保持寄存器（SERVO_REG_FB_POS32_START）为 32bit 有符号位置；低字在前。
 */
static uint8_t parse_holdings_fb_pos32(const uint8_t *rx, uint16_t rxl, int32_t *pos)
{
	uint32_t u32;
	uint16_t low16;
	uint16_t high16;

	if (pos == NULL)
	{
		return MB_MASTER_ERR_ARG;
	}
	if ((rxl < 9u) || (rx[1u] != 0x03u) || (rx[2u] < 4u))
	{
		return MB_MASTER_ERR_LEN;
	}
	low16 = (uint16_t)(((uint16_t)rx[3u] << 8) | (uint16_t)rx[4u]);
	high16 = (uint16_t)(((uint16_t)rx[5u] << 8) | (uint16_t)rx[6u]);
	u32 = ((uint32_t)low16) | ((uint32_t)high16 << 16);
	*pos = (int32_t)u32;
	return MB_MASTER_OK;
}

/*
 * 功能：解析 FC03 读 1 个保持寄存器的应答。
 */
static uint8_t parse_holdings_u16(const uint8_t *rx, uint16_t rxl, uint16_t *out_val)
{
	if (out_val == NULL)
	{
		return MB_MASTER_ERR_ARG;
	}
	if ((rxl < 7u) || (rx[1u] != 0x03u) || (rx[2u] < 2u))
	{
		return MB_MASTER_ERR_LEN;
	}
	*out_val = (uint16_t)(((uint16_t)rx[3u] << 8) | (uint16_t)rx[4u]);
	return MB_MASTER_OK;
}

/*
 * 功能：把 32bit 有符号脉冲拆成 FC10 的两个 Modbus 寄存器（低字在前），每寄存器大端字节序。
 */
static void pulse32_to_modbus_regs_lo_hi(int32_t v, uint8_t *four)
{
	uint32_t u = (uint32_t)v;
	uint16_t low16 = (uint16_t)(u & 0xFFFFu);
	uint16_t high16 = (uint16_t)((u >> 16) & 0xFFFFu);

	four[0] = (uint8_t)((low16 >> 8) & 0xFFu);
	four[1] = (uint8_t)(low16 & 0xFFu);
	four[2] = (uint8_t)((high16 >> 8) & 0xFFu);
	four[3] = (uint8_t)(high16 & 0xFFu);
}

/*
 * 功能：关节角(°)换算为伺服脉冲整数（按减速比与每圈脉冲宏）。
 */
int32_t AppMotor_ServoAngleToPulse(float joint_angle_deg)
{
	double p = (double)joint_angle_deg * (double)CFG_ICHUANDONG_RATIO;

	p = p * (double)SERVO_PULSE_PER_360_REV / 360.0;
	return (int32_t)p;
}

/*
 * 功能：FC10 写入 32bit 目标位置寄存器起始地址（低字寄存器在前）。
 */
uint8_t AppMotor_WriteTargetPulse(uint8_t slave, int32_t pulse)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;
	uint8_t pl[4];

	pulse32_to_modbus_regs_lo_hi(pulse, pl);
	return ModbusMaster_WriteMultipleRegisters(slave, SERVO_REG_POS32_START, pl, 4u,
						   rx, APP_MOTOR_RXCAP, &rxl,
						   CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 写使能 ON。
 */
uint8_t AppMotor_Enable(uint8_t slave)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_ENABLE, SERVO_VAL_ENABLE_ON,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 写使能 OFF（急停或异常收尾）。
 */
uint8_t AppMotor_Disable(uint8_t slave)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_ENABLE, SERVO_VAL_ENABLE_OFF,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 写急停模式寄存器。
 */
uint8_t AppMotor_SetEmergencyMode(uint8_t slave, uint8_t on)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;
	uint16_t val = (on != 0u) ? SERVO_VAL_ESTOP_ON : SERVO_VAL_ESTOP_OFF;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_ESTOP_MODE, val,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 触发绝对位置移动（总线指令）。
 */
uint8_t AppMotor_TriggerAbsMove(uint8_t slave)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_TRIGGER, SERVO_VAL_TRIGGER_ABS_MOVE,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 清除总线触发 / 运行指令（P0D-08=0）。
 */
uint8_t AppMotor_ClearTrigger(uint8_t slave)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_TRIGGER, SERVO_VAL_TRIGGER_CLEAR,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：FC06 触发伺服运行当前目标（历史语义；点到点流程请用 TriggerAbsMove）。
 */
uint8_t AppMotor_TriggerRun(uint8_t slave)
{
	uint8_t rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	return ModbusMaster_WriteSingleRegister(slave, SERVO_REG_TRIGGER, SERVO_VAL_TRIGGER_RUN,
					       rx, APP_MOTOR_RXCAP, &rxl,
					       CFG_MODBUS_RSP_TIMEOUT_MS);
}

/*
 * 功能：同 ClearTrigger（兼容旧名）。
 */
uint8_t AppMotor_TriggerReset(uint8_t slave)
{
	return AppMotor_ClearTrigger(slave);
}

/*
 * 功能：FC03 读 SERVO_REG_REACH_STATUS；Bit0→reached_bit0。
 */
uint8_t AppMotor_ReadReachedStatus(uint8_t slave, uint8_t *reached_bit0)
{
	uint8_t  rx[APP_MOTOR_RXCAP];
	uint16_t rxl;
	uint8_t  err;
	uint16_t regv;

	if (reached_bit0 == NULL)
	{
		return MB_MASTER_ERR_ARG;
	}
	*reached_bit0 = 0u;
	memset(rx, 0, sizeof(rx));
	rxl = 0u;
	err = ModbusMaster_ReadHoldingRegisters(slave, SERVO_REG_REACH_STATUS, 1u,
						rx, APP_MOTOR_RXCAP, &rxl,
						CFG_MODBUS_RSP_TIMEOUT_MS);
	if (err != MB_MASTER_OK)
	{
		return err;
	}
	err = parse_holdings_u16(rx, rxl, &regv);
	if (err != MB_MASTER_OK)
	{
		return err;
	}
	if ((regv & 1u) != 0u)
	{
		*reached_bit0 = 1u;
	}
	return MB_MASTER_OK;
}

uint8_t AppMotor_ReadFeedbackPosition32(uint8_t slave, int32_t *out_pos)
{
	uint8_t  rx[APP_MOTOR_RXCAP];
	uint16_t rxl;
	uint8_t  err;

	if (out_pos == NULL)
	{
		return MB_MASTER_ERR_ARG;
	}
	memset(rx, 0, sizeof(rx));
	rxl = 0u;

	err = ModbusMaster_ReadHoldingRegisters(slave, SERVO_REG_FB_POS32_START, 2u,
						rx, APP_MOTOR_RXCAP, &rxl,
						CFG_MODBUS_RSP_TIMEOUT_MS);
	if (err != MB_MASTER_OK)
	{
		return err;
	}
	return parse_holdings_fb_pos32(rx, rxl, out_pos);
}

uint8_t AppMotor_WaitPosition(uint8_t slave, int32_t target_pulse, uint32_t timeout_ms)
{
	uint32_t t0 = SysTick_GetMs();
	uint8_t  rx[APP_MOTOR_RXCAP];
	uint16_t rxl;

	for (;;)
	{
		uint8_t err;
		uint8_t rch = 0u;

		if (AppMotionAbort_PollEscalate())
		{
			(void)AppMotor_Disable(slave);
			(void)AppMotor_ClearTrigger(slave);
			return MB_MASTER_ERR_ABORT;
		}

		if ((SysTick_GetMs() - t0) >= timeout_ms)
		{
			(void)AppMotor_ClearTrigger(slave);
			return MB_MASTER_ERR_TIMEOUT;
		}

		err = AppMotor_ReadReachedStatus(slave, &rch);
		if ((err == MB_MASTER_OK) && (rch != 0u))
		{
			(void)AppMotor_ClearTrigger(slave);
			return MB_MASTER_OK;
		}

		memset(rx, 0, sizeof(rx));
		rxl = 0u;

		err = ModbusMaster_ReadHoldingRegisters(slave, SERVO_REG_FB_POS32_START, 2u,
							rx, APP_MOTOR_RXCAP, &rxl,
							CFG_MODBUS_RSP_TIMEOUT_MS);

		if (err == MB_MASTER_OK)
		{
			int32_t cur;

			err = parse_holdings_fb_pos32(rx, rxl, &cur);
			if (err == MB_MASTER_OK)
			{
				int32_t diff = cur - target_pulse;

				if ((diff <= (int32_t)SERVO_TARGET_TOL_PULSE) &&
				    (diff >= -(int32_t)SERVO_TARGET_TOL_PULSE))
				{
					(void)AppMotor_ClearTrigger(slave);
					return MB_MASTER_OK;
				}
			}
		}

		Delay_ms(SERVO_POS_POLL_MS);
	}
}

/*
 * 功能：伺服点到点绝对流程：写目标 → 使能 → 触发 ABS_MOVE → Wait →（Wait 内清 TRIGGER）。
 */
uint8_t AppMotor_GotoPulse(uint8_t slave, int32_t pulse)
{
	uint8_t e;

	e = AppMotor_WriteTargetPulse(slave, pulse);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	e = AppMotor_Enable(slave);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	e = AppMotor_TriggerAbsMove(slave);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	return AppMotor_WaitPosition(slave, pulse, SERVO_MOVE_TIMEOUT_MS);
}

uint8_t AppMotor_GotoAbsTargetAsRelative(uint8_t slave, int32_t target_abs)
{
	uint8_t e;
	int32_t cur = 0;
	int32_t delta;
	int32_t cmd_pulse;

	e = AppMotor_ReadFeedbackPosition32(slave, &cur);
	if (e != MB_MASTER_OK)
	{
		return AppMotor_GotoPulse(slave, target_abs);
	}

	delta = target_abs - cur;

#if (CFG_SERVO_POS_CMD_IS_REL_DELTA != 0u)
	cmd_pulse = delta;
#else
	cmd_pulse = target_abs;

	(void)delta;
#endif

	e = AppMotor_WriteTargetPulse(slave, cmd_pulse);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	e = AppMotor_Enable(slave);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	e = AppMotor_TriggerAbsMove(slave);
	if (e != MB_MASTER_OK)
	{
		return e;
	}
	return AppMotor_WaitPosition(slave, target_abs, SERVO_MOVE_TIMEOUT_MS);
}
