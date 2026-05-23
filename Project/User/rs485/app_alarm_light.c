/*
 * app_alarm_light.c — 七色报警灯（RS485 Modbus 从站，与伺服同总线）
 *
 * 站号 CFG_ALARM_MODBUS_SLAVE；FC06 写单寄存器 CFG_ALARM_REG_DIRECT，数值为灯色与蜂鸣组合。
 * App485_Alarm_DirectFc06_NoResp 发完不等应答；app_sort 在故障/完成等节点调 AlarmLight_Set/Off。
 */

#include "app_alarm_light.h"

#include "app_485_devices.h"

#include "delay.h"
#include "global_config.h"
#include "modbus_master.h"

#include <stddef.h>

static const uint16_t k_steady_no_horn[7] = {
	0x0011u, 0x0012u, 0x0013u, 0x0015u, 0x0016u, 0x0017u, 0x0018u,
};

static const uint16_t k_slow_no_horn[7] = {
	0x0021u, 0x0022u, 0x0023u, 0x0025u, 0x0026u, 0x0027u, 0x0028u,
};

static const uint16_t k_fast_no_horn[7] = {
	0x0031u, 0x0032u, 0x0033u, 0x0035u, 0x0036u, 0x0037u, 0x0038u,
};

/*
 * 功能：报警灯模块占位初始化（灯控无本地状态）。
 * 交互：外部由 main 上电调用。
 */
void AlarmLight_Init(void)
{
}

/*
 * 功能：经 485 下发一条灯控寄存器写；失败映射为 ALARM_LIGHT_ERR_BUS。
 * 交互：内部被 AlarmLight_* 公共 API 调用；App485_Alarm_DirectFc06_NoResp。
 */
static uint8_t alarm_light_tx(uint16_t reg_value)
{
	uint8_t e = App485_Alarm_DirectFc06_NoResp(reg_value);

	if (e != MB_MASTER_OK)
	{
		/* 灯控模块不做应答校验：只要总线层报错就向上返回 */
		return ALARM_LIGHT_ERR_BUS;
	}
	return ALARM_LIGHT_OK;
}

/*
 * 功能：打开蜂鸣（设备码 0x0040）。
 * 交互：外部业务或组合色后补发。
 */
uint8_t AlarmLight_BuzzerOn(void)
{
	return alarm_light_tx(0x0040u);
}

/*
 * 功能：关闭蜂鸣（0x0041）。
 * 交互：外部恢复静音。
 */
uint8_t AlarmLight_BuzzerOff(void)
{
	return alarm_light_tx(0x0041u);
}

/*
 * 功能：关灯/全关图案（0x0060）。
 * 交互：外部 main 初始化、分拣完成清零等。
 */
uint8_t AlarmLight_Off(void)
{
	return alarm_light_tx(0x0060u);
}

/*
 * 功能：按颜色/闪烁模式/蜂鸣选项查表下发一条或多条寄存器写（必要时帧间延时）。
 * 交互：外部由 app_sort 故障、完成、SYS44 等调用；封装设备手册码字。
 */
uint8_t AlarmLight_Set(AlarmLightColor_t color, AlarmLightMode_t mode,
		       AlarmLightBuzzer_t buzzer)
{
	uint16_t cmd;
	uint8_t  e;

	if ((uint8_t)color > (uint8_t) ALARM_LIGHT_COLOR_BLUE)
	{
		return ALARM_LIGHT_ERR_PARAM;
	}

	if (mode == ALARM_LIGHT_MODE_OFF)
	{
		/* 关灯：忽略蜂鸣参数，一条寄存器写直达关机图案 */
		(void)buzzer;
		return alarm_light_tx(0x0060u);
	}

	switch (mode)
	{
	case ALARM_LIGHT_MODE_STEADY:
		if (buzzer == ALARM_LIGHT_BUZZER_ON)
		{
			if (color == ALARM_LIGHT_COLOR_RED)
			{
				/* 红灯常亮且自带蜂鸣的一体指令（设备手册定义） */
				e = alarm_light_tx(0x0014u);
				if (e != ALARM_LIGHT_OK)
				{
					return e;
				}
			}
			else
			{
				/* 其它颜色：先发纯色灯再补一条蜂鸣寄存器 */
				cmd = k_steady_no_horn[(unsigned int)color];
				e = alarm_light_tx(cmd);
				if (e != ALARM_LIGHT_OK)
				{
					return e;
				}
				Delay_ms(CFG_ALARM_INTER_FRAME_MS);
				e = alarm_light_tx(0x0040u);
				if (e != ALARM_LIGHT_OK)
				{
					return e;
				}
			}
		}
		else
		{
			cmd = k_steady_no_horn[(unsigned int)color];
			e = alarm_light_tx(cmd);
			if (e != ALARM_LIGHT_OK)
			{
				return e;
			}
		}
		break;

	case ALARM_LIGHT_MODE_SLOW:
		if (buzzer == ALARM_LIGHT_BUZZER_ON)
		{
			/* 灯手册：慢闪+蜂鸣组合码目前仅定义红色 */
			if (color != ALARM_LIGHT_COLOR_RED)
			{
				return ALARM_LIGHT_ERR_UNSUPPORTED;
			}
			cmd = 0x0024u;
		}
		else
		{
			cmd = k_slow_no_horn[(unsigned int)color];
		}
		e = alarm_light_tx(cmd);
		if (e != ALARM_LIGHT_OK)
		{
			return e;
		}
		break;

	case ALARM_LIGHT_MODE_FAST:
		if (buzzer == ALARM_LIGHT_BUZZER_ON)
		{
			if (color != ALARM_LIGHT_COLOR_RED)
			{
				return ALARM_LIGHT_ERR_UNSUPPORTED;
			}
			cmd = 0x0034u;
		}
		else
		{
			cmd = k_fast_no_horn[(unsigned int)color];
		}
		e = alarm_light_tx(cmd);
		if (e != ALARM_LIGHT_OK)
		{
			return e;
		}
		break;

	default:
		return ALARM_LIGHT_ERR_PARAM;
	}

	return ALARM_LIGHT_OK;
}
