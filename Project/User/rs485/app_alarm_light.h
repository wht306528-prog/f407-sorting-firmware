/**
 * app_alarm_light.h — 485 七色报警灯（Modbus RTU 功能码 06，寄存器 0x00C2，直接控制）
 *
 * 物理层：与电机伺服共用 USART2 PA2 PA3 与 PC0 DE，115200 8N1；
 * - 伺服：使用 `modbus_master` 的 FC03/06/10 封装（内部已是 `TxnExec` + 重试 + 历史帧）。
 * - 报警灯：仅调用 `App485_Alarm_DirectFc06_NoResp`（只发不收应答）。
 * @note KEY2 手写帧、调试工具使用 `ModbusMaster_SendRawBlocking`，同样经过总线互斥。
 *
 * 通信策略示例：只发送 ADU（本工程不阻塞等待灯体应答，以减轻主循环负担——请按现场灯体协议决定是否加读回）。
 * 协议依据：`TEST/485新/485报警灯使用说明书改版.pdf`
 */
#ifndef APP_ALARM_LIGHT_H
#define APP_ALARM_LIGHT_H

#include <stdint.h>

#define ALARM_LIGHT_OK               0u
#define ALARM_LIGHT_ERR_PARAM        1u
#define ALARM_LIGHT_ERR_UNSUPPORTED  2u /* 说明书不允许的组合（如非红灯闪烁时开喇叭） */
#define ALARM_LIGHT_ERR_BUS          3u

/** 颜色顺序与说明书「直接控制」数值递增一致（红→黄→绿→白→青→紫→蓝） */
typedef enum {
	ALARM_LIGHT_COLOR_RED = 0,
	ALARM_LIGHT_COLOR_YELLOW,
	ALARM_LIGHT_COLOR_GREEN,
	ALARM_LIGHT_COLOR_WHITE,
	ALARM_LIGHT_COLOR_CYAN,
	ALARM_LIGHT_COLOR_PURPLE,
	ALARM_LIGHT_COLOR_BLUE,
} AlarmLightColor_t;

typedef enum {
	ALARM_LIGHT_MODE_STEADY = 0, /* 常亮 */
	ALARM_LIGHT_MODE_SLOW,       /* 慢闪 */
	ALARM_LIGHT_MODE_FAST,       /* 快闪 */
	ALARM_LIGHT_MODE_OFF,        /* 关灯关喇叭（写 0x60） */
} AlarmLightMode_t;

typedef enum {
	ALARM_LIGHT_BUZZER_OFF = 0,
	ALARM_LIGHT_BUZZER_ON,
} AlarmLightBuzzer_t;

void AlarmLight_Init(void);

/**
 * @brief 设置灯光颜色、闪烁模式及喇叭请求。
 * @note 说明书约束：喇叭主要在「常亮」下配合；快闪/慢闪仅「红灯」可提供带喇叭指令，
 *       其它颜色在闪烁模式下请求喇叭将返回 ALARM_LIGHT_ERR_UNSUPPORTED。
 *       非红灯「常亮 + 喇叭开」将先发颜色帧再发 0x40（喇叭开）。
 */
uint8_t AlarmLight_Set(AlarmLightColor_t color, AlarmLightMode_t mode,
		       AlarmLightBuzzer_t buzzer);

/** 单独喇叭开（寄存器值 0x0040） */
uint8_t AlarmLight_BuzzerOn(void);

/** 单独喇叭关（寄存器值 0x0041） */
uint8_t AlarmLight_BuzzerOff(void);

/** 关灯且喇叭关（0x0060，说明书亦列出 0x0061，二者等价用途） */
uint8_t AlarmLight_Off(void);

#endif /* APP_ALARM_LIGHT_H */
