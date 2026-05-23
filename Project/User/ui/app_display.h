/**

 * app_display.h — 液晶屏上「看得见的一切」都在这里汇总

 *

 * 【零基础说明】

 * - `AppDisplay_Refresh`：主循环末尾调用；含主页 / Raw / Final / 串口诊断页（见 `AppDisplayPage_te`）。

 * - `AppDisplay_SetFaultText`：主页 Alarm 行。

 * - `AppDisplay_SetRunFlagText`：主页 RunFlag 行。

 * - `AppDisplay_SetServoBriefText`：主页 Servo 行（Modbus Tx/Rx 摘要）。

 * - 触摸：`Touch_Button_Down/Up` — 主页底栏 SER | MAIN | FIN（含 Raw 带半屏翻页）；子页 Up | Back | Dn。

 *

 * `APP_DISPLAY_PAGE_DEBUG`：兼容旧枚举，等同于 MAIN。

 */

#ifndef __APP_DISPLAY_H__

#define __APP_DISPLAY_H__



#include <stdint.h>



typedef enum

{

	APP_IND_IDLE = 0,

	APP_IND_ARM_RUN,

	APP_IND_PAUSE_BLINK,

	APP_IND_OK_HOLD,

	APP_IND_FAIL_HOLD,

	APP_IND_FAIL_BLINK

} AppIndicatorState_t;



typedef enum

{

	APP_DISPLAY_PAGE_MAIN = 0,

	APP_DISPLAY_PAGE_RAW_MATRIX,

	APP_DISPLAY_PAGE_FINAL_MATRIX,

	APP_DISPLAY_PAGE_SERIAL_DIAG,

	APP_DISPLAY_PAGE_DEBUG

} AppDisplayPage_te;



void AppDisplay_Init(void);

/** KEY2 矩阵轮开始前：丢弃 USART3 HEX 拼行半成品，避免与上轮粘连 */
void AppDisplay_ResetU3HexScrollAccum(void);



void AppIndicator_SetState(AppIndicatorState_t st);



/** 切换 LCD 页面（DEBUG 映射为 MAIN） */

void AppDisplay_SetPage(AppDisplayPage_te pg);

AppDisplayPage_te AppDisplay_GetPage(void);



/** 由 `AppSort_Poll` 调用：主页 Servo 行 */

void AppDisplay_SetServoBriefText(const char *main_state, const char *step_state);



void AppDisplay_Refresh(uint32_t tick_ms);

/** 触摸按下后尽快刷新底栏按钮高亮（不阻塞主循环） */
void AppDisplay_KickRefresh(void);



void AppDisplay_SetFaultText(const char *ascii);



void AppDisplay_SetRunFlagText(const char *ascii);



/**

 * Modbus 层诊断：`dir` 为 TX/RX；`fc` 为功能码（0x10 写多寄存器）；`byte_len` 为整帧 ADU 字节数（不是寄存器个数）。

 */

void AppDisplay_LogRs485Summary(uint8_t is_tx, uint8_t slave, uint8_t fc,

				uint16_t byte_len, uint8_t err_or_ok);



void AppDisplay_LogModbusBusy(void);



#endif

