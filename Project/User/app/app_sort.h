#ifndef __APP_SORT_H__
#define __APP_SORT_H__

#include <stdint.h>

/**
 * app_sort.h — 分拣编排对外接口
 *
 * 读代码顺序建议：
 * 1. AppSortMainState_e：大状态 IDLE/RUN/PAUSE/FAULT/ESTOP
 * 2. AppSortStep_e：RUN 内 WAIT_MATRIX 或 SORTING
 * 3. AppSortSysFlag_e：屏上 SYS 数值，仅 RUN+SORTING 推进时有意义
 *
 * 系统运行标志速查：
 * - 0：空闲或等矩阵
 * - 2~6：二类、一类、三条传送带子流程（细节见 app_sort.c）
 * - 44：缺盘等告警快闪，时段结束回 IDLE；与 ESTOP 软件闭锁（需 RequestClearFault）不同
 * - 45：待机 AppArm_GoHome
 *
 * 人机：触摸以矩阵分页为主；启停见上位机。RequestEstop 保留 API（板载 KEY2 不发急停；入口待接上位机/触摸等）。
 */

typedef enum
{
	APP_SORT_MAIN_IDLE = 0,
	APP_SORT_MAIN_RUN,
	APP_SORT_MAIN_PAUSE,
	APP_SORT_MAIN_FAULT,
	APP_SORT_MAIN_ESTOP,
} AppSortMainState_e;

/**
 * 主编排内部节拍（比触摸屏的 RUN 更细一层）。
 * - APP_SORT_STEP_WAIT_MATRIX：等矩阵 Modbus（KEY2）落盘 + 几何/IK 就绪。
 * - SORTING：执行 FL2/3… 搬东西；Pause 时可卡在这一层里冻结推进。
 */
typedef enum
{
	APP_SORT_STEP_WAIT_MATRIX = 0,
	APP_SORT_STEP_SORTING,
} AppSortStep_e;

/**
 * 业务「系统运行标志」枚举（数值与旧版 hex 协议一致，不要随意改数字）。
 * 屏上 `SYS:` 后显示的就是这里的十进制值；`AppSort_GetSysRunFlagText` 给缩写（FL2、F44…）。
 */
typedef enum
{
	APP_SORT_SYS_IDLE = 0,
	APP_SORT_SYS_FL2 = 2,
	APP_SORT_SYS_FL3 = 3,
	APP_SORT_SYS_FL4 = 4,
	APP_SORT_SYS_FL5 = 5,
	APP_SORT_SYS_FL6 = 6,
	APP_SORT_SYS_FL44 = 44,
	APP_SORT_SYS_FL45 = 45,
} AppSortSysFlag_e;

void AppSort_Init(void);

void AppSort_Poll(uint32_t tick_ms);

/** 开始进入 RUN（从 IDLE/FAULT）或 PAUSE→RUN */
void AppSort_RequestStart(void);

/** 运行中切换到 PAUSE：冻结系统标志机，不关矩阵缓存 */
void AppSort_RequestPause(void);

/** 清除故障/急停闭锁，回到 IDLE */
void AppSort_RequestClearFault(void);

/** 软件急停（闭锁 + Modbus 急停寄存器）；当前不由板载 KEY2 直接调用 */
void AppSort_RequestEstop(void);

/**
 * 待机回位（系统运行标志 45）：主循环下一次 Poll 内执行 AppArm_GoHome，
 * 适用于「人确认后停在小鸡位」场景；不与 ESTOP 抢占混用。
 */
void AppSort_RequestStandbyHome(void);

uint8_t AppSort_IsEstopLatched(void);

AppSortMainState_e AppSort_GetMainState(void);
AppSortStep_e      AppSort_GetStep(void);

/** 触摸屏一行缩写：IDLE / RUN / … */
const char *AppSort_GetMainStateText(void);

/**
 * 系统运行标志位（业务层）：0 空闲；2/3/4/5/6 流程；44 缺盘/故障提示；45 待机回位。
 * 仅 RUN+SORTING 时非 0 有意义；FAULT/ESTOP 时读回可能为 0。
 */
uint8_t AppSort_GetSysRunFlag(void);

/** 短文本：FL2 / F44 / H45 等，便于 LCD 窄行显示 */
const char *AppSort_GetSysRunFlagText(void);

/**
 * FL2/FL3 执行中是否应丢弃 Matrix_Raw 字节流（OnStream），避免 ASCII 路径改写快照。
 * 量产矩阵以 USART3 Modbus 为主；本规则仍适用于调试注入。
 *
 * 规则：`RUN + SORTING` 且系统运行标志为 FL2 或 FL3 时返回 1，否则 0。
 */
uint8_t AppSort_ShouldDiscardTcpMatrixStream(void);

/** ESTOP 闭锁期间最近一次读取缓存的两轴 P0B-07 绝对脉冲（空指针忽略该项） */
void AppSort_GetEstopMonitoredPositions(int32_t *m1, int32_t *m2);

#endif
