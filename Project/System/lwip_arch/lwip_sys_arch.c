/**
 * =============================================================================
 * 模块职责：
 *   为 lwIP（NO_SYS=1 裸机）提供最小的系统时间与时钟滴答实现。
 * =============================================================================
 * 硬件资源：
 *   依赖 SysTick 产生的毫秒计数（见 delay.c）。
 * =============================================================================
 * 调用入口：
 *   由 lwIP TCP/IP 核心在运行期隐式调用 `sys_now()` / `sys_jiffies()`。
 * =============================================================================
 * 被谁调用：
 *   lwIP：`tcp_out.c`、`init.c` 等。
 * =============================================================================
 */
#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/timers.h"

#include "delay.h"

#if LWIP_TCP && !LWIP_TIMERS
#include "lwip/tcp_impl.h"

/*
 * NO_SYS + NO_SYS_NO_TIMERS 时 lwIP 不编译 timers.c，但 tcp.c 仍会经 TCP_REG 调用本函数。
 * 应用层已由 LwIP_Periodic_Handle() 周期性调用 tcp_tmr()，此处为空即可。
 */

/*
 * 功能：lwIP TCP 定时器注册占位（tcp_tmr 由应用周期调用）。
 * 交互：lwIP 核心。
 */
void tcp_timer_needed(void)
{
}

#endif

/*
 * 功能：lwIP sys 适配初始化（裸机 noop）。
 * 交互：lwip_init 路径。
 */
void sys_init(void)
{
}

/*
 * 功能：lwIP 取当前毫秒时间戳。
 * 交互：超时与重传；映射 Delay_GetTick。
 */
u32_t sys_now(void)
{
	return (u32_t)Delay_GetTick();
}

/*
 * 功能：与 sys_now 同源的 jiffies（本工程等价毫秒）。
 * 交互：lwIP 内部节拍。
 */
u32_t sys_jiffies(void)
{
	return (u32_t)Delay_GetTick();
}
