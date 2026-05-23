/**
 * delay.c — SysTick 1ms 节拍与阻塞毫秒延时
 *
 * 【用途】业务状态机、Modbus 超时、`Delay_ms` 软等待。
 * 【注意】长时间 `Delay_ms` 会阻塞整个裸机主循环；传送带/屏幕刷新在此期间停顿，
 * 后续若要彻底消除卡顿需改成「返回式」状态机而不是拉长延时。
 */
#include "delay.h"
#include "stm32f4xx.h"

static volatile uint32_t s_tick;

/*
 * 功能：配置 SysTick 为 1ms 节拍并让 s_tick 与 SysTick_Handler 对齐。
 * 交互：外部 main 最先调用之一；失败后死循环。
 */
void Delay_Init(void)
{
	s_tick = 0;
	if (SysTick_Config(SystemCoreClock / 1000u))
	{
		/* SysTick 寄存器配置失败（极少见）：停在中断无法前进，提示检查时钟/HAL */
		while (1) {}
	}
}

/*
 * 功能：SysTick 中断回调：毫秒计数递增（与 stm32f4xx_it SysTick_Handler 对接）。
 * 交互：内核每 1ms 调用。
 */
void Delay_Inc(void)
{
	s_tick++;
}

/*
 * 功能：读取当前毫秒 tick（别名 SysTick_GetMs）。
 * 交互：超时、节拍轮询广泛使用。
 */
uint32_t Delay_GetTick(void)
{
	return s_tick;
}

/*
 * 功能：同 Delay_GetTick，供新老代码统一命名。
 * 交互：modbus_arm_conveyor_sort 等。
 */
uint32_t SysTick_GetMs(void)
{
	return s_tick;
}

/*
 * 功能：阻塞忙等毫秒数（饿死主循环）。
 * 交互：BSP 节拍、短时 Modbus 间隔等。
 */
void Delay_ms(uint32_t ms)
{
	uint32_t start = Delay_GetTick();
	/* 忙等：对比 SysTick 计数溢出环绕仍成立（32-bit 差值在无符号减法下正确） */
	while ((Delay_GetTick() - start) < ms)
	{
	}
}
