/**
 * stm32f4xx_it.c — Cortex-M4 中断服务程序（本工程裁掉了用不到的向量）
 *
 * 【给零基础：中断是什么】
 * 按键按下去那一刻 CPU 会“打断”主程序，先跑进下面的 `EXTI0_IRQHandler`；
 * 这里只做**极短**的事（清标志、记一下“键按过了”），然后回到主程序继续跑 for 循环。
 *
 * 【为什么不在中断里发 RS485】
 * 发一串 Modbus 要好久，若占着中断，触摸屏扫描、双路 RS485、SysTick 都会被耽误，甚至丢字节。
 * 所以 KEY1：`EXTI0` 只置 `s_key1_isr_hit`，真正 `AppArm_GoHome`（RS485）在 main。
 *
 * 【按键脚】KEY1=PA0→EXTI0；KEY2=PC13→EXTI10~15 共用的 `EXTI15_10` 向量（库里这么分组的）。
 */
#include "stm32f4xx_it.h"
#include "delay.h"
#include "stm32f4xx_exti.h"

/** 供调试/故障屏：累加各类致命异常（上电清零；HardFault 后只能维持到复位） */
volatile uint32_t g_arm_core_fault_accum;

static volatile uint8_t s_key1_isr_hit;
static volatile uint32_t s_key1_isr_ms;
static volatile uint8_t s_key2_isr_hit;

/*
 * 功能：CPU NMI 向量占位。
 * 交互：内核。
 */
void NMI_Handler(void)
{
}

/*
 * 功能：硬fault：累加调试标志并死循环停机。
 * 交互：MCU 硬件异常路径。
 */
void HardFault_Handler(void)
{
	/* 典型原因：空指针解引用、栈溢出、非法指令；调试时可查看 CFSR/BFAR */
	g_arm_core_fault_accum |= 1u;
	while (1)
	{
	}
}

/*
 * 功能：MPU/访问违例 Manage fault。
 * 交互：MCU 异常。
 */
void MemManage_Handler(void)
{
	g_arm_core_fault_accum |= 2u;
	while (1)
	{
	}
}

/*
 * 功能：总线 Fault。
 * 交互：MCU 异常。
 */
void BusFault_Handler(void)
{
	g_arm_core_fault_accum |= 4u;
	while (1)
	{
	}
}

/*
 * 功能：用法 Fault（对齐、未定义指令等）。
 * 交互：MCU 异常。
 */
void UsageFault_Handler(void)
{
	g_arm_core_fault_accum |= 8u;
	while (1)
	{
	}
}

/*
 * 功能：调试监视器占位。
 * 交互：调试器触发。
 */
void DebugMon_Handler(void)
{
}

/*
 * 功能：1ms SysTick → Delay_Inc 毫秒计数。
 * 交互：与 Delay_Init 配套。
 */
void SysTick_Handler(void)
{
	Delay_Inc();
}

/*
 * 功能：KEY1 下降沿 EXTI0：清标志并锁存按下时刻戳。
 * 交互：main EXTI_PopKey1 消费；勿在此发 485。
 */
void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		/* 只做「记录按下时刻」，耗时通讯放到 main 轮询 */
		s_key1_isr_hit = 1u;
		s_key1_isr_ms = SysTick_GetMs();
	}
}

/*
 * 功能：KEY2 EXTI13：清中断并锁存 hit（与 EXTI15_10 向量）。
 * 交互：main Pop；AppMotionAbort_Peek。
 */
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13);
		s_key2_isr_hit = 1u;
	}
}

/*
 * 功能：弹出 KEY1 ISR 命中一次（可读按下 ms）。
 * 交互：main 主循环演示 KEY1。
 */
uint8_t EXTI_PopKey1(uint32_t *press_ms)
{
	if (!s_key1_isr_hit)
	{
		return 0u;
	}
	s_key1_isr_hit = 0u;
	if (press_ms)
	{
		/* 可选带回按下时刻，便于长按/双击扩展 */
		*press_ms = s_key1_isr_ms;
	}
	return 1u;
}

/*
 * 功能：弹出 KEY2 命中一次。
 * 交互：main KEY2。
 */
uint8_t EXTI_PopKey2(void)
{
	if (!s_key2_isr_hit)
	{
		return 0u;
	}
	s_key2_isr_hit = 0u;
	return 1u;
}

/*
 * 功能：窥视 KEY2 是否仍处于命中未 Pop（当前无调用方；保留扩展）。
 */
uint8_t EXTI_Key2HitPeek(void)
{
	return s_key2_isr_hit ? 1u : 0u;
}
