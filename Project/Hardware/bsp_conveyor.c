/**
 * =============================================================================
 * bsp_conveyor.c — 传送带电磁阀光电 GPIO（板级驱动）
 * =============================================================================
 * 模块职责：配置并保持输出默认安全状态，提供继电器/晶体管驱动的 GPIO 拉高拉低接口。
 * 硬件资源：`global_config.h` 声明的 Valve / Motor / Photo 端口。
 * 调用入口：`BSP_Conveyor_Init()`，`BSP_Valve_*`，`BSP_Conveyor_SetMotor`，`BSP_Photo_IsTriggered`。
 * 被谁调用：`app_conveyor.c`、`app_arm.c`。
 * =============================================================================
 */
#include <stddef.h>
#include "bsp_conveyor.h"
#include "global_config.h"
#include "misc.h"

/*
 * 功能：配置推挽输出脚并默认拉低（安全态）。
 * 交互：内部被 BSP_Conveyor_Init 多次调用。
 */
static void gpio_out(GPIO_TypeDef *port, uint32_t clk, uint32_t pin)
{
	GPIO_InitTypeDef g;

	RCC_AHB1PeriphClockCmd(clk, ENABLE);

	g.GPIO_Pin = pin;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	/* 继电器/晶体管栅极默认下拉，防止上电浮空误动作 */
	g.GPIO_PuPd = GPIO_PuPd_DOWN;
	g.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(port, &g);

	GPIO_ResetBits(port, pin);
}

/*
 * 功能：配置上拉输入（光电、限位类）。
 * 交互：BSP_Conveyor_Init。
 */
static void gpio_in_pullup(GPIO_TypeDef *port, uint32_t clk, uint32_t pin)
{
	GPIO_InitTypeDef g;

	RCC_AHB1PeriphClockCmd(clk, ENABLE);
	g.GPIO_Pin = pin;
	g.GPIO_Mode = GPIO_Mode_IN;
	g.GPIO_PuPd = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(port, &g);
}

/*
 * 功能：初始化阀/带输出与光电输入等与安全相关的 GPIO。
 * 交互：main 先于 app_sort；app_conveyor/app_arm 只调 Set API。
 */
void BSP_Conveyor_Init(void)
{
	gpio_out(VALVE_ACTUATOR_Z_PORT, VALVE_ACTUATOR_Z_CLK, VALVE_ACTUATOR_Z_PIN);
	gpio_out(VALVE_GRIP_PORT, VALVE_GRIP_CLK, VALVE_GRIP_PIN);

	gpio_out(CONV_MOTOR_0_PORT, CONV_MOTOR_0_CLK, CONV_MOTOR_0_PIN);
	gpio_out(CONV_MOTOR_1_PORT, CONV_MOTOR_1_CLK, CONV_MOTOR_1_PIN);
	gpio_out(CONV_MOTOR_2_PORT, CONV_MOTOR_2_CLK, CONV_MOTOR_2_PIN);

	gpio_in_pullup(PHOTO_0_PORT, PHOTO_0_CLK, PHOTO_0_PIN);
	gpio_in_pullup(PHOTO_1_PORT, PHOTO_1_CLK, PHOTO_1_PIN);
	gpio_in_pullup(PHOTO_2_PORT, PHOTO_2_CLK, PHOTO_2_PIN);

	gpio_in_pullup(LIMIT_SOFT1_PORT, LIMIT_SOFT1_CLK, LIMIT_SOFT1_PIN);
	gpio_in_pullup(ESTOP_PORT, ESTOP_CLK, ESTOP_PIN);
}

/*
 * 功能：Z 向气缸电磁阀开关。
 * 交互：app_arm valve 序列。
 */
void BSP_Valve_SetZ(uint8_t on)
{
	if (on)
		GPIO_SetBits(VALVE_ACTUATOR_Z_PORT, VALVE_ACTUATOR_Z_PIN);
	else
		/* off：强制输出低，气动阀回到弹簧位 */
		GPIO_ResetBits(VALVE_ACTUATOR_Z_PORT, VALVE_ACTUATOR_Z_PIN);
}

/*
 * 功能：夹爪电磁阀开关。
 * 交互：app_arm valve 序列。
 */
void BSP_Valve_SetGrip(uint8_t on)
{
	if (on)
		GPIO_SetBits(VALVE_GRIP_PORT, VALVE_GRIP_PIN);
	else
		GPIO_ResetBits(VALVE_GRIP_PORT, VALVE_GRIP_PIN);
}

/*
 * 功能：控制三条传送带电机的 GPIO（继电器/晶体管）。
 * 交互：app_conveyor 阻塞环；BSP_Actuators_AllSafe。
 */
void BSP_Conveyor_SetMotor(uint8_t id, uint8_t on)
{
	GPIO_TypeDef *port = NULL;
	uint32_t     pin = 0u;

	switch (id)
	{
	case BSP_CONV_ID_0:
		port = CONV_MOTOR_0_PORT;
		pin = CONV_MOTOR_0_PIN;
		break;
	case BSP_CONV_ID_1:
		port = CONV_MOTOR_1_PORT;
		pin = CONV_MOTOR_1_PIN;
		break;
	default:
	case BSP_CONV_ID_2:
		/* 非法 id 回落到通道 2，避免 NULL 指针写 GPIO */
		port = CONV_MOTOR_2_PORT;
		pin = CONV_MOTOR_2_PIN;
		break;
	}

	if (on)
		GPIO_SetBits(port, pin);
	else
		GPIO_ResetBits(port, pin);
}

/*
 * 功能：读取对应传送带光电是否触发（低有效）。
 * 交互：app_conveyor 轮询停带。
 */
uint8_t BSP_Photo_IsTriggered(uint8_t id)
{
	GPIO_TypeDef *port = NULL;
	uint32_t      pin = 0u;

	switch (id)
	{
	default:
	case BSP_CONV_ID_0:
		port = PHOTO_0_PORT;
		pin = PHOTO_0_PIN;
		break;
	case BSP_CONV_ID_1:
		port = PHOTO_1_PORT;
		pin = PHOTO_1_PIN;
		break;
	case BSP_CONV_ID_2:
		port = PHOTO_2_PORT;
		pin = PHOTO_2_PIN;
		break;
	}

	/* 默认低有效：读到 0 认为触发 */
	return (GPIO_ReadInputDataBit(port, pin) == Bit_RESET) ? 1u : 0u;
}

/*
 * 功能：全部执行器回安全：阀关、带停。
 * 交互：急停、故障、sort_safe_stop_outputs、Init 后业务。
 */
void BSP_Actuators_AllSafe(void)
{
	BSP_Valve_SetZ(0u);
	BSP_Valve_SetGrip(0u);
	BSP_Conveyor_SetMotor(BSP_CONV_ID_0, 0u);
	BSP_Conveyor_SetMotor(BSP_CONV_ID_1, 0u);
	BSP_Conveyor_SetMotor(BSP_CONV_ID_2, 0u);
}
