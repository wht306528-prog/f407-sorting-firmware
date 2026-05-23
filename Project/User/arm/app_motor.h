/**
 * app_motor.h — 两轴伺服 Modbus RTU 封装（寄存器细节见 global_config.h / F407/docs/F407_维护说明.txt）
 *
 * 点到点：写 P10-14 绝对脉冲（低字寄存器在前）→ 使能 → P0D-08=3 触发绝对移动 → WaitPosition（P0B-04.Bit0 或位置误差）
 * → P0D-08=0；Wait 内轮询急停中止。
 */
#ifndef __APP_MOTOR_H__
#define __APP_MOTOR_H__

#include <stdint.h>

/** 关节角(deg) → 驱动器脉冲（比例与方向在 app_motor.c / 宏中） */
int32_t AppMotor_ServoAngleToPulse(float joint_angle_deg);

uint8_t AppMotor_WriteTargetPulse(uint8_t slave, int32_t pulse);

uint8_t AppMotor_Enable(uint8_t slave);

uint8_t AppMotor_Disable(uint8_t slave);

/** 急停模式：FC06 写 SERVO_REG_ESTOP_MODE（P0D-05），on≠0 为急停 ON */
uint8_t AppMotor_SetEmergencyMode(uint8_t slave, uint8_t on);

uint8_t AppMotor_TriggerAbsMove(uint8_t slave);

/** 清除总线运动触发（P0D-08=0）；与 TriggerReset 等价 */
uint8_t AppMotor_ClearTrigger(uint8_t slave);

uint8_t AppMotor_TriggerRun(uint8_t slave);

uint8_t AppMotor_TriggerReset(uint8_t slave);

/** FC03 读 SERVO_REG_REACH_STATUS，Bit0→reached_bit0（0/1） */
uint8_t AppMotor_ReadReachedStatus(uint8_t slave, uint8_t *reached_bit0);

uint8_t AppMotor_WaitPosition(uint8_t slave, int32_t target_pulse, uint32_t timeout_ms);

/** 读当前反馈 32bit 有符号脉冲（FC03 SERVO_REG_FB_POS32_START，低字在前拼 32 位） */
uint8_t AppMotor_ReadFeedbackPosition32(uint8_t slave, int32_t *out_pos);

/** 写目标+使能+ABS 触发+等待+（等待成功/超时/中止时清触发） */
uint8_t AppMotor_GotoPulse(uint8_t slave, int32_t pulse);

/**
 * 先到绝对目标脉冲：先读反馈，算 Δ；再按 CFG_SERVO_POS_CMD_IS_REL_DELTA 写绝对终点或 Δ；
 * Write→Enable→TriggerAbsMove→Wait(feedback≈target_abs)。
 */
uint8_t AppMotor_GotoAbsTargetAsRelative(uint8_t slave, int32_t target_abs);

#endif
