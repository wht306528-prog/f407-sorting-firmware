/**
 * app_arm.h — 机械臂取放编排（读 Matrix + Modbus 伺服 + IO 阀序）
 *
 * 与 app_sort 配合：先由矩阵选出源/目的行索引，再经本模块完成阻塞式 PickPlace。
 */
#ifndef __APP_ARM_H__
#define __APP_ARM_H__

#include <stdint.h>

/**
 * 回基点（MCU 零点脉冲）：AppMotor_GotoPulse(CALIB_JOINT*_ZERO_PULSE)。
 * 被谁调用：`app_sort` 标志 44/45、传送带 FL4/5/6 前、`run_conveyor_flag` 内回避。
 */
void AppArm_GoHome(void);

/**
 * 按 Matrix 行索引完成一次取放（阻塞 Modbus + 阀序）。
 * @param src_idx 矩阵表 s_tbl 中行索引：穴位须有非 0 class_id
 * @param dst_idx 目标行索引：穴位须为 class_id==0（空穴）
 * @return 0 成功并已 `AppMatrix_ApplyTransfer`；非 0 失败（电机/索引）
 */
uint8_t AppArm_PickPlace(uint16_t src_idx, uint16_t dst_idx);

/* PickPlace 返回码短文本，供屏显与日志；未知码返回 "?" */
const char *AppArm_ResultText(uint8_t code);

#endif
