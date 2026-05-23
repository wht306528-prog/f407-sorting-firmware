/**
 * demo_modbus_frames.h — KEY2 板载按键用的「预制 Modbus RTU 字节条」
 *
 * 【职责】main.c Pop KEY2 后直接 `ModbusMaster_SendRawBlocking()`；完整 ADU 须含 CRC。
 * 【改了什么】任一 payload 字节变化 → Modbus CRC16（多项式 0xA001）重算末尾 2 字节。
 * 【约定】缓冲区 = 站地址 + FC + PDU + CRC 低字节 + CRC 高字节。
 *
 * KEY1：`AppArm_GoHome()`（两轴 Modbus），不使用本缓冲。
 */
#ifndef DEMO_MODBUS_FRAMES_H
#define DEMO_MODBUS_FRAMES_H

#include <stdint.h>

/**
 * KEY2：`ModbusMaster_SendRawBlocking(...)` 整块下发（已含 CRC）。
 * 固定：`03 06 00 C2 00 11` + CRC=E9 D8（先发低字节 0xE9）。
 * 语义：站号 3，FC06，寄存器 0x00C2，写入值 0x0011（与 CFG_ALARM_REG_DIRECT / 报警灯演示写入对齐）。
 */
static const uint8_t g_demo_key2_modbus_pdu[] = {
	0x03u, 0x06u, 0x00u, 0xC2u, 0x00u, 0x11u, 0xE9u, 0xD8u
};

#endif /* DEMO_MODBUS_FRAMES_H */
