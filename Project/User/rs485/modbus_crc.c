/*
 * modbus_crc.c：Modbus RTU CRC16 独立实现（与工程内其它模块解耦便于测试复用）。
 *
 * 功能：标准 CRC 计算、校验、向 ADU 尾部写入 CRC。
 * 交互：被 modbus_master、app_protocol、app_demo 等通过 modbus_crc.h 调用。
 */
#include "modbus_crc.h"
#include <stddef.h>

/*
 * 功能：计算 Modbus RTU 标准 CRC16（多项式 0xA001，初值 0xFFFF）。
 * 交互：被 Modbus_CheckCRC()、Modbus_AppendCRC()、app_protocol.c、app_demo.c 等调用。
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFFu;
	uint16_t i;

	for (i = 0; i < len; i++)
	{
		uint8_t j;

		crc ^= data[i];

		/* 逐 bit 处理：本质是把多项式除法硬件化成移位异或；若最低位为 1，右移后还要异或生成多项式 */
		for (j = 0; j < 8u; j++)
		{
			if (crc & 1u)
			{
				crc = (uint16_t)((crc >> 1) ^ 0xA001u);
			}
			else
			{
				crc >>= 1;
			}
		}
	}
	return crc;
}

/*
 * 功能：校验 ADU 末尾两字节 CRC（低字节在前）是否与前面 PDU 一致。
 * 交互：内部调用 Modbus_CRC16()；被 modbus_master.c、app_protocol.c（经包装）等调用。
 */
uint8_t Modbus_CheckCRC(const uint8_t *frame, uint16_t len_total)
{
	uint16_t crc_calc;
	uint16_t crc_rx;

	/* 太短不可能包含「至少 1 字节数据 + 2 字节 CRC」，直接视为格式非法 */
	if ((frame == NULL) || (len_total < 4u))
	{
		return 0u;
	}

	crc_calc = Modbus_CRC16(frame, (uint16_t)(len_total - 2u));

	/* RTU 规定 CRC 先发低字节再发高字节；这里按字节序还原成 uint16 再比较 */
	crc_rx = (uint16_t)frame[len_total - 2u] |
		 ((uint16_t)frame[len_total - 1u] << 8);

	return (crc_calc == crc_rx) ? 1u : 0u;
}

/*
 * 功能：在 adu[pdu_len]、adu[pdu_len+1] 写入 CRC（低字节在前）。
 * 交互：调用 Modbus_CRC16()；被 modbus_master.c 等发送路径调用。
 */
void Modbus_AppendCRC(uint8_t *adu, uint16_t pdu_len)
{
	uint16_t crc;

	if (adu == NULL)
	{
		return;
	}

	crc = Modbus_CRC16(adu, pdu_len);
	adu[pdu_len] = (uint8_t)(crc & 0xFFu);
	adu[pdu_len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
}
