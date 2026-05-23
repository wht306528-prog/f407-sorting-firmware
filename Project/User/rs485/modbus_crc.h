/**
 * modbus_crc.h — Modbus RTU CRC16（多项式 0xA001，初始 0xFFFF，低字节在前）
 * 供主站/从站及用户测试程序统一调用，与 app_protocol 内算法一致。
 */
#ifndef MODBUS_CRC_H
#define MODBUS_CRC_H

#include <stdint.h>

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

/** frame 总长度 len_total，校验含最后 2 字节 CRC */
uint8_t Modbus_CheckCRC(const uint8_t *frame, uint16_t len_total);

/**
 * adu 前 pdu_len 字节为 PDU（无 CRC），在本函数内写入 CRC 至 adu[pdu_len]、adu[pdu_len+1]
 */
void Modbus_AppendCRC(uint8_t *adu, uint16_t pdu_len);

#endif /* MODBUS_CRC_H */
