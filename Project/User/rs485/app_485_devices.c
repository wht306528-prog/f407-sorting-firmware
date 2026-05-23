/**
 * app_485_devices.c — 报警灯等 Modbus 从站的设备层封装
 *
 * 与 app_alarm_light 分界：此处只拼接 PDU flags 并调用 ModbusMaster_TxnExec；
 * 灯色字面量与业务时机在 alarm_light。
 */

#include "app_485_devices.h"
#include "modbus_master.h"
#include "global_config.h"

#include <string.h>

/*
 * 功能：向报警灯从站发 FC06 写直通寄存器（PDU 大端），且不等待解析应答帧。
 * 交互：外部由 app_alarm_light.alarm_light_tx 调用；占用 ModbusMaster_TxnExec 互斥。
 */
uint8_t App485_Alarm_DirectFc06_NoResp(uint16_t value_for_direct_register)
{
	ModbusTxnRequest_t rq;
	ModbusTxnResult_t rs;
	uint8_t            pdu[4];

	memset(&rq, 0, sizeof(rq));
	memset(&rs, 0, sizeof(rs));

	/* FC06 PDU：寄存器地址大端 + 写入值大端；寄存器号来自 global_config 避免散落魔术数字 */
	pdu[0] = (uint8_t)((CFG_ALARM_REG_DIRECT >> 8) & 0xFFu);
	pdu[1] = (uint8_t)(CFG_ALARM_REG_DIRECT & 0xFFu);
	pdu[2] = (uint8_t)((value_for_direct_register >> 8) & 0xFFu);
	pdu[3] = (uint8_t)(value_for_direct_register & 0xFFu);

	rq.slave = (uint8_t)CFG_ALARM_MODBUS_SLAVE;
	rq.fc = 0x06u;
	rq.payload = pdu;
	rq.payload_len = 4u;
	rq.rsp_timeout_ms = CFG_MODBUS_RSP_TIMEOUT_MS;

	/*
	 * NO_RESPONSE：报警灯有些场景不写应答解析，减少等待；
	 * 但依然走 TxnExec + mutex，避免和伺服事务穿插打断帧。
	 */
	rq.flags = MB_MODBUS_TXN_FLAG_NO_RESPONSE;

	return ModbusMaster_TxnExec(&rq, &rs);
}
