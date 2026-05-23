/**
 * modbus_master.h — RS485 Modbus RTU 主机侧 API
 *
 * ADU：一整帧串口字节（站号、FC、数据、CRC）。
 * MB_MASTER_ERR_BUSY(7)：互斥未抢到；屏上简写 BSY。
 * MB_MASTER_ERR_TIMEOUT(1)：规定时间内未收齐应答。
 *
 * 物理 DE 仅由 bsp_uart.c 切换；业务请走 ModbusMaster_TxnExec 或 FC03/06/10 封装，勿旁路直写 UART。
 */
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <stdint.h>

#define MB_MASTER_OK           0u
#define MB_MASTER_ERR_TIMEOUT  1u
#define MB_MASTER_ERR_CRC      2u
#define MB_MASTER_ERR_SLAVE    3u
#define MB_MASTER_ERR_FC       4u
#define MB_MASTER_ERR_BUF      5u
#define MB_MASTER_ERR_ARG      6u
#define MB_MASTER_ERR_BUSY     7u
#define MB_MASTER_ERR_EXCEPTION 8u
#define MB_MASTER_ERR_LEN       9u
#define MB_MASTER_ERR_ABORT     10u /*!< 运动被急停闭锁等长路径提前打断（非从站协议错误） */

#ifndef MB_MUTEX_WAIT_MS
#define MB_MUTEX_WAIT_MS 200u
#endif

/** @deprecated 实际静默间隔见 `modbus_master.c` 中 `mb_rtu_gap_ms()` */
#ifndef MB_RTU_FRAME_GAP_MS
#define MB_RTU_FRAME_GAP_MS 3u
#endif

#define MB_MODBUS_TXN_FLAG_NO_RESPONSE   0x01u /*!< 仅发不写从站寄存器应答等待（报警灯单向写）*/
/** RAW ADU：`raw_complete[len]`==站+FC+PDU+CRC(lo-hi)，调用方要自己算 CRC；见 `txn_once_build_tx` */
#define MB_MODBUS_TXN_FLAG_RAW_ADU       0x02u
#define MB_MODBUS_TXN_FLAG_NO_RETRY      0x04u /*!< 禁止自动重试（一般不用）*/

#define MB_MASTER_MAX_PAYLOAD 251u
#define MB_MASTER_MAX_ADU    268u /*!< addr+fc+pdu + crc2 */

#define MB_HIST_DEPTH 8u

typedef struct ModbusTxnRequest
{
	uint8_t  slave;
	uint8_t  fc;
	const uint8_t *payload; /*!< 非 RAW 时为 PDU 载荷（不含 slave/fc）；RAW 时可为 NULL */
	uint16_t payload_len;
	uint32_t rsp_timeout_ms;
	uint8_t  flags;

	/** RAW 模式：完整 ADU 已含 CRC；同时置 `MB_MODBUS_TXN_FLAG_RAW_ADU` */
	const uint8_t *raw_complete;
	uint16_t        raw_complete_len;

	/** RS485 事务互斥等待超时(ms)，0 表示使用 `MB_MUTEX_WAIT_MS` */
	uint32_t mutex_wait_ms;
} ModbusTxnRequest_t;

typedef struct ModbusTxnParsed
{
	uint8_t  slave;
	uint8_t  fc_echo;
	uint8_t  is_exception;
	uint8_t  exception_code;
	uint8_t  data[251];
	uint16_t data_len;
} ModbusTxnParsed_t;

typedef struct ModbusTxnResult
{
	uint8_t  err;
	uint8_t  attempts_used;
	uint8_t  retries_done;
	uint16_t rx_len;
	uint16_t tx_len;
	uint8_t  rx_raw[MB_MASTER_MAX_ADU];
	uint8_t  tx_raw[MB_MASTER_MAX_ADU];
	ModbusTxnParsed_t parsed;
} ModbusTxnResult_t;

typedef struct ModbusHistEntry
{
	uint32_t ms_tick;
	uint16_t tx_len;
	uint16_t rx_len;
	uint8_t  final_err;
	uint8_t  attempts;
	uint8_t  tx[MB_MASTER_MAX_ADU];
	uint8_t  rx[MB_MASTER_MAX_ADU];
} ModbusHistEntry_t;

uint8_t ModbusMaster_ParseResponse(const uint8_t *rx, uint16_t rx_len, uint8_t req_fc,
				  ModbusTxnParsed_t *out);

uint8_t ModbusMaster_TxnExec(const ModbusTxnRequest_t *rq, ModbusTxnResult_t *rs);

/** 最近一次事务结果快照（供触摸屏显示伺服/总线诊断）*/
const ModbusTxnResult_t *ModbusMaster_GetLastResult(void);

void ModbusMaster_HistGet(uint16_t idx_from_newest0, ModbusHistEntry_t *out);
uint16_t ModbusMaster_HistCount(void);
void ModbusMaster_FormatHistLine(uint16_t idx_from_newest0, char *dst, unsigned cap);

/** 简短错误标签（LCD），与 `MB_MASTER_ERR_*` 对应 */
const char *ModbusMaster_ErrTag(uint8_t err);

/**
 * 手写 **完整 RTU 帧**：`len`≥4，最后两字节必须为 Modbus CRC16（先行低字节），走互斥排队。
 * 【失败路径】仍可查 `ModbusMaster_GetLastResult()`；KEY2 手写 RAW 见 demo_modbus_frames.h。
 */
uint8_t ModbusMaster_SendRawBlocking(const uint8_t *data, uint16_t len,
				    uint32_t mutex_wait_ms);

/** 即发不收 RAW ADU：`rsp_timeout=0` & `TXN_FLAG_NO_RESPONSE`，禁止等待回包 */
uint8_t ModbusMaster_SendRawNoResponse(const uint8_t *data, uint16_t len,
				      uint32_t mutex_wait_ms);

uint16_t ModbusMaster_GetLastTxAdu(uint8_t *out, uint16_t cap);

uint8_t ModbusMaster_WriteSingleRegister(uint8_t slave, uint16_t reg,
					 uint16_t value, uint8_t *rx,
					 uint16_t rx_cap, uint16_t *rx_len,
					 uint32_t timeout_ms);

uint8_t ModbusMaster_ReadHoldingRegisters(uint8_t slave, uint16_t reg,
					  uint16_t count, uint8_t *rx,
					  uint16_t rx_cap, uint16_t *rx_len,
					  uint32_t timeout_ms);

uint8_t ModbusMaster_WriteMultipleRegisters(uint8_t slave, uint16_t start_reg,
					    const uint8_t *regs_be, uint16_t reg_bytes,
					    uint8_t *rx, uint16_t rx_cap, uint16_t *rx_len,
					    uint32_t timeout_ms);

#endif /* MODBUS_MASTER_H */
