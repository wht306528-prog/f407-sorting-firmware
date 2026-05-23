/**
 * modbus_master.c — 整机 Modbus RTU 主站（从站为伺服、报警灯等）
 *
 * 组帧、静音判帧尾、CRC、超时重试、历史记录；底层字节收发在 bsp_uart.c。
 * 业务请用 ModbusMaster_TxnExec 或读写封装，勿绕过本模块直接 UART。
 *
 * 互斥 s_mb_owner：裸机下同一时刻仅一事务占 485；失败返回 MB_MASTER_ERR_BUSY，屏上 BSY。
 * KEY2 手写整帧走 SendRawBlocking，流程仍受互斥与 DE 管理。
 */

#include "modbus_master.h"

#include "bsp_uart.h"
#include "delay.h"
#include "misc.h"
#include "modbus_crc.h"
#include "global_config.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/** 独占 RS485 事务互斥标记（裸机轮询上下文，不设优先级继承）*/
static volatile uint8_t       s_mb_owner;

static ModbusTxnResult_t     s_last_result;
static uint8_t               s_last_tx_cache[MB_MASTER_MAX_ADU];
static uint16_t              s_last_tx_cache_len;

static ModbusHistEntry_t     s_hist[MB_HIST_DEPTH];
static uint8_t               s_hist_count;
static uint8_t               s_hist_w;

/*
 * 功能：阻塞等待获取 RS485 主站互斥；超时返回 BUSY。
 * 交互：内部仅被 ModbusMaster_TxnExec 调用；临界区 __disable_irq。
 */
static uint8_t mutex_take_ms(uint32_t wait_ms)
{
	uint32_t t0 = SysTick_GetMs();

	for (;;)
	{
		uint32_t now = SysTick_GetMs();
		uint8_t  taken = 0u;

		/* 临界区：原子判断「当前无人占用 RS485」并立刻占位，避免两个任务交错发帧 */
		__disable_irq();
		if (s_mb_owner == 0u)
		{
			/* 空闲 → 本上下文获得互斥，后续发送/接收独占电机口 USART（BSP_USART1_* 实现为 USART2） */
			s_mb_owner = 1u;
			taken = 1u;
		}
		__enable_irq();
		if (taken != 0u)
		{
			return MB_MASTER_OK;
		}
		/* 已被占用：轮询等待至超时；超时返回 BUSY，调用方可稍后重试或放弃 */
		if ((now - t0) >= wait_ms)
		{
			return MB_MASTER_ERR_BUSY;
		}
	}
}

/*
 * 功能：释放 RS485 互斥，允许下一事务。
 * 交互：TxnExec 末尾与错误路径配对 mutex_take_ms。
 */
static void mutex_release(void)
{
	s_mb_owner = 0u;
}

/*
 * 功能：按波特率估算 Modbus RTU 帧间静默时间（ms），用于收包判帧尾。
 * 交互：内部仅被 master_recv_adu 使用；读 CFG_RS485_BAUD。
 */
static uint32_t mb_rtu_gap_ms(void)
{
	uint32_t baud = CFG_RS485_BAUD;

	/* 高速：规范允许用固定 ≥1.75ms；这里取 2ms，足够判定「一帧结束后的线路静默」 */
	if (baud > 19200u)
	{
		return 2u;
	}
	/* 异常低配：避免除零，退回常用 9600 估算字符时间 */
	if (baud < 1200u)
	{
		baud = 9600u;
	}
	{
		uint32_t bpm = baud / 1000u;

		if (bpm == 0u)
		{
			bpm = 1u;
		}
		/* 低速：按约 3.5 个字符时间折算毫秒（经验系数 38），用于判定整帧边界 */
		return (38u + bpm - 1u) / bpm;
	}
}

/*
 * 功能：将一次事务结果压入环形历史缓冲（调用方已持互斥）。
 * 交互：TxnExec 成功后调用；拷贝 tx/rx RAW。
 */
static void hist_push_locked(const ModbusTxnResult_t *rs, uint32_t ms)
{
	ModbusHistEntry_t *slot = &s_hist[s_hist_w];

	slot->ms_tick = ms;
	slot->tx_len = rs->tx_len;
	if (slot->tx_len > MB_MASTER_MAX_ADU)
	{
		/* 与 RX 同理：历史环形缓冲固定上限 */
		slot->tx_len = MB_MASTER_MAX_ADU;
	}
	slot->rx_len = rs->rx_len;
	/* 历史缓冲容量有限：超长 ADU 只截断存储，避免 memcpy 越界 */
	if (slot->rx_len > MB_MASTER_MAX_ADU)
	{
		slot->rx_len = MB_MASTER_MAX_ADU;
	}
	slot->final_err = rs->err;
	slot->attempts = rs->attempts_used;
	memcpy(slot->tx, rs->tx_raw, slot->tx_len);
	/* 仅当有接收内容才拷贝 RX；超时未收到字节时 rx_len 为 0 */
	if (slot->rx_len > 0u)
	{
		memcpy(slot->rx, rs->rx_raw, slot->rx_len);
	}

	s_hist_w = (uint8_t)((s_hist_w + 1u) % MB_HIST_DEPTH);
	/* 环形写指针前进；未满深度时递增计数，便于 HistCount 判断可读条数 */
	if (s_hist_count < MB_HIST_DEPTH)
	{
		s_hist_count++;
	}
}

/*
 * 功能：自电机口 USART RX 环轮询收字节直到静音 gap 满足或超时，凑齐一帧 RTU ADU。
 * 交互：内部被 txn_once_inner 调用；BSP_USART1_*、BSP_RS485_TracePushRxFrameEnd、mb_rtu_gap_ms。
 */
static uint8_t master_recv_adu(uint8_t *rx, uint16_t rx_cap, uint16_t *out_len,
			       uint32_t timeout_ms)
{
	uint32_t t0 = SysTick_GetMs();
	uint32_t last_rx = 0u;
	uint16_t len = 0u;
	uint32_t gap = mb_rtu_gap_ms();

	*out_len = 0u;

	for (;;)
	{
		uint8_t  b;
		uint32_t now = SysTick_GetMs();

		while (BSP_USART1_ReadByte(&b) != 0u)
		{
			/* 防止上位机/噪声无限灌数据撑爆应用缓冲 */
			if (len >= rx_cap)
			{
				return MB_MASTER_ERR_BUF;
			}
			rx[len] = b;
			len++;
			last_rx = SysTick_GetMs();
		}

		now = SysTick_GetMs();

		if (len > 0u)
		{
			/* 已有字节且距离最后一字节已超过 gap：Modbus RTU 判定「一帧收齐」 */
			if ((now - last_rx) >= gap)
			{
				*out_len = len;
				BSP_RS485_TracePushRxFrameEnd();
				return MB_MASTER_OK;
			}
		}

		if ((now - t0) >= timeout_ms)
		{
			/* 整个窗口内一个字节没来：从站掉线、线路断路或地址不匹配 */
			if (len == 0u)
			{
				return MB_MASTER_ERR_TIMEOUT;
			}
			/* 超时瞬间若已满足静默，仍可当作残缺环境下的「尽力完整帧」 acceptance */
			if ((now - last_rx) >= gap)
			{
				*out_len = len;
				BSP_RS485_TracePushRxFrameEnd();
				return MB_MASTER_OK;
			}
			/* 已超时但未满足 RTU 静默：视为半帧/噪声，丢弃避免误判成功 */
			return MB_MASTER_ERR_TIMEOUT;
		}
	}
}

/*
 * 功能：解析一帧 Modbus 应答：CRC、异常码、FC03/06/10 数据域提取到 out。
 * 交互：外部/内部被 txn_once_inner 调用；依赖 Modbus_CheckCRC。
 */
uint8_t ModbusMaster_ParseResponse(const uint8_t *rx, uint16_t rx_len, uint8_t req_fc,
				  ModbusTxnParsed_t *out)
{
	memset(out, 0, sizeof(*out));
	/* 最短帧：站址+功能码+CRC(2)，再加若干数据；否则无法合法解析 */
	if ((rx_len < 4u) || (rx == NULL))
	{
		return MB_MASTER_ERR_ARG;
	}
	/* CRC 失败通常是 EMI、波特率不一致或帧边界判错，上层可触发重试 */
	if (!Modbus_CheckCRC(rx, rx_len))
	{
		return MB_MASTER_ERR_CRC;
	}
	out->slave = rx[0];
	out->fc_echo = rx[1];
	out->data_len = 0u;

	/* 功能码最高位置 1：从站返回「异常应答」，后跟异常码而非业务数据 */
	if ((rx[1] & 0x80u) != 0u)
	{
		out->is_exception = 1u;
		if (rx_len >= 5u)
		{
			out->exception_code = rx[2];
		}
		return MB_MASTER_OK;
	}
	/* 正常应答却与请求功能码不一致：非法帧或串扰 */
	if (rx[1] != req_fc)
	{
		return MB_MASTER_ERR_FC;
	}

	switch (req_fc)
	{
	case 0x03u:
	{
		uint8_t bc = rx[2];
		/* 03 应答格式：addr, fc, byte_count, data..., crc；总长必须与声明一致 */
		if ((uint32_t)3u + (uint32_t)bc + 2u != (uint32_t)rx_len)
		{
			return MB_MASTER_ERR_LEN;
		}
		/* 防止寄存器数量过大超出本地解析缓冲 */
		if (bc > sizeof(out->data))
		{
			return MB_MASTER_ERR_BUF;
		}
		memcpy(out->data, &rx[3], bc);
		out->data_len = bc;
		break;
	}
	case 0x06u:
		/* 写单寄存器应答与请求同为 8 字节固定格式 */
		if (rx_len != 8u)
		{
			return MB_MASTER_ERR_LEN;
		}
		memcpy(out->data, &rx[2], 4u);
		out->data_len = 4u;
		break;

	case 0x10u:
		/* 写多寄存器应答：起始地址+数量echo，固定 8 字节 */
		if (rx_len != 8u)
		{
			return MB_MASTER_ERR_LEN;
		}
		memcpy(out->data, &rx[2], 4u);
		out->data_len = 4u;
		break;
	default:
		break;
	}
	return MB_MASTER_OK;
}


/*
 * 功能：根据 rq 拼 ADU：普通模式 slave+fc+payload+CRC，或 RAW 整帧拷贝。
 * 交互：内部被 txn_once_inner 调用；Modbus_AppendCRC。
 */
static uint8_t txn_once_build_tx(const ModbusTxnRequest_t *rq, uint8_t *tx, uint16_t *tx_total)
{
	if ((rq->flags & MB_MODBUS_TXN_FLAG_RAW_ADU) != 0u)
	{
		/* 原始 ADU：调用方已拼好含 CRC 的完整帧，此处只做合法性边界检查 */
		if (rq->raw_complete == NULL || rq->raw_complete_len < 4u ||
		    rq->raw_complete_len > MB_MASTER_MAX_ADU)
		{
			return MB_MASTER_ERR_ARG;
		}
		memcpy(tx, rq->raw_complete, rq->raw_complete_len);
		*tx_total = rq->raw_complete_len;
		return MB_MASTER_OK;
	}

	/* 站址 0 在 Modbus 里多为广播禁止应答；本主站实现不允许 0 */
	if (rq->slave == 0u)
	{
		return MB_MASTER_ERR_ARG;
	}
	if ((rq->payload_len > 0u) && (rq->payload == NULL))
	{
		return MB_MASTER_ERR_ARG;
	}
	{
		uint16_t pdu_len = (uint16_t)(2u + rq->payload_len);

		if (((uint32_t)pdu_len + 2u) > (uint32_t)MB_MASTER_MAX_ADU)
		{
			return MB_MASTER_ERR_ARG;
		}
		tx[0] = rq->slave;
		tx[1] = rq->fc;
		if (rq->payload_len > 0u)
		{
			/* PDU 数据域紧随功能码之后 */
			memcpy(&tx[2], rq->payload, rq->payload_len);
		}
		Modbus_AppendCRC(tx, pdu_len);
		*tx_total = (uint16_t)(pdu_len + 2u);
	}
	return MB_MASTER_OK;
}

/*
 * 功能：执行单次 Modbus 事务：组帧、USART 发送、可选收包、CRC/站址/FC 校验、ParseResponse。
 * 交互：内部被 ModbusMaster_TxnExec 重试循环调用；BSP_USART1_*、master_recv_adu、ModbusMaster_ParseResponse。
 */
static uint8_t txn_once_inner(const ModbusTxnRequest_t *rq, ModbusTxnResult_t *rs,
			      uint8_t attempt_idx)
{
	uint8_t  tx[MB_MASTER_MAX_ADU];
	uint16_t tx_total = 0u;
	uint8_t  br;
	uint16_t rlen;
	uint8_t  req_fc_for_parse;

	memset(rs, 0, sizeof(*rs));
	rs->attempts_used = attempt_idx;

	br = txn_once_build_tx(rq, tx, &tx_total);
	if (br != MB_MASTER_OK)
	{
		rs->err = br;
		return br;
	}

	req_fc_for_parse = rq->fc;
	if ((rq->flags & MB_MODBUS_TXN_FLAG_RAW_ADU) != 0u)
	{
		/* 原始模式下「期望功能码」以 ADU 第二个字节为准 */
		req_fc_for_parse = tx[1];
	}

	memcpy(rs->tx_raw, tx, tx_total);
	rs->tx_len = tx_total;
	memcpy(s_last_tx_cache, tx, tx_total);
	s_last_tx_cache_len = tx_total;

	/* 先发前清空 RX FIFO，避免上一轮残留字节拼进当前应答 */
	BSP_USART1_RxFlush();
	BSP_USART1_SendBlocking(tx, tx_total);

	if ((rq->flags & MB_MODBUS_TXN_FLAG_NO_RESPONSE) != 0u)
	{
		/* 广播/测试：只发送不等答，立即返回成功（风险：无法确认从站收到） */
		rs->rx_len = 0u;
		rs->err = MB_MASTER_OK;
		return MB_MASTER_OK;
	}

	br = master_recv_adu(rs->rx_raw, sizeof(rs->rx_raw), &rlen, rq->rsp_timeout_ms);
	if (br != MB_MASTER_OK)
	{
		rs->rx_len = 0u;
		rs->err = br;
		return br;
	}

	rs->rx_len = rlen;

	if (rlen < 4u || !Modbus_CheckCRC(rs->rx_raw, rlen))
	{
		rs->err = MB_MASTER_ERR_CRC;
		return MB_MASTER_ERR_CRC;
	}

	/* 应答站址必须与请求一致，否则认为是总线上其它设备的串扰帧 */
	if (rs->rx_raw[0] != tx[0])
	{
		rs->err = MB_MASTER_ERR_SLAVE;
		return MB_MASTER_ERR_SLAVE;
	}

	if ((rs->rx_raw[1] != req_fc_for_parse) &&
	    (rs->rx_raw[1] != (uint8_t)(req_fc_for_parse | 0x80u)))
	{
		rs->err = MB_MASTER_ERR_FC;
		return MB_MASTER_ERR_FC;
	}

	{
		uint8_t pr = ModbusMaster_ParseResponse(rs->rx_raw, rlen, req_fc_for_parse,
							 &rs->parsed);

		if (pr != MB_MASTER_OK)
		{
			rs->err = pr;
			return pr;
		}
		/* 业务层若未处理异常码，这里统一转成 EXC 便于 UI/日志 */
		if (rs->parsed.is_exception != 0u)
		{
			rs->err = MB_MASTER_ERR_EXCEPTION;
			return MB_MASTER_ERR_EXCEPTION;
		}
	}

	rs->err = MB_MASTER_OK;
	return MB_MASTER_OK;
}

/*
 * 功能：对外主入口：互斥、单次/重试事务、更新 last result 与历史。
 * 交互：外部被 app_motor、app_485_devices、SendRaw* 等调用；mutex_*、txn_once_inner、hist_push_locked。
 */
uint8_t ModbusMaster_TxnExec(const ModbusTxnRequest_t *rq, ModbusTxnResult_t *rs)
{
	uint8_t        mx;
	uint8_t        e1;
	ModbusTxnResult_t local;
	ModbusTxnResult_t *rr = (rs != NULL) ? rs : &local;
	uint8_t        max_try = 2u;

	if (rq == NULL)
	{
		return MB_MASTER_ERR_ARG;
	}
	/* 调用方禁止自动重试：例如调试时要观测单次失败波形 */
	if ((rq->flags & MB_MODBUS_TXN_FLAG_NO_RETRY) != 0u)
	{
		max_try = 1u;
	}
	if ((rq->flags & MB_MODBUS_TXN_FLAG_NO_RESPONSE) != 0u)
	{
		max_try = 1u;
	}

	uint8_t        att;
	uint32_t       mtx_wait;

	mtx_wait = (rq->mutex_wait_ms != 0u) ? rq->mutex_wait_ms : MB_MUTEX_WAIT_MS;
	mx = mutex_take_ms(mtx_wait);
	if (mx != MB_MASTER_OK)
	{
		return mx;
	}

	e1 = MB_MASTER_ERR_TIMEOUT;
	rr->retries_done = 0u;

	for (att = 1u; att <= max_try; att++)
	{
		e1 = txn_once_inner(rq, rr, att);
		if (e1 == MB_MASTER_OK)
		{
			break;
		}
		/* 仅超时/CRC 这类「可得帧不完整」错误自动重试；其它错误不重发以免副作用 */
		if (att < max_try &&
		    (e1 == MB_MASTER_ERR_TIMEOUT || e1 == MB_MASTER_ERR_CRC))
		{
			rr->retries_done++;
			Delay_ms(2u);
			continue;
		}
		break;
	}

	memcpy(&s_last_result, rr, sizeof(*rr));
	hist_push_locked(rr, SysTick_GetMs());
	mutex_release();
	return e1;
}

/*
 * 功能：将 MB_MASTER_* 错误码映射为短 TAG 串（屏显 KEY2 等前缀）。
 * 交互：外部被 main.Main_ReportModbusFault 等调用。
 */
const char *ModbusMaster_ErrTag(uint8_t err)
{
	static const char *const tags[] =
	{"OK", "TO", "CRC", "ADR", "FC", "BUF", "ARG", "BSY", "EXC", "LEN", "ABT"};

	if (err <= MB_MASTER_ERR_ABORT)
	{
		return tags[err];
	}
	return "?";
}

/*
 * 功能：返回指向最后一次 TxnExec 结果结构的只读指针。
 * 交互：调试/UI 可选。
 */
const ModbusTxnResult_t *ModbusMaster_GetLastResult(void)
{
	return &s_last_result;
}

/*
 * 功能：按「从新到旧」索引读取历史环中一条事务摘要。
 * 交互：外部调试显示；与 ModbusMaster_HistCount 配合。
 */
void ModbusMaster_HistGet(uint16_t idx_from_newest0, ModbusHistEntry_t *out)
{
	uint16_t i;

	if (out == NULL)
	{
		return;
	}
	memset(out, 0, sizeof(ModbusHistEntry_t));

	if (s_hist_count == 0u)
	{
		return;
	}
	/* 索引越界时钳到最旧可读槽，避免调试接口崩溃 */
	if (idx_from_newest0 >= s_hist_count)
	{
		idx_from_newest0 = (uint16_t)(s_hist_count - 1u);
	}
	i = (uint16_t)((s_hist_w + MB_HIST_DEPTH - 1u - idx_from_newest0) % MB_HIST_DEPTH);
	*out = s_hist[i];
}

/*
 * 功能：返回当前历史中可读条数。
 * 交互：外部 UI/格式化前检查。
 */
uint16_t ModbusMaster_HistCount(void)
{
	return s_hist_count;
}

/*
 * 功能：生成单行 ASCII 摘要（长度、错误码、TX/RX HEX 片段）。
 * 交互：外部屏显或小日志缓冲。
 */
void ModbusMaster_FormatHistLine(uint16_t idx_from_newest0, char *dst, unsigned cap)
{
	ModbusHistEntry_t e;
	unsigned           pos;

	if (dst == NULL || cap < 24u)
	{
		return;
	}
	if (ModbusMaster_HistCount() == 0u)
	{
		(void)snprintf(dst, cap, "no RS485 txn history yet");
		dst[cap - 1u] = '\0';
		return;
	}
	memset(&e, 0, sizeof(e));
	dst[0] = '\0';
	ModbusMaster_HistGet(idx_from_newest0, &e);
	pos = (unsigned)snprintf(dst, cap,
				 "HS:%lu Tx%d Rx%d ER=%02x",
				 (unsigned long)e.ms_tick, (unsigned)e.tx_len,
				 (unsigned)e.rx_len,
				 (unsigned)e.final_err);
	/* 在单行 dbg 缓冲内追加 HEX（截断保护） */
	if (pos < cap && e.tx_len > 0u && pos + 8u < cap)
	{
		unsigned j;

		pos += (unsigned)snprintf(dst + pos, cap - pos, "|TX:");
		for (j = 0u; j < (unsigned)e.tx_len && pos + 5u < cap; j++)
		{
			pos += (unsigned)snprintf(dst + pos, cap - pos, "%02X",
						  (unsigned int)e.tx[j]);
		}
	}
	if (pos < cap && e.rx_len > 0u && pos + 8u < cap)
	{
		unsigned j;

		pos += (unsigned)snprintf(dst + pos, cap - pos, "|RX:");
		for (j = 0u; j < (unsigned)e.rx_len && pos + 5u < cap; j++)
		{
			pos += (unsigned)snprintf(dst + pos, cap - pos, "%02X",
						  (unsigned int)e.rx[j]);
		}
	}
	dst[cap - 1u] = '\0';
}

/*
 * 功能：拷贝最后一次发出的 TX ADU（截断至 cap）。
 * 交互：调试对比波形。
 */
uint16_t ModbusMaster_GetLastTxAdu(uint8_t *out, uint16_t cap)
{
	uint16_t copy_len = s_last_tx_cache_len;

	if (copy_len > cap)
	{
		copy_len = cap;
	}
	if ((out != NULL) && (copy_len > 0u))
	{
		memcpy(out, s_last_tx_cache, copy_len);
	}
	return copy_len;
}

/*
 * 功能：发送已含 CRC 的完整 RAW ADU 并等待标准应答。
 * 交互：外部由 main KEY2、demo 帧调用；TxnExec RAW_ADU 分支。
 */
uint8_t ModbusMaster_SendRawBlocking(const uint8_t *data, uint16_t len,
				    uint32_t mutex_wait_ms)
{
	ModbusTxnRequest_t rq = {0};
	ModbusTxnResult_t  rs = {0};

	rq.mutex_wait_ms = (mutex_wait_ms != 0u) ? mutex_wait_ms : MB_MUTEX_WAIT_MS;
	rq.raw_complete = data;
	rq.raw_complete_len = len;
	rq.rsp_timeout_ms = CFG_MODBUS_RSP_TIMEOUT_MS;
	rq.flags = MB_MODBUS_TXN_FLAG_RAW_ADU;
	return ModbusMaster_TxnExec(&rq, &rs);
}

/*
 * 功能：发送 RAW ADU（已含 CRC）且不解析应答（占位测试/不要求应答的设备）。
 * 交互：外部极少用；TxnExec NO_RESPONSE。
 */
uint8_t ModbusMaster_SendRawNoResponse(const uint8_t *data, uint16_t len,
				       uint32_t mutex_wait_ms)
{
	ModbusTxnRequest_t rq = {0};
	ModbusTxnResult_t  rs = {0};

	rq.mutex_wait_ms = (mutex_wait_ms != 0u) ? mutex_wait_ms : MB_MUTEX_WAIT_MS;
	rq.raw_complete = data;
	rq.raw_complete_len = len;
	rq.rsp_timeout_ms = 0u;
	rq.flags = (uint8_t)(MB_MODBUS_TXN_FLAG_RAW_ADU | MB_MODBUS_TXN_FLAG_NO_RESPONSE);
	return ModbusMaster_TxnExec(&rq, &rs);
}

/*
 * 功能：旧式封装：组装 ModbusTxnRequest 调 TxnExec，并把应答 RAW 拷入 legacy 缓冲。
 * 交互：内部被 WriteSingle/ReadMultiple/FC10 包装调用。
 */
static uint8_t wrapper_txn(uint8_t slave, uint8_t fc,
			   const uint8_t *payload, uint16_t payload_len,
			   uint32_t rsp_to,
			   uint8_t *legacy_rx_buf, uint16_t legacy_cap, uint16_t *legacy_rx_len)
{
	ModbusTxnRequest_t rq = {0};
	ModbusTxnResult_t  rs;
	uint8_t            e;

	rq.slave = slave;
	rq.fc = fc;
	rq.payload = payload;
	rq.payload_len = payload_len;
	rq.rsp_timeout_ms = rsp_to;
	rq.flags = 0u;

	e = ModbusMaster_TxnExec(&rq, &rs);
	if (legacy_rx_len != NULL)
	{
		*legacy_rx_len = rs.rx_len;
	}
	if ((legacy_rx_buf != NULL) && (legacy_cap > 0u) && (rs.rx_len > 0u))
	{
		uint16_t cp = rs.rx_len;

		if (cp > legacy_cap)
		{
			cp = legacy_cap;
		}
		memcpy(legacy_rx_buf, rs.rx_raw, cp);
	}
	return e;
}

/*
 * 功能：FC06 写单寄存器便捷 API。
 * 交互：外部 app_motor 报警寄存器等；wrapper_txn。
 */
uint8_t ModbusMaster_WriteSingleRegister(uint8_t slave, uint16_t reg,
					 uint16_t value, uint8_t *rx,
					 uint16_t rx_cap, uint16_t *rx_len,
					 uint32_t timeout_ms)
{
	uint8_t pdu[4];

	pdu[0] = (uint8_t)((reg >> 8) & 0xFFu);
	pdu[1] = (uint8_t)(reg & 0xFFu);
	pdu[2] = (uint8_t)((value >> 8) & 0xFFu);
	pdu[3] = (uint8_t)(value & 0xFFu);

	return wrapper_txn(slave, 0x06u, pdu, 4u, timeout_ms, rx, rx_cap, rx_len);
}

/*
 * 功能：FC03 读保持寄存器（量大端起始地址+数量）。
 * 交互：外部 app_motor 读反馈位置等。
 */
uint8_t ModbusMaster_ReadHoldingRegisters(uint8_t slave, uint16_t reg,
					  uint16_t count, uint8_t *rx,
					  uint16_t rx_cap, uint16_t *rx_len,
					  uint32_t timeout_ms)
{
	uint8_t pdu[4];

	pdu[0] = (uint8_t)((reg >> 8) & 0xFFu);
	pdu[1] = (uint8_t)(reg & 0xFFu);
	pdu[2] = (uint8_t)((count >> 8) & 0xFFu);
	pdu[3] = (uint8_t)(count & 0xFFu);

	return wrapper_txn(slave, 0x03u, pdu, 4u, timeout_ms, rx, rx_cap, rx_len);
}

/*
 * 功能：FC10 写多个寄存器，regs_be 为大数据端字节序寄存器逐个拼接。
 * 交互：外部 app_motor 写 32bit 目标位置等多寄存器。
 */
uint8_t ModbusMaster_WriteMultipleRegisters(uint8_t slave, uint16_t start_reg,
					    const uint8_t *regs_be, uint16_t reg_bytes,
					    uint8_t *rx, uint16_t rx_cap, uint16_t *rx_len,
					    uint32_t timeout_ms)
{
	uint8_t  txp[256];
	uint16_t reg_count;
	uint16_t pdu_len;

	/* 寄存器数据必须为偶数字节（每寄存器 16bit）；且本地 PDU 缓冲固定 256 */
	if (regs_be == NULL || (reg_bytes & 1u) != 0u || reg_bytes == 0u)
	{
		return MB_MASTER_ERR_ARG;
	}
	reg_count = (uint16_t)(reg_bytes / 2u);
	if ((uint32_t)5u + reg_bytes > sizeof(txp))
	{
		return MB_MASTER_ERR_ARG;
	}

	txp[0] = (uint8_t)((start_reg >> 8) & 0xFFu);
	txp[1] = (uint8_t)(start_reg & 0xFFu);
	txp[2] = (uint8_t)((reg_count >> 8) & 0xFFu);
	txp[3] = (uint8_t)(reg_count & 0xFFu);
	txp[4] = (uint8_t)reg_bytes;
	memcpy(&txp[5], regs_be, reg_bytes);
	pdu_len = (uint16_t)(5u + reg_bytes);

	return wrapper_txn(slave, 0x10u, txp, pdu_len, timeout_ms, rx, rx_cap, rx_len);
}
