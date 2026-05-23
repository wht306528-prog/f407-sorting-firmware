/**
 * app_485_devices.h — RS485(Modbus) 报警灯等设备语义封装
 *
 * - 伺服：仍用 modbus_master 的 FC03/06/16 便捷函数（内部统一 TxnExec）。
 * - 报警灯：`App485_Alarm_DirectFc06_NoResp` 只发不等答。
 */
#ifndef APP_485_DEVICES_H
#define APP_485_DEVICES_H

#include <stdint.h>

uint8_t App485_Alarm_DirectFc06_NoResp(uint16_t value_for_direct_register);

#endif
