/**
 * app_ros2_bridge.h — 名义 micro-ROS 桥：MCU 实际只做 Matrix TCP Server
 *
 * AppRos2Bridge_Init：内部调用 AppTcpMatrix_Init（lwIP 已就绪时调）。
 * AppRos2Bridge_OnTick：占位；主循环可每圈调用，当前无状态。
 */
#ifndef __APP_ROS2_BRIDGE_H__
#define __APP_ROS2_BRIDGE_H__

#include <stdint.h>

void AppRos2Bridge_Init(void);

void AppRos2Bridge_OnTick(uint32_t tick_ms);

#endif
