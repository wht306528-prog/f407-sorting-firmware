/**
 * app_ros2_bridge.c — 命名含 ROS2，实参为启动 Matrix ASCII TCP 服务端
 *
 * 说明：本机未运行 ROS2 节点。若上位机为 ROS2，需在 PC 或 3588 侧将表格打成
 * app_protocol.c 定义的 START … END 文本，经 TCP 发往本板。
 *
 * 数据流：Init → `AppTcpMatrix_Init`（lwIP accept）→ `matrix_recv`；
 * 分拣 FL2/FL3 时 RJ45 载荷会被 `AppSort_ShouldDiscardTcpMatrixStream` 丢进位桶，不进入解析器。
 *
 * AppRos2Bridge_OnTick 当前为空实现，主循环可每圈调用，几乎无开销。
 */

#include "app_ros2_bridge.h"

#include "app_tcp_server.h"

/*
 * 功能：启动矩阵 ASCII TCP 服务端（工程内命名含 ROS2，本板不跑 ROS 节点）。
 * 交互：外部由 main 上电调用一次；内部调用 AppTcpMatrix_Init。
 */
void AppRos2Bridge_Init(void)
{
	AppTcpMatrix_Init();
}

/*
 * 功能：预留周期性钩子；当前空实现，可与未来网管/诊断扩展对接。
 * 交互：外部由 main 主循环每圈调用。
 */
void AppRos2Bridge_OnTick(uint32_t tick_ms)
{
	(void)tick_ms;
}
