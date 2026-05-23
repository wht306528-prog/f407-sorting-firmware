/**
 * app_tcp_server.h — Matrix ASCII TCP 服务端对外入口
 *
 * 实现见 app_tcp_server.c：lwIP raw API 监听 MATRIX_TCP_SERVER_PORT，将 pbuf 流
 * 交给 AppProtocol_OnStream（先 `AppProtocol_ShouldAcceptStream`，再 `AppSort_ShouldDiscardTcpMatrixStream` 决定是否真喂解析器）。
 * 与上位机断连时会清协议状态与矩阵缓存。
 */
#ifndef __APP_TCP_SERVER_H__
#define __APP_TCP_SERVER_H__

/** 创建监听 PCB、绑定端口并接受连接；应在 netif 就绪后调用一次 */
void AppTcpMatrix_Init(void);

#endif
