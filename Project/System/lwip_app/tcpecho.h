/**
 * tcpecho.h — 遗留 TCP echo 初始化（端口 TCP_ECHO_PORT）
 *
 * 实现 tcpecho.c；与 User/tcp 矩阵服务独立。
 */
#ifndef _TCPECHO_H_
#define _TCPECHO_H_

#define TCP_ECHO_PORT 5001

void TCP_Echo_Init(void);

#endif


