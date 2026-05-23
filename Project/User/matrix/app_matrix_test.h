/**
 * app_matrix_test.h — 调试：连续注入 MATRIX_SAMPLE_WINDOW_FRAMES 帧随机合法 Matrix_Raw，经 AppProtocol_OnStream 走与 TCP 相同路径
 */
#ifndef APP_MATRIX_TEST_H
#define APP_MATRIX_TEST_H

#include <stdint.h>

void AppMatrixTest_InjectRandomViaProtocol(uint32_t seed);

#endif /* APP_MATRIX_TEST_H */
