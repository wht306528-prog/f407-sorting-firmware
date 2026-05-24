/**
 * app_matrix.h — 视觉「试管矩阵」在单片机里的缓存
 *
 * 【数据从哪来、谁在读】
 * - 来：`app_matrix_modbus.c` 经 USART3 Modbus RTU 读保持寄存器组装行表 → `AppMatrix_SetFromTcpParser`；
 *   可选 ASCII 流路径（`app_protocol` OnStream）仅调试/RAW 快照，量产主入口为 Modbus。
 * - 算：`AppSort_Poll` 里调 `AppMatrix_FlushPendingGeometry` 批量做逆解。
 * - 读：`app_sort.c`、`app_arm.c`、`app_display.c`。
 */

#ifndef __APP_MATRIX_H__
#define __APP_MATRIX_H__

#include <stdint.h>
#include "global_config.h"

typedef struct
{
	uint16_t tray_id;
	uint16_t col;
	uint16_t row;
	uint16_t class_id;
	uint16_t confidence; /* 0~10000 定点 */

	int32_t  u;
	int32_t  v;
	int32_t  z_mm;

	float    Xc_mm;
	float    Yc_mm;
	float    Xw_mm;
	float    Yw_mm;
	float    theta1_deg;
	float    theta2_deg;
	uint8_t  geom_ok; /*!< 逆解有效；失败时为 0，禁止盲走 0° */
	/**
	 * Matrix_Final 行对外语义末尾两列：电机 1、2 相对机械定义的绝对目标脉冲
	 * = CALIB_JOINT*_ZERO_PULSE + AppMotor_ServoAngleToPulse(命令关节角)。
	 * 运动下发前先读反馈再在 MCU 内得到 Δ（见 AppMotor_GotoAbsTargetAsRelative）。
	 */
	int32_t pulse_motor1_abs;
	int32_t pulse_motor2_abs;
} MatrixFinalRow_t;

typedef struct
{
	uint16_t bad_count;
	uint8_t  first_valid;
	uint16_t first_idx;
	uint16_t tray_id;
	uint16_t col;
	uint16_t row;
	uint16_t class_id;
	int32_t  u;
	int32_t  v;
	int32_t  z_mm;
	float    Xw_mm;
	float    Yw_mm;
	int32_t  ik_err;
} AppMatrixGeomDiag_t;

typedef struct
{
	uint16_t total_count;
	uint16_t empty_count;
	uint16_t white_count;
	uint16_t yellow_count;
	uint16_t invalid_count;
	uint8_t  complete;
} AppMatrixTrayStats_t;

void AppMatrix_Clear(void);

uint16_t AppMatrix_GetValidCount(void);

uint8_t AppMatrix_GetRow(uint16_t index, MatrixFinalRow_t *out);

/** 最近一次矩阵帧声明行数不足 MATRIX_EXPECTED_ROWS（仅置位，不自动改标志位） */
uint8_t AppMatrix_LastFrameUnderflow(void);

/**
 * 托盘在位粗判：本帧数据中是否出现过 tray_id=1/2/3。
 * @param[out] mask 可选；bit0=盘1 bit1=盘2 bit2=盘3
 * @return 同上 mask 值
 */
uint8_t AppMatrix_TrayPresence(uint8_t *mask);

/**
 * 判定指定苗盘是否「格数齐全」且类别一致。
 * @param tray_id 1..3
 * @param cols,rows 单盘网格（通常 MATRIX_TRAY_COLS × MATRIX_TRAY_ROWS）
 * @return 0=该盘全部 class_id==0（全空）；
 *         1=全部 class_id==1（满一类）；
 *         2=全部 class_id==2（满二类）；
 *         44=格数不对、缺行、或类别混合/非法；
 *         255=保留（内部未使用）
 */
uint8_t AppMatrix_CheckTrayFullOrEmpty(uint8_t tray_id, uint16_t cols, uint16_t rows);

uint8_t AppMatrix_GetTrayClassStats(uint8_t tray_id, uint16_t cols, uint16_t rows,
				    AppMatrixTrayStats_t *out);

/** 三盘 mask 是否均为 1（mask==0x07） */
uint8_t AppMatrix_AllThreeTraysPresent(void);

/** 在 table 原始顺序扫描：是否存在 tray_id 上指定 class_id（1 或 2） */
uint8_t AppMatrix_TrayHasClass(uint8_t tray_id, uint16_t class_id);

/**
 * 找第一个匹配 tray_id 且 class_id==class_want 的行索引（0..used-1），且 geom_ok=1。
 * @return 1 找到并写入 *out_idx；0 未找到
 */
uint8_t AppMatrix_FindFirstRowByTrayClass(uint8_t tray_id, uint16_t class_want,
					  uint16_t *out_idx);

/**
 * 找 tray_id 上第一个空穴（class_id==0），且 geom_ok=1。
 */
uint8_t AppMatrix_FindFirstEmptyOnTray(uint8_t tray_id, uint16_t *out_idx);

/**
 * 机械臂完成一次搬移后更新内存矩阵（源穴置空，目标穴写入原类别），并立刻刷新逆解。
 * @return 1 成功；0 索引非法。
 */
uint8_t AppMatrix_ApplyTransfer(uint16_t src_idx, uint16_t dst_idx);

/**
 * 格位全集检查通过后拷贝入表。
 * @param failbuf 失败原因的短 ASCII（可上屏）；可为 NULL 则不写原因
 * @param fail_cap failbuf 字节容量，含结尾 0
 * @return 1 成功；0 失败（保留上一帧成功矩阵，不自动 AppMatrix_Clear）
 */
uint8_t AppMatrix_SetFromTcpParser(const MatrixFinalRow_t *rows, uint16_t n_rows,
				  uint16_t declared_count,
				  char *failbuf, uint16_t fail_cap);

void AppMatrix_FlushPendingGeometry(void);

uint8_t AppMatrix_IsGeomDirty(void);

uint8_t AppMatrix_RowGeomValid(uint16_t index);

void AppMatrix_GetGeomDiag(AppMatrixGeomDiag_t *out);

/** 全部有效行逆解已通过（常用于进入 SORTING 前的硬门槛） */
uint8_t AppMatrix_AllRowsGeomValid(void);

/** 矩阵行数为期望值、且无待算几何 dirty、各行 geom_ok（需先 Flush） */
uint8_t AppMatrix_IsSortGeometryReady(void);

/** tray 上是否存在指定 class（且该行 geom_ok=1）；用于分拣策略分支不含“不可达盲点” */
uint8_t AppMatrix_TrayHasClassWithGeom(uint8_t tray_id, uint16_t class_id);

#endif
