/**
 * app_matrix.c — 矩阵表 `s_tbl[]`：增删查改 + 苗盘统计 + 搬移后本地更新
 *
 * 【零基础读法】
 * - s_used：当前缓存有效矩阵行（满格常为 150，稀疏时为 N<M_expected）。
 * - `s_underflow`：上一帧上位声明的行数不够——只记灯/提示，不自动把状态机打死（防上位机抖一下整机锁死）。
 * - `s_have_mask`：bit0~2 表示矩阵里是否出现过盘 1/2/3 的数据（用来粗判「三只盘是不是都见过」）。
 * - `s_geom_dirty`：有几何还没批量算完；`FlushPendingGeometry` 会清 dirty 并调用 `recompute_geom`。
 *
 * 【与 app_sort 的分工】`CheckTrayFullOrEmpty` / `FindFirst*` 是分拣标志位 FL2/FL3… 的「眼睛」；
 * 真正动手的是 `app_arm` + `app_conveyor`。
 *
 * 性能说明：`AppMatrix_ApplyTransfer` 只交换两行 class_id；UVZ 与 theta 不变，刻意不再全表逆解，
 * 逆解仅在新矩阵写入后由 `s_geom_dirty`→`FlushPendingGeometry` 批量完成。
 */

#include "app_matrix.h"
#include "app_matrix_modbus.h"
#include "app_matrix_raw_validator.h"
#include "app_calibration_params.h"
#include "app_motor.h"
#include "app_kinematics.h"

#include <stdio.h>

#include <string.h>
static MatrixFinalRow_t s_tbl[MATRIX_MAX_ROWS];
static uint16_t         s_used;
static uint8_t          s_underflow;
static uint8_t          s_have_mask;
static uint8_t          s_geom_dirty;
static AppMatrixGeomDiag_t s_geom_diag;

/*
 * 功能：对缓存表 s_tbl 全量做相机→臂平面坐标与两关节逆解，失败行 geom_ok=0。
 * 交互：内部仅在 AppMatrix_FlushPendingGeometry 中调用；内部调用 UV 射线求交平面、对称五杆逆解；读 app_calibration_params.h。
 */
static void recompute_geom(void)
{
	uint16_t i;

	memset(&s_geom_diag, 0, sizeof(s_geom_diag));
	for (i = 0u; i < s_used; i++)
	{
		float xc = 0.f;
		float yc = 0.f;
		float xw = 0.f;
		float yw = 0.f;
		float th1 = 0.f;
		float th2 = 0.f;
		int32_t ik;

		AppKinematics_UvRayTray_ToArmPlane(s_tbl[i].u, s_tbl[i].v,
						 &xc, &yc, &xw, &yw);
		s_tbl[i].Xc_mm = xc;
		s_tbl[i].Yc_mm = yc;
		s_tbl[i].Xw_mm = xw;
		s_tbl[i].Yw_mm = yw;

		ik = Robotic_arm_dynamics_cal(1u, &th1, &th2, &xw,
					      &yw);
		if (ik == 0)
		{
			s_tbl[i].theta1_deg = th1;
			s_tbl[i].theta2_deg = th2;
			s_tbl[i].geom_ok = 1u;
			s_tbl[i].pulse_motor1_abs =
				(int32_t)CALIB_JOINT1_ZERO_PULSE +
				AppMotor_ServoAngleToPulse(th1);
			s_tbl[i].pulse_motor2_abs =
				(int32_t)CALIB_JOINT2_ZERO_PULSE +
				AppMotor_ServoAngleToPulse(th2);
		}
		else
		{
			s_geom_diag.bad_count++;
			if (s_geom_diag.first_valid == 0u)
			{
				s_geom_diag.first_valid = 1u;
				s_geom_diag.first_idx = i;
				s_geom_diag.tray_id = s_tbl[i].tray_id;
				s_geom_diag.col = s_tbl[i].col;
				s_geom_diag.row = s_tbl[i].row;
				s_geom_diag.class_id = s_tbl[i].class_id;
				s_geom_diag.u = s_tbl[i].u;
				s_geom_diag.v = s_tbl[i].v;
				s_geom_diag.z_mm = s_tbl[i].z_mm;
				s_geom_diag.Xw_mm = xw;
				s_geom_diag.Yw_mm = yw;
				s_geom_diag.ik_err = ik;
			}
			s_tbl[i].theta1_deg = 0.f;
			s_tbl[i].theta2_deg = 0.f;
			s_tbl[i].geom_ok = 0u;
			s_tbl[i].pulse_motor1_abs = 0;
			s_tbl[i].pulse_motor2_abs = 0;
		}
	}
}

/*
 * 功能：清空矩阵缓存、行数与几何脏标志。
 * 交互：外部被 app_tcp_server断线/新连接、app_protocol 失败路径、AppMatrix_SetFromTcpParser 校验失败等调用。
 */
void AppMatrix_Clear(void)
{
	memset(s_tbl, 0, sizeof(s_tbl));
	s_used = 0u;
	s_underflow = 0u;
	s_have_mask = 0u;
	s_geom_dirty = 0u;
	memset(&s_geom_diag, 0, sizeof(s_geom_diag));
	AppMatrixModbus_OnMatrixCleared();
}

/*
 * 功能：返回当前有效矩阵行数 s_used。
 * 交互：外部被 app_sort、协议/显示等查询矩阵是否收到数据。
 */
uint16_t AppMatrix_GetValidCount(void)
{
	return s_used;
}

/*
 * 功能：按索引拷贝一行最终矩阵数据到 out。
 * 交互：外部被机械臂/调试路径读取单格；越界或空指针返回 0。
 */
uint8_t AppMatrix_GetRow(uint16_t index, MatrixFinalRow_t *out)
{
	/* 防御：无效指针或索引越过当前有效行数 */
	if (out == NULL || index >= s_used)
	{
		return 0u;
	}
	*out = s_tbl[index];
	return 1u;
}

/*
 * 功能：查询上一帧是否出现「声明行数不足」类 underflow 标志。
 * 交互：外部被 app_sort notify_matrix_warn 等用于软提示。
 */
uint8_t AppMatrix_LastFrameUnderflow(void)
{
	return s_underflow;
}

/*
 * 功能：输出（并返回）当前矩阵中出现过的 tray 位掩码 s_have_mask。
 * 交互：外部被分拣逻辑判断三盘是否曾出现。
 */
uint8_t AppMatrix_TrayPresence(uint8_t *mask)
{
	if (mask)
	{
		*mask = s_have_mask;
	}
	return s_have_mask;
}

/*
 * 功能：按 tray 统计 class 是否整盘单一（0/1/2）或混合/不一致（44）。
 * 交互：外部被 app_sort FL2/FL3 等策略；只读 s_tbl。
 */
uint8_t AppMatrix_CheckTrayFullOrEmpty(uint8_t tray_id, uint16_t cols, uint16_t rows)
{
	uint32_t         slots;
	uint32_t         n_match;
	uint16_t         i;
	uint32_t         c0 = 0u;
	uint32_t         c1 = 0u;
	uint32_t         c2 = 0u;

	/* 约定：44 表示「判定无效/不一致」，供上层状态机进入故障路径 */
	if (cols == 0u || rows == 0u || tray_id == 0u)
	{
		return 44u;
	}

	slots = (uint32_t)cols * (uint32_t)rows;
	n_match = 0u;

	for (i = 0u; i < s_used; i++)
	{
		/* 只统计指定托盘上的格子 */
		if (s_tbl[i].tray_id != (uint16_t)tray_id)
		{
			continue;
		}
		n_match++;
		if (s_tbl[i].class_id == 0u)
		{
			c0++;
		}
		else if (s_tbl[i].class_id == 1u)
		{
			c1++;
		}
		else if (s_tbl[i].class_id == 2u)
		{
			c2++;
		}
		else
		{
			/* 类别编码超出协议约定 → 整个托盘视为不可用 */
			return 44u;
		}
	}

	/* 上位声明的行列乘积必须与矩阵里该托盘实际条目数一致，否则缺格/多格 */
	if (n_match != slots)
	{
		return 44u;
	}

	if (c0 == slots)
	{
		return 0u;
	}
	if (c1 == slots)
	{
		return 1u;
	}
	if (c2 == slots)
	{
		return 2u;
	}
	/* 混合类别或非单一归类：对「整盘同类」检测而言不合格 */
	return 44u;
}

/*
 * 功能：判断 bit0~2 是否表示三盘均曾在矩阵数据中出现。
 * 交互：外部被 app_sort FL2 门禁。
 */
uint8_t AppMatrix_AllThreeTraysPresent(void)
{
	/* bit0~2：分别在矩阵数据中见过托盘 1/2/3 至少一格 */
	if ((s_have_mask & 0x07u) == 0x07u)
	{
		return 1u;
	}
	return 0u;
}

/*
 * 功能：判断指定盘上是否存在任意 class_id 格（不检查几何）。
 * 交互：外部策略查询存量。
 */
uint8_t AppMatrix_TrayHasClass(uint8_t tray_id, uint16_t class_id)
{
	uint16_t i;

	for (i = 0u; i < s_used; i++)
	{
		if (s_tbl[i].tray_id == (uint16_t)tray_id &&
		    s_tbl[i].class_id == class_id)
		{
			return 1u;
		}
	}
	return 0u;
}

/*
 * 功能：判断指定盘上是否存在 class_id 且该行几何有效（geom_ok）。
 * 交互：外部被 app_sort 取苗前门禁。
 */
uint8_t AppMatrix_TrayHasClassWithGeom(uint8_t tray_id, uint16_t class_id)
{
	uint16_t i;

	for (i = 0u; i < s_used; i++)
	{
		if (s_tbl[i].tray_id == (uint16_t)tray_id &&
		    s_tbl[i].class_id == class_id && s_tbl[i].geom_ok != 0u)
		{
			return 1u;
		}
	}
	return 0u;
}

/*
 * 功能：查找首个满足 tray+class 且 geom_ok 的行索引。
 * 交互：外部被 app_sort 机械臂取放选源格。
 */
uint8_t AppMatrix_FindFirstRowByTrayClass(uint8_t tray_id, uint16_t class_want,
					  uint16_t *out_idx)
{
	uint16_t i;

	if (out_idx == NULL)
	{
		return 0u;
	}
	for (i = 0u; i < s_used; i++)
	{
		if (s_tbl[i].tray_id == (uint16_t)tray_id &&
		    s_tbl[i].class_id == class_want && s_tbl[i].geom_ok != 0u)
		{
			*out_idx = i;
			return 1u;
		}
	}
	return 0u;
}

/*
 * 功能：查找指定盘上首个 class_id=0（空位）且 geom_ok 的行。
 * 交互：外部被 app_sort 作为放苗目标格。
 */
uint8_t AppMatrix_FindFirstEmptyOnTray(uint8_t tray_id, uint16_t *out_idx)
{
	uint16_t i;

	if (out_idx == NULL)
	{
		return 0u;
	}
	for (i = 0u; i < s_used; i++)
	{
		if (s_tbl[i].tray_id == (uint16_t)tray_id &&
		    s_tbl[i].class_id == 0u && s_tbl[i].geom_ok != 0u)
		{
			*out_idx = i;
			return 1u;
		}
	}
	return 0u;
}

/*
 * 功能：若 s_geom_dirty 置位则批量调用 recompute_geom 完成逆解。
 * 交互：外部主循环由 AppSort_Poll 调用；避免在中断语境里算几何。
 */
void AppMatrix_FlushPendingGeometry(void)
{
	/* 解析阶段只置 dirty，主循环统一批量逆解，减轻瞬时 CPU 峰值 */
	if (s_geom_dirty == 0u)
	{
		return;
	}
	s_geom_dirty = 0u;
	recompute_geom();
}

/*
 * 功能：返回几何是否尚待 Flush（矩阵写入后批量算）。
 * 交互：调试/协议侧可选查询。
 */
uint8_t AppMatrix_IsGeomDirty(void)
{
	return s_geom_dirty;
}

/*
 * 功能：判断单行逆解是否有效。
 * 交互：外部业务或测试。
 */
uint8_t AppMatrix_RowGeomValid(uint16_t index)
{
	if (index >= s_used)
	{
		return 0u;
	}
	return s_tbl[index].geom_ok;
}

void AppMatrix_GetGeomDiag(AppMatrixGeomDiag_t *out)
{
	if (out == NULL)
	{
		return;
	}
	*out = s_geom_diag;
}

/*
 * 功能：分拣门禁：全域 geom_ok。
 * 交互：被 AppMatrix_IsSortGeometryReady 使用。
 */
uint8_t AppMatrix_AllRowsGeomValid(void)
{
	uint16_t i;

	if (s_used == 0u)
	{
		return 0u;
	}
	for (i = 0u; i < s_used; i++)
	{
		if (s_tbl[i].geom_ok == 0u)
		{
			return 0u;
		}
	}
	return 1u;
}

/*
 * 功能：分拣门禁：有行、无待算 dirty、且全域几何有效。
 * 交互：外部被 app_sort sort_matrix_ready_strict / sorting_tick。
 */
uint8_t AppMatrix_IsSortGeometryReady(void)
{
	if (s_used == 0u)
	{
		return 0u;
	}
	if (s_geom_dirty != 0u)
	{
		return 0u;
	}
	return AppMatrix_AllRowsGeomValid();
}

/*
 * 功能：逻辑搬运：交换源/目的 class_id，不重算几何与关节角。
 * 交互：外部在机械臂成功后更新本地矩阵镜像；索引越界返回 0。
 */
uint8_t AppMatrix_ApplyTransfer(uint16_t src_idx, uint16_t dst_idx)
{
	uint16_t cls;

	if (src_idx >= s_used || dst_idx >= s_used)
	{
		/* 索引越界说明上层与矩阵快照不一致，拒绝搬运以防内存踩踏 */
		return 0u;
	}
	cls = s_tbl[src_idx].class_id;
	s_tbl[src_idx].class_id = 0u;
	s_tbl[dst_idx].class_id = cls;
	/* class_id 仅逻辑标签；几何与关节角不因搬运改变，不重算全表以省 CPU */
	return 1u;
}

/*
 * 功能：经协议解析写入矩阵表；校验行列/网格规则，置 s_geom_dirty 待主循环逆解。
 * 交互：外部被 Modbus 成功路径与 app_protocol END 成功路径调用；内部调用 AppMatrix_Clear、AppMatrixRaw_Validate*；失败写 failbuf。
 */
uint8_t AppMatrix_SetFromTcpParser(const MatrixFinalRow_t *rows, uint16_t n_rows,
				   uint16_t declared_count,
				   char *failbuf, uint16_t fail_cap)
{
	char   local_fb[24];
	char  *fb = (failbuf != NULL) ? failbuf : local_fb;
	uint16_t fc = (failbuf != NULL) ? fail_cap : (uint16_t)sizeof(local_fb);

	if (rows == NULL)
	{
		AppMatrix_Clear();
		if (fb && fc > 0u)
		{
			(void)snprintf(fb, fc, "MAT:null rows");
		}
		return 0u;
	}
	if (declared_count != n_rows || n_rows > MATRIX_MAX_ROWS ||
	    n_rows == 0u)
	{
		AppMatrix_Clear();
		if (fb && fc > 0u)
		{
			(void)snprintf(fb, fc, "MAT:decl/n");
		}
		return 0u;
	}
	if (n_rows == MATRIX_EXPECTED_ROWS)
	{
		if (!AppMatrixRaw_ValidateFullGrid(rows, n_rows, fb, fc))
		{
			AppMatrix_Clear();
			return 0u;
		}
	}
	else
	{
		if (!AppMatrixRaw_ValidateSparseSlots(rows, n_rows, fb, fc))
		{
			AppMatrix_Clear();
			return 0u;
		}
	}

	{
		uint16_t copy = n_rows;

		if (copy > MATRIX_MAX_ROWS)
		{
			copy = MATRIX_MAX_ROWS;
		}
		memcpy(s_tbl, rows, sizeof(MatrixFinalRow_t) * copy);
		s_used = copy;
		/* START,count<150（稀疏下发）不计为“上位机抖动缺行”；仅保留字段供将来扩展 */
		s_underflow = 0u;
		s_have_mask = 0u;
		s_geom_dirty = 1u;
		{
			uint16_t ii;

			for (ii = 0u; ii < s_used; ii++)
			{
				if (s_tbl[ii].tray_id == 1u)
				{
					s_have_mask |= 1u;
				}
				if (s_tbl[ii].tray_id == 2u)
				{
					s_have_mask |= 2u;
				}
				if (s_tbl[ii].tray_id == 3u)
				{
					s_have_mask |= 4u;
				}
			}
		}
	}

	return 1u;
}
