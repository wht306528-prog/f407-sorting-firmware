/**
 * app_kinematics.h — 相机针孔 + 关节正逆解合一接口
 *
 * 对称五杆正/逆解在 `Robotic_arm_dynamics_cal()`；连杆与零位脉冲宏 `CALIB_ARM_*`、
 * `CALIB_JOINT*_ZERO_PULSE` 见 app_calibration_params.h（由 Calibration/apply_calibration_to_f407.py 生成）。
 *
 * 像素 u,v + 射线与工作平面求交 → tray_from_cam → T_tray_to_arm → 臂平面 Xw,Yw(mm)，
 * 再经 Robotic_arm_dynamics_cal 逆解。
 */
#ifndef __APP_KINEMATICS_H__
#define __APP_KINEMATICS_H__

#include <stdint.h>

/* --------- 连杆 mm（按图纸修改；当前：基座 50cm、主动杆 60cm、从动杆 70cm） --------- */
#ifndef ARM_PARAM_L0_MM
#define ARM_PARAM_L0_MM   500.0f /* 基座 / 偏置 mm */
#endif
#ifndef ARM_PARAM_L1_MM
#define ARM_PARAM_L1_MM   600.0f /* 主动杆（等效逆解杆 1） */
#endif
#ifndef ARM_PARAM_L2_MM
#define ARM_PARAM_L2_MM   700.0f /* 从动杆（五杆中间段占位） */
#endif
#ifndef ARM_PARAM_L3_MM
#define ARM_PARAM_L3_MM   700.0f
#endif
#ifndef ARM_PARAM_L4_MM
#define ARM_PARAM_L4_MM   600.0f /* 末端杆（等效逆解杆 2，与 L1 对称占位） */
#endif

/* 相机标定参数（可从 global_config 宏覆盖）；单位：像素 / mm */
#ifndef CAM_PARAM_FX
#define CAM_PARAM_FX   900.0f
#endif
#ifndef CAM_PARAM_FY
#define CAM_PARAM_FY   900.0f
#endif
#ifndef CAM_PARAM_CX
#define CAM_PARAM_CX   400.0f
#endif
#ifndef CAM_PARAM_CY
#define CAM_PARAM_CY   240.0f
#endif

/**
 * Robotic_arm_dynamics_cal — 对称五连杆平面正/逆解（角度与 app_motor/AppArm 语义一致）：
 * flag=0 正解 — *a,*b = 伺服命令关节角（°，含 SIGN+零点），输出 *x,*y=末端(mm)
 * flag=1 逆解 — *x,*y = 末端(mm)，输出命令角 *a,*b（°）。
 *
 * @retval 0 成功；-2 几何不可达等。
 */
int32_t Robotic_arm_dynamics_cal(uint8_t flag, float *a, float *b, float *x, float *y);

/** 仅针孔深度模型到相机坐标系（调试）。 */
void AppKinematics_CameraUVZ_ToWorld(int32_t u, int32_t v, int32_t z_mm,
				     float *Xc_mm, float *Yc_mm, float *Zw_mm);

/** UV + 射线与 tray Z≈平面求交 → 臂基 XY；兼容旧名，忽略 z_mm。 */
void AppKinematics_UvRayTray_ToArmPlane(int32_t u, int32_t v, float *Xc_mm,
					float *Yc_mm, float *Xw_mm,
					float *Yw_mm);

void AppKinematics_CameraUVZ_ToArmPlane(int32_t u, int32_t v, int32_t z_mm,
					float *Xc_mm, float *Yc_mm,
					float *Xw_mm, float *Yw_mm);

#endif
