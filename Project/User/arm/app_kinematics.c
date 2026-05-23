/**
 * app_kinematics.c — 针孔射线 × 苗盘平面 → tray→arm ，对称五杆正/逆解
 * 标定宏见 app_calibration_params.h。
 */
#include <stddef.h>
#include <stdint.h>
#include "app_kinematics.h"
#include "app_calibration_params.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CAL_EPS (1e-4f)

static uint8_t dist_all_zero(void)
{
	return (CALIB_DIST_K1 == 0.0f && CALIB_DIST_K2 == 0.0f &&
		CALIB_DIST_P1 == 0.0f && CALIB_DIST_P2 == 0.0f &&
		CALIB_DIST_K3 == 0.0f) ?
		       1u :
		       0u;
}

static void undistort_normalized(float xd, float yd, float *x_ideal,
				 float *y_ideal)
{
	float x = xd;
	float y = yd;
	uint16_t it;

	if (dist_all_zero() != 0u)
	{
		*x_ideal = xd;
		*y_ideal = yd;
		return;
	}
	for (it = 0u; it < 15u; it++)
	{
		float r2 = x * x + y * y;
		float r4 = r2 * r2;
		float r6 = r4 * r2;
		float k_rad =
			1.0f + CALIB_DIST_K1 * r2 + CALIB_DIST_K2 * r4 +
			CALIB_DIST_K3 * r6;
		float tx = 2.0f * CALIB_DIST_P1 * x * y +
			   CALIB_DIST_P2 * (r2 + 2.0f * x * x);
		float ty = CALIB_DIST_P1 * (r2 + 2.0f * y * y) +
			   2.0f * CALIB_DIST_P2 * x * y;
		float x0 = x * k_rad + tx;
		float y0 = y * k_rad + ty;

		x += xd - x0;
		y += yd - y0;
	}
	*x_ideal = x;
	*y_ideal = y;
}

static void pixel_undistort(float u_in, float v_in, float *u_out,
			    float *v_out)
{
	float xn = (u_in - CALIB_CAM_CX_PX) / CALIB_CAM_FX_PX;
	float yn = (v_in - CALIB_CAM_CY_PX) / CALIB_CAM_FY_PX;
	float xu, yu;

	undistort_normalized(xn, yn, &xu, &yu);
	*u_out = xu * CALIB_CAM_FX_PX + CALIB_CAM_CX_PX;
	*v_out = yu * CALIB_CAM_FY_PX + CALIB_CAM_CY_PX;
}

void AppKinematics_CameraUVZ_ToWorld(int32_t u, int32_t v, int32_t z_mm,
				     float *Xc_mm, float *Yc_mm, float *Zw_mm)
{
	float Zc = (float)z_mm;
	float uu, vv;

	pixel_undistort((float)u, (float)v, &uu, &vv);
	if (Xc_mm)
	{
		*Xc_mm = (uu - CALIB_CAM_CX_PX) * Zc / CALIB_CAM_FX_PX;
	}
	if (Yc_mm)
	{
		*Yc_mm = (vv - CALIB_CAM_CY_PX) * Zc / CALIB_CAM_FY_PX;
	}
	if (Zw_mm)
	{
		*Zw_mm = Zc;
	}
}

/* 两圆求交，返回交点个数 0/1/2 */
static uint8_t circle_circle_ix(float xa, float ya, float ra, float xb,
				float yb, float rb, float *x1, float *y1,
				float *x2, float *y2)
{
	float dx = xb - xa;
	float dy = yb - ya;
	float d2 = dx * dx + dy * dy;
	float d;

	if (d2 < CAL_EPS * CAL_EPS)
	{
		return 0u;
	}
	d = sqrtf(d2);
	if (d > ra + rb + CAL_EPS || d < fabsf(ra - rb) - CAL_EPS)
	{
		return 0u;
	}

	{
		float a = (ra * ra - rb * rb + d * d) / (2.f * d);
		float hh = ra * ra - a * a;

		if (hh < -CAL_EPS)
		{
			return 0u;
		}
		if (hh < 0.f)
		{
			hh = 0.f;
		}
		hh = sqrtf(hh);

		float mx = xa + dx * a / d;
		float my = ya + dy * a / d;
		float ox = -(dy / d) * hh;
		float oy = (dx / d) * hh;

		*x1 = mx + ox;
		*y1 = my + oy;
		*x2 = mx - ox;
		*y2 = my - oy;
		return hh <= CAL_EPS ? 1u : 2u;
	}
}

void AppKinematics_UvRayTray_ToArmPlane(int32_t u, int32_t v, float *Xc_mm,
					float *Yc_mm,
					float *Xw_mm, float *Yw_mm)
{
	float uu, vv;
	float dnx, dny, dnz = 1.f;
	float den;
	float scal;
	float Pcx, Pcy, Pcz;
	float ptx, pty, ptz;
	float xh, yh, zh, wh;

	pixel_undistort((float)u, (float)v, &uu, &vv);
	dnx = (uu - CALIB_CAM_CX_PX) / CALIB_CAM_FX_PX;
	dny = (vv - CALIB_CAM_CY_PX) / CALIB_CAM_FY_PX;

	den =
		CALIB_R_TRAY_FROM_CAM_20 * dnx +
		CALIB_R_TRAY_FROM_CAM_21 * dny +
		CALIB_R_TRAY_FROM_CAM_22 * dnz;
	if (fabsf(den) < 1e-7f)
	{
		goto fail;
	}

	scal = -CALIB_T_TRAY_FROM_CAM_Z_MM / den;
	Pcx = scal * dnx;
	Pcy = scal * dny;
	Pcz = scal * dnz;

	if (Xc_mm)
	{
		*Xc_mm = Pcx;
	}
	if (Yc_mm)
	{
		*Yc_mm = Pcy;
	}

	ptx =
		CALIB_R_TRAY_FROM_CAM_00 * Pcx +
		CALIB_R_TRAY_FROM_CAM_01 * Pcy +
		CALIB_R_TRAY_FROM_CAM_02 * Pcz +
		CALIB_T_TRAY_FROM_CAM_X_MM;
	pty =
		CALIB_R_TRAY_FROM_CAM_10 * Pcx +
		CALIB_R_TRAY_FROM_CAM_11 * Pcy +
		CALIB_R_TRAY_FROM_CAM_12 * Pcz +
		CALIB_T_TRAY_FROM_CAM_Y_MM;
	ptz =
		CALIB_R_TRAY_FROM_CAM_20 * Pcx +
		CALIB_R_TRAY_FROM_CAM_21 * Pcy +
		CALIB_R_TRAY_FROM_CAM_22 * Pcz +
		CALIB_T_TRAY_FROM_CAM_Z_MM;

	xh =
		CALIB_T_TRAY_TO_ARM_00 * ptx +
		CALIB_T_TRAY_TO_ARM_01 * pty +
		CALIB_T_TRAY_TO_ARM_02 * ptz +
		CALIB_T_TRAY_TO_ARM_03;
	yh =
		CALIB_T_TRAY_TO_ARM_10 * ptx +
		CALIB_T_TRAY_TO_ARM_11 * pty +
		CALIB_T_TRAY_TO_ARM_12 * ptz +
		CALIB_T_TRAY_TO_ARM_13;
	zh =
		CALIB_T_TRAY_TO_ARM_20 * ptx +
		CALIB_T_TRAY_TO_ARM_21 * pty +
		CALIB_T_TRAY_TO_ARM_22 * ptz +
		CALIB_T_TRAY_TO_ARM_23;
	wh =
		CALIB_T_TRAY_TO_ARM_30 * ptx +
		CALIB_T_TRAY_TO_ARM_31 * pty +
		CALIB_T_TRAY_TO_ARM_32 * ptz +
		CALIB_T_TRAY_TO_ARM_33;

	if (fabsf(wh) >= 1e-9f)
	{
		xh /= wh;
		yh /= wh;
	}

	if (Xw_mm)
	{
		*Xw_mm = xh;
	}
	if (Yw_mm)
	{
		*Yw_mm = yh;
	}
	return;

fail:
	if (Xc_mm)
	{
		*Xc_mm = 0.f;
	}
	if (Yc_mm)
	{
		*Yc_mm = 0.f;
	}
	if (Xw_mm)
	{
		*Xw_mm = 0.f;
	}
	if (Yw_mm)
	{
		*Yw_mm = 0.f;
	}
}

void AppKinematics_CameraUVZ_ToArmPlane(int32_t u, int32_t v, int32_t z_mm,
					float *Xc_mm, float *Yc_mm,
					float *Xw_mm, float *Yw_mm)
{
	(void)z_mm;
	AppKinematics_UvRayTray_ToArmPlane(u, v, Xc_mm, Yc_mm,
					  Xw_mm, Yw_mm);
}


/*
 * 给定末端 E(xy)，对称五连杆：肘点 C(on A+Lal)、D(on B+Lar)，再求主动角 θ1 θ2。
 */

static void choose_elbow_higher_y(float ux1, float uy1, float ux2,
				  float uy2, float *ox, float *oy)
{
	if (uy1 >= uy2)
	{
		*ox = ux1;
		*oy = uy1;
	}
	else
	{
		*ox = ux2;
		*oy = uy2;
	}
}

static uint8_t segment_dist_ok_mm(float xa, float ya, float xb, float yb,
				  float rex)
{
	return fabsf(hypotf(xa - xb, ya - yb) - rex) <= 5.0f ? 1u : 0u;
}

static int32_t fb_inv(float Ex, float Ey, float *t1_deg, float *t2_deg)
{
	float ax = CALIB_ARM_AX_MM;
	float ay = CALIB_ARM_AY_MM;
	float bx = CALIB_ARM_BX_MM;
	float by = CALIB_ARM_BY_MM;
	float lal = CALIB_ARM_L_ACTIVE_LEFT_MM;
	float pal = CALIB_ARM_L_PASSIVE_LEFT_MM;
	float lar = CALIB_ARM_L_ACTIVE_RIGHT_MM;
	float par = CALIB_ARM_L_PASSIVE_RIGHT_MM;
	float ux1;
	float uy1;
	float ux2;
	float uy2;
	float vx1;
	float vy1;
	float vx2;
	float vy2;
	uint8_t nl;
	float Cx;
	float Cy;
	float Dx;
	float Dy;
	float ia1_deg;
	float ia2_deg;

	nl =
		circle_circle_ix(ax, ay, lal, Ex, Ey, pal,
				 &ux1, &uy1, &ux2, &uy2);
	if (nl == 0u)
	{
		return -2;
	}
	if (nl == 2u)
	{
		choose_elbow_higher_y(ux1, uy1, ux2, uy2, &Cx, &Cy);
	}
	else
	{
		Cx = ux1;
		Cy = uy1;
	}

	nl =
		circle_circle_ix(bx, by, lar, Ex, Ey, par,
				 &vx1, &vy1, &vx2, &vy2);
	if (nl == 0u)
	{
		return -2;
	}
	if (nl == 2u)
	{
		choose_elbow_higher_y(vx1, vy1, vx2, vy2, &Dx, &Dy);
	}
	else
	{
		Dx = vx1;
		Dy = vy1;
	}

	if (!(segment_dist_ok_mm(Ex, Ey, Cx, Cy, pal)) ||
	    !(segment_dist_ok_mm(Ex, Ey, Dx, Dy, par)))
	{
		return -2;
	}

	ia1_deg = atan2f(Cy - ay, Cx - ax) * (float)(180.0 / M_PI);
	ia2_deg = atan2f(Dy - by, Dx - bx) * (float)(180.0 / M_PI);

	*t1_deg =
		CALIB_JOINT1_SIGN * (ia1_deg + CALIB_JOINT1_ZERO_DEG);
	*t2_deg =
		CALIB_JOINT2_SIGN * (ia2_deg + CALIB_JOINT2_ZERO_DEG);

	return 0;
}

static int32_t fb_fwd(float t1_cmd_deg, float t2_cmd_deg, float *Ex,
		      float *Ey)
{
	float ax = CALIB_ARM_AX_MM;
	float ay = CALIB_ARM_AY_MM;
	float bx = CALIB_ARM_BX_MM;
	float by = CALIB_ARM_BY_MM;
	float lal = CALIB_ARM_L_ACTIVE_LEFT_MM;
	float pal = CALIB_ARM_L_PASSIVE_LEFT_MM;
	float lar = CALIB_ARM_L_ACTIVE_RIGHT_MM;
	float par = CALIB_ARM_L_PASSIVE_RIGHT_MM;
	float ia1_deg;
	float ia2_deg;
	float t1_rad;
	float t2_rad;
	float Cx;
	float Cy;
	float Dx;
	float Dy;
	float ux1;
	float uy1;
	float ux2;
	float uy2;
	uint8_t nl;

	if (fabsf(CALIB_JOINT1_SIGN) < CAL_EPS ||
	    fabsf(CALIB_JOINT2_SIGN) < CAL_EPS)
	{
		return -1;
	}

	ia1_deg = (t1_cmd_deg / CALIB_JOINT1_SIGN) -
		  CALIB_JOINT1_ZERO_DEG;
	ia2_deg = (t2_cmd_deg / CALIB_JOINT2_SIGN) -
		  CALIB_JOINT2_ZERO_DEG;

	t1_rad = ia1_deg * (float)(M_PI / 180.0);
	t2_rad = ia2_deg * (float)(M_PI / 180.0);

	Cx = ax + lal * cosf(t1_rad);
	Cy = ay + lal * sinf(t1_rad);
	Dx = bx + lar * cosf(t2_rad);
	Dy = by + lar * sinf(t2_rad);

	nl =
		circle_circle_ix(Cx, Cy, pal, Dx, Dy, par,
				 &ux1, &uy1, &ux2, &uy2);
	if (nl == 0u)
	{
		return -2;
	}
	if (nl == 2u)
	{
		choose_elbow_higher_y(ux1, uy1, ux2, uy2, Ex, Ey);
	}
	else
	{
		*Ex = ux1;
		*Ey = uy1;
	}

	if (!(segment_dist_ok_mm(Cx, Cy, *Ex, *Ey, pal)) ||
	    !(segment_dist_ok_mm(Dx, Dy, *Ex, *Ey, par)))
	{
		return -2;
	}

	return 0;
}

int32_t Robotic_arm_dynamics_cal(uint8_t flag, float *a, float *b, float *x,
				 float *y)
{
	if (a == NULL || b == NULL || x == NULL || y == NULL)
	{
		return -1;
	}

	if (flag == 0u)
	{
		float ex;
		float ey;

		if (fb_fwd(*a, *b, &ex, &ey) != 0)
		{
			return -2;
		}
		*x = ex;
		*y = ey;
		return 0;
	}
	if (flag == 1u)
	{
		return fb_inv(*x, *y, a, b);
	}

	return -3;
}
