/**
 * gui_drv_cfg.h — 脱 emXGUI：仅保留触摸屏回调占位。
 */
#ifndef GUI_DRV_CFG_H
#define GUI_DRV_CFG_H

#include <stdint.h>

void Touch_Button_Down(int32_t x, int32_t y);
void Touch_Button_Up(int32_t x, int32_t y);

#endif
