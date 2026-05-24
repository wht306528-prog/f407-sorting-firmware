/**
 * =============================================================================
 * global_config.h — 调机向导：换线、换脚、换波特率，优先只改这一个头文件
 * =============================================================================
 * 零基础说明：
 * - 这里全是 #define 常数，不改 .c 就能改很多行为；改错会导致电机不动、485 不通、矩阵对不上。
 * - 量产 Keil 目标为**双路 RS485**（USART2 电机/灯 + USART3 矩阵）；下列以太网宏仅 **归档/对照**，应用勿依赖。
 * - ROS2 侧 YAML 真源：ros2_ws/src/psort_bringup/config/
 * - 数字后面带 u 表示 unsigned；时间类单位在注释里标明 ms。
 * - 引脚宏成对出现：*_PORT、*_PIN、*_CLK 是给 bsp_* 初始化 GPIO 用的。
 *
 * 模块职责：为 User / Hardware 提供统一宏，避免业务层写死常数。
 * 硬件资源：不涉及直接寄存器操作，仅 PIN、波特率、机构参数等。
 *
 * 与本头文件强相关的应用模块（改宏后请同步 F407/docs/F407_维护说明.txt、F407_IO引脚与逻辑表.txt）：
 *   RS485/伺服/报警灯：CFG_RS485_BAUD、CFG_MODBUS_SLAVE_*、SERVO_REG_*、CFG_ALARM_*
 *   矩阵：USART3 Modbus RTU（app_matrix_modbus）接 RK3588 等 Modbus 从站，站号 CFG_MATRIX_MODBUS_SLAVE_ID。
 *   执行器脚位：VALVE_*、CONV_MOTOR_*、PHOTO_*、LIMIT_SOFT1、ESTOP
 * =============================================================================
 */
#ifndef GLOBAL_CONFIG_H
#define GLOBAL_CONFIG_H

#include <stdint.h>

/* --------- Modbus / RS485（电机与七色灯总线 = USART2）---------
 * 硬件：USART2（APB1），RCC 分频后USART 逻辑自动算波特率。
 * 引脚：PA2=TX、PA3=RX、PC0=DE/~RE（高=发送）。
 * 波特率：挂在此 485 上的从机（伺服、七色报警灯等）须一致。
 */
#define CFG_RS485_BAUD               115200u

/* 与其它工程名兼容：电机 Modbus / 报警灯 / 手动测试帧 均走上述同一总线 */
#define CFG_USART_MODBUS_BAUD        CFG_RS485_BAUD

#define CFG_MODBUS_SLAVE_MOTOR1      1u
#define CFG_MODBUS_SLAVE_MOTOR2      2u

#define CFG_MODBUS_RSP_TIMEOUT_MS    80u

/* --------- 485 七色报警灯（与电机共用 USART2+PC0 RS485 总线）---------
 * Modbus 06 写寄存器 CFG_ALARM_REG_DIRECT；站号 CFG_ALARM_MODBUS_SLAVE。
 * CFG_ALARM_INTER_FRAME_MS：连续发两帧 ADU 之间的间隔，给从机解析时间。
 */
#define CFG_ALARM_USART_BAUD           CFG_RS485_BAUD
#define CFG_ALARM_MODBUS_SLAVE         0xFFu
#define CFG_ALARM_REG_DIRECT           0x00C2u
#define CFG_ALARM_INTER_FRAME_MS       5u

/** KEY1：仅发一帧 FC06 到七色灯从站（USART2 RS485）；站号可与 CFG_ALARM_MODBUS_SLAVE 不同 */
#define CFG_KEY1_LAMP_SLAVE            3u
#define CFG_KEY1_LAMP_REG              0x00C2u
#define CFG_KEY1_LAMP_VALUE            0x000Fu

/* --------- 伺服寄存器（示例品牌，若实机不同请只改地址/数值，勿改业务语义） --------- */
/* P0D-18：强制使能（FC06，工程寄存器地址见驱动手册；与过往固件 0x0D12 别名一致） */
#define SERVO_REG_ENABLE             0x0D12u
#define SERVO_VAL_ENABLE_ON          507u
#define SERVO_VAL_ENABLE_OFF         511u

#define SERVO_REG_TRIGGER            0x0D08u
/** 历史相对/速度触发（当前点到点绝对流程优先用 SERVO_VAL_TRIGGER_ABS_MOVE） */
#define SERVO_VAL_TRIGGER_RUN        1u
#define SERVO_VAL_TRIGGER_RESET      0u
#define SERVO_VAL_TRIGGER_ABS_MOVE   3u
#define SERVO_VAL_TRIGGER_CLEAR      0u

/** P10-14 → Modbus 起始保持寄存器 0x100E，FC16 写 2×16bit，「低字寄存器在前、高字在后」 */
#define SERVO_REG_POS32_START        0x100Eu

/** P0B-07：反馈绝对脉冲，FC03 读 2 寄存器，低字在前 */
#define SERVO_REG_FB_POS32_START     0x0B07u

/** P0B-04：到位状态，FC03 读 1 寄存器，Bit0=1 视为到位（与位置误差判定二选一即可） */
#define SERVO_REG_REACH_STATUS       0x0B04u

/** P0D-05：急停模式（FC06），1=急停闭锁语义，0=正常 */
#define SERVO_REG_ESTOP_MODE         0x0D05u
#define SERVO_VAL_ESTOP_ON           1u
#define SERVO_VAL_ESTOP_OFF          0u

#define SERVO_POS_POLL_MS            100u
#define SERVO_MOVE_TIMEOUT_MS        30000u
/** AppMotor_WaitPosition：|反馈−目标|≤该脉冲且或 P0B-04.Bit0 即判到位 */
#define SERVO_TARGET_TOL_PULSE       10

/*
 * 若为 0（默认）：SERVO_REG_POS32_START 写入伺服「绝对目标脉冲」终点（与 Final 中 pulse_motor*_abs 一致）。
 * 若为 1：该寄存器写入「相对当前反馈的位移 Δ」脉冲；最终绝对终点仍为 cur_feedback+Δ。
 * 由 AppMotor_GotoAbsTargetAsRelative() 读出反馈后换算；请先对照驱动说明书再改此项。
 */
#ifndef CFG_SERVO_POS_CMD_IS_REL_DELTA
#define CFG_SERVO_POS_CMD_IS_REL_DELTA 0u
#endif

/* 伺服：关节角→驱动器脉冲折算用常数（梯形/速度曲线不在 F407，在驱动器功能码） */

#define SERVO_PULSE_PER_360_REV      10000L

/* 五杆/电机减速：电机角 : 关节角（与 app_kinematics / app_motor 一致；实机 30:1） */

#define CFG_ICHUANDONG_RATIO         30 /* 电机角 : 关节角 */
/* --------- 传送带 / 执行器 / 光电（与 RMII/LCD 解耦后的默认脚，务必对照 F407_IO引脚与逻辑表.txt） ----------
 * 说明：野火 F407 底板 RMII 占用 PC4/PC5(PHY_RXD0/1)、PA1(REF_CLK)、PA2(MDIO)、PB11(TX_EN)、
 * PG13/14(TXD) 等，故电磁阀/电机不能再用「计划初稿」中与 ETH 冲突脚位。
 * 下列为推荐可布线的 GPIO 默认；实板请仅在 global_config 中改宏即可。
 */
#define VALVE_ACTUATOR_Z_PORT        GPIOA
#define VALVE_ACTUATOR_Z_CLK         RCC_AHB1Periph_GPIOA
#define VALVE_ACTUATOR_Z_PIN         GPIO_Pin_8 /* 升降电磁阀：高=开 */

#define VALVE_GRIP_PORT              GPIOB
#define VALVE_GRIP_CLK               RCC_AHB1Periph_GPIOB
#define VALVE_GRIP_PIN               GPIO_Pin_2 /* 吸盘/夹爪：高=开 */

#define CONV_MOTOR_0_PORT            GPIOB
#define CONV_MOTOR_0_CLK             RCC_AHB1Periph_GPIOB
#define CONV_MOTOR_0_PIN             GPIO_Pin_0

#define CONV_MOTOR_1_PORT            GPIOB
#define CONV_MOTOR_1_CLK             RCC_AHB1Periph_GPIOB
#define CONV_MOTOR_1_PIN             GPIO_Pin_12

#define CONV_MOTOR_2_PORT            GPIOE
#define CONV_MOTOR_2_CLK             RCC_AHB1Periph_GPIOE
#define CONV_MOTOR_2_PIN             GPIO_Pin_5

#define PHOTO_0_PORT                 GPIOB
#define PHOTO_0_CLK                  RCC_AHB1Periph_GPIOB
#define PHOTO_0_PIN                  GPIO_Pin_13 /* 默认低电平触发 */

#define PHOTO_1_PORT                 GPIOB
#define PHOTO_1_CLK                  RCC_AHB1Periph_GPIOB
#define PHOTO_1_PIN                  GPIO_Pin_15

#define PHOTO_2_PORT                 GPIOE
#define PHOTO_2_CLK                  RCC_AHB1Periph_GPIOE
#define PHOTO_2_PIN                  GPIO_Pin_3

/* 限位/急停备用输入（无上拉，外部电路决定有效电平） */
#define LIMIT_SOFT1_PORT             GPIOE
#define LIMIT_SOFT1_CLK              RCC_AHB1Periph_GPIOE
#define LIMIT_SOFT1_PIN              GPIO_Pin_4

#define ESTOP_PORT                   GPIOE
#define ESTOP_CLK                    RCC_AHB1Periph_GPIOE
#define ESTOP_PIN                    GPIO_Pin_6

#define CFG_CONVEYOR_TIMEOUT_MS      5000u

/** 取料：Z 下降到位后、输出夹持电磁阀前的等待（现场可调，调用见 app_arm.c valve_pick_sequence） */
#define CFG_ARM_PICK_LOWER_MS        800u
/** 取料：夹爪夹紧输出后的保持时间（与「夹紧后到下一关节移动」不同，见 CFG_ARM_PICK_TO_MOVE_MS） */
#define CFG_ARM_PICK_HOLD_MS         400u
/** 取料：整段 valve_pick_sequence（含抬 Z）结束后，再下发 move_joints_to_row(目标) 前的等待 */
#ifndef CFG_ARM_PICK_TO_MOVE_MS
#define CFG_ARM_PICK_TO_MOVE_MS      50u
#endif
#define CFG_ARM_PICK_RAISE_MS        800u
#define CFG_ARM_PLACE_LOWER_MS       800u
#define CFG_ARM_PLACE_RELEASE_MS     400u
#define CFG_ARM_PLACE_RAISE_MS       800u

/* --------- 矩阵（试管表格大小，必须与上位机下发行数一致）---------
 * MATRIX_EXPECTED_ROWS：分拣逻辑在 `app_sort` 里用它判断是否“表格满了可以开干”。
 * 若上位机少发一行，屏会提示 MAT<EXPECT rows 一类，而不会直接崩。
 */
#define MATRIX_MAX_ROWS              150u
#define MATRIX_EXPECTED_ROWS         150u
/**
 * Matrix_Raw 文本（app_protocol）择优采样窗口：连续 N 个**格位校验通过**的合法帧后再择 1 帧写入矩阵；
 * 默认 1（单帧即提交）；可调 2~4。须与 app_protocol.c 候选缓存一致。
 */
#ifndef MATRIX_SAMPLE_WINDOW_FRAMES
#define MATRIX_SAMPLE_WINDOW_FRAMES  1u
#endif
/**
 * Matrix_Raw 择优提交成功后是否保持 OnStream 开窗：1=可连续解析多帧；0=关闸直至传送带开窗。
 * 注意：不得用 AppProtocol_ArmSampleWindow 代替，其会清 s_last_ok 影响分拣就绪判定。
 */
#ifndef CFG_MATRIX_KEEP_RX_OPEN_AFTER_OK
#define CFG_MATRIX_KEEP_RX_OPEN_AFTER_OK 1u
#endif
/** 单盘列数 × 行数 = 每盘穴位数；三盘合计须与 MATRIX_EXPECTED_ROWS 一致 */
#define MATRIX_TRAY_COLS             5u
#define MATRIX_TRAY_ROWS             10u
#define MATRIX_TRAY_COUNT            3u

/** Matrix_Raw CSV：列/行下标协议为 1..COLS / 1..ROWS；内部槽位仍为 0 基推导 */
#define MATRIX_COL_ROW_BASE          1u
#define MATRIX_CONFIDENCE_MAX        10000u
#define MATRIX_UV_ABS_MAX            1000000L
/**
 * Matrix_Raw CSV 深度列 z_mm 可为占位非正；几何用像素射线与苗盘平面求交，不依此列门控逆解。
 * 占位值仍须在合理范围内以避免解析异常。
 */
#define MATRIX_Z_COORD_ABSENT_MAX    0L
#define MATRIX_Z_MM_MIN              (-10000L)
#define MATRIX_Z_MM_MAX              10000000L

/** AppSort 系统运行标志 44：红灯快闪后关灯回到标志 0 的持续时间 */
#define CFG_SYS_FLAG44_ALARM_MS      2000u

/** AppSort ESTOP 大状态：板载 RGB 红蓝交替提示时长，超时后切红灯常亮（非 KEY2 专用） */
#define CFG_ESTOP_BLINK_MS           1500u

/* --------- 演示用 LED 节拍 --------- */
#define CFG_LED_BLINK_MS             250u

/** NT35510 文本界面完整排版刷新周期（与 RGB 指示灯节拍解耦） */
#define CFG_UI_REFRESH_MS            100u

/** HEX trace：单次 Drain 最大事件数，防止 UI 长时间卡顿 */
#define CFG_HEX_TRACE_DRAIN_MAX      768u

/* --------- USART3：矩阵 Modbus RTU（PB10=TX, PB11=RX）---------
 * 电机/报警灯 RS485 走 USART2（见上）；矩阵独立 USART3，不抢占 modbus_master 互斥。
 * 若接半双工 RS485 且需 DE：置 CFG_MATRIX_RS485_USE_DE=1 并核对 DE 引脚宏。
 */
#define CFG_MATRIX_MODBUS_SLAVE_ID      1u
#define CFG_MATRIX_MODBUS_BAUD            115200u
#ifndef CFG_MATRIX_MODBUS_RSP_TIMEOUT_MS
#define CFG_MATRIX_MODBUS_RSP_TIMEOUT_MS  220u
#endif
#ifndef CFG_MATRIX_MODBUS_POLL_IDLE_MS
#define CFG_MATRIX_MODBUS_POLL_IDLE_MS    350u
#endif
#define CFG_MATRIX_REG_HEADER_BASE      0x0000u
#define CFG_MATRIX_REG_MATRIX_BASE      0x0010u
#define CFG_MATRIX_REGS_PER_CELL        9u
#define CFG_MATRIX_CELLS_PER_PACKET     10u
#define CFG_MATRIX_REGS_PER_PACKET      90u
#define CFG_MATRIX_PACKET_COUNT         15u

/**
 * 1=KEY2 整轮不因字头门禁/尾 batch+CRC32/单包失败而中止：失败包寄存器填 0，
 * 仍凑满 15×90 再组 150 行（联调看收发；量产请改 0 恢复严格校验）。
 */
#ifndef CFG_MATRIX_K2_BYPASS_GATES
#define CFG_MATRIX_K2_BYPASS_GATES      1u
#endif

#ifndef CFG_MATRIX_RS485_USE_DE
#define CFG_MATRIX_RS485_USE_DE         0u
#endif

/*
 * --------- 调试场景选择：只改 0 / 1，不用记一堆散开关 ---------
 *
 * 改这里以后必须重新编译、重新烧录，F407 不会在运行时读取本文件。
 *
 * 你日常只需要看下面两个开关：
 *
 *   CFG_SCENE_PC_SERVO_SIM_TEST = 0/1
 *   CFG_SCENE_REAL_HARDWARE     = 0/1
 *
 * 规则：
 *
 *   0 = 关 / false / 不启用这个场景
 *   1 = 开 / true  / 启用这个场景
 *
 * 场景 0：两个都填 0，最安全，日常默认
 *
 *   CFG_SCENE_PC_SERVO_SIM_TEST = 0
 *   CFG_SCENE_REAL_HARDWARE     = 0
 *
 *   用途：
 *     - 只验证鲁班猫矩阵、Matrix Final、RunFlag/FL 状态机。
 *     - 不发真实电机 Modbus。
 *     - 不开真实传送带。
 *     - 适合日常烧录、拍 LCD、给别人演示矩阵和 FL 状态。
 *
 *   自动得到：
 *     - 几何 SIM 开：用假 X/Y/theta/脉冲，强制 geom_ok=1。
 *     - 机械臂 SIM 开：AppArm 只改内存矩阵，不发电机命令，不驱动阀。
 *     - 传送带 SIM 开：不等光电，不开传送带电机。
 *
 * 场景 1：PC 模拟伺服测试，只把第一个填 1
 *
 *   CFG_SCENE_PC_SERVO_SIM_TEST = 1
 *   CFG_SCENE_REAL_HARDWARE     = 0
 *
 *   用途：
 *     - 验证“Final 生成后，F407 真的会从 USART2 发电机 Modbus”。
 *     - 必须用 USB-RS485 接电脑，并运行 tools/servo_modbus_sim.py。
 *     - 不接真实电机；如果接了真实电机，这个场景可能连续下发多次运动命令。
 *
 *   接线示例：
 *     - USB-RS485 T/R+ -> 开发板电机 485A
 *     - USB-RS485 T/R- -> 开发板电机 485B
 *     - USB-RS485 GND  -> 开发板 GND（建议接）
 *
 *   PC 命令示例：
 *     python tools\servo_modbus_sim.py --port COM4 --baud 115200 -v
 *
 *   自动得到：
 *     - 几何 SIM 开：仍用假脉冲，避免真实标定没做导致 geom:BAD。
 *     - 机械臂 SIM 关：会真实发送 USART2 电机 Modbus。
 *     - 传送带 SIM 开：仍不动传送带。
 *
 * 场景 2：真实硬件，暂时不要开
 *
 *   CFG_SCENE_PC_SERVO_SIM_TEST = 0
 *   CFG_SCENE_REAL_HARDWARE     = 1
 *
 *   用途：
 *     - 将来真实相机标定、真实电机、真实机械臂、真实传送带都准备好以后再用。
 *
 *   启用前必须确认：
 *     - 急停可用；
 *     - 两个电机方向正确；
 *     - 零点脉冲正确；
 *     - 单轴小行程测试通过；
 *     - 真实几何标定参数已经写入；
 *     - 传送带和光电接线确认；
 *     - 不再使用假坐标/假脉冲。
 *
 * 禁止组合：
 *
 *   CFG_SCENE_PC_SERVO_SIM_TEST = 1
 *   CFG_SCENE_REAL_HARDWARE     = 1
 *
 *   这两个不能同时开。一个是“电脑假伺服”，一个是“真实硬件”。
 */
#ifndef CFG_SCENE_PC_SERVO_SIM_TEST
#define CFG_SCENE_PC_SERVO_SIM_TEST  0u
#endif

#ifndef CFG_SCENE_REAL_HARDWARE
#define CFG_SCENE_REAL_HARDWARE      0u
#endif

#if (CFG_SCENE_PC_SERVO_SIM_TEST != 0u) && (CFG_SCENE_REAL_HARDWARE != 0u)
#error "Choose only one scene: PC servo simulator OR real hardware."
#endif

#if CFG_SCENE_REAL_HARDWARE
#define CFG_SCENE_NAME               "REAL_HW"
#define CFG_DERIVED_MATRIX_GEOM_SIM  0u
#define CFG_DERIVED_ARM_MOTION_SIM   0u
#define CFG_DERIVED_CONVEYOR_SIM     0u
#define CFG_DERIVED_AUTO_START_KEY2  0u
#define CFG_DERIVED_STEP_HOLD_MS     0u
#elif CFG_SCENE_PC_SERVO_SIM_TEST
#define CFG_SCENE_NAME               "PC_SERVO_SIM"
#define CFG_DERIVED_MATRIX_GEOM_SIM  1u
#define CFG_DERIVED_ARM_MOTION_SIM   0u
#define CFG_DERIVED_CONVEYOR_SIM     1u
#define CFG_DERIVED_AUTO_START_KEY2  1u
#define CFG_DERIVED_STEP_HOLD_MS     500u
#else
#define CFG_SCENE_NAME               "SAFE_SIM"
#define CFG_DERIVED_MATRIX_GEOM_SIM  1u
#define CFG_DERIVED_ARM_MOTION_SIM   1u
#define CFG_DERIVED_CONVEYOR_SIM     1u
#define CFG_DERIVED_AUTO_START_KEY2  1u
#define CFG_DERIVED_STEP_HOLD_MS     500u
#endif

#ifndef CFG_MATRIX_GEOM_SIM_MODE
#define CFG_MATRIX_GEOM_SIM_MODE        CFG_DERIVED_MATRIX_GEOM_SIM
#endif

#ifndef CFG_ARM_MOTION_SIM_MODE
#define CFG_ARM_MOTION_SIM_MODE         CFG_DERIVED_ARM_MOTION_SIM
#endif

#ifndef CFG_CONVEYOR_SIM_MODE
#define CFG_CONVEYOR_SIM_MODE           CFG_DERIVED_CONVEYOR_SIM
#endif

#ifndef CFG_SORT_DEBUG_AUTO_START_AFTER_KEY2
#define CFG_SORT_DEBUG_AUTO_START_AFTER_KEY2 CFG_DERIVED_AUTO_START_KEY2
#endif

#ifndef CFG_SORT_DEBUG_STEP_HOLD_MS
#define CFG_SORT_DEBUG_STEP_HOLD_MS     CFG_DERIVED_STEP_HOLD_MS
#endif

#if CFG_MATRIX_RS485_USE_DE
#define CFG_MATRIX_RS485_DE_PORT        GPIOC
#define CFG_MATRIX_RS485_DE_PIN         GPIO_Pin_1
#define CFG_MATRIX_RS485_DE_CLK         RCC_AHB1Periph_GPIOC
#endif

/* --------- 归档：以太网 / lwIP（当前双 RS485 Keil 目标不编入；应用勿依赖下列宏）---------
 * 仅供历史对照或未编入工程的 net 源文件对齐；矩阵下发量产走 USART3 Modbus。
 */
#define CFG_NET_LOCAL_IP0            192u
#define CFG_NET_LOCAL_IP1            168u
#define CFG_NET_LOCAL_IP2            137u
#define CFG_NET_LOCAL_IP3            155u

#define CFG_NET_NETMASK0             255u
#define CFG_NET_NETMASK1             255u
#define CFG_NET_NETMASK2             255u
#define CFG_NET_NETMASK3             0u

#define CFG_NET_GW0                  192u
#define CFG_NET_GW1                  168u
#define CFG_NET_GW2                  137u
#define CFG_NET_GW3                  1u

/** 网卡 MAC（同一局域网内必须唯一；只改最后一位通常最安全） */
#define CFG_NET_MAC0                 0x02u
#define CFG_NET_MAC1                 0x00u
#define CFG_NET_MAC2                 0x00u
#define CFG_NET_MAC3                 0x00u
#define CFG_NET_MAC4                 0x00u
#define CFG_NET_MAC5                 0x01u

/** 归档：Matrix ASCII 服务端端口（未编入 lwIP 时不使用）。十进制 5000 */
#define CFG_NET_MATRIX_TCP_SERVER_PORT 5000u

/** 归档：网测探测目标（未编入 net 测试时不使用） */
#define CFG_NET_TEST_TARGET_IP0        192u
#define CFG_NET_TEST_TARGET_IP1        168u
#define CFG_NET_TEST_TARGET_IP2        137u
#define CFG_NET_TEST_TARGET_IP3        154u
#define CFG_NET_TEST_TARGET_TCP_PORT   CFG_NET_MATRIX_TCP_SERVER_PORT

/** PHY link 状态轮询周期 ms（只做链路灯/状态维护，与分拣节拍无关） */
#define CFG_NET_LINK_TIMER_INTERVAL_MS 1000u

/* --------- Matrix_Raw 文本行缓冲（app_protocol，与 ASCII CSV 列宽一致）---------
 * 改小则超长 CSV 行截断致整帧作废；改太大则 RAM 占用上升。
 */
#define CFG_APP_PROTO_RAW_LINE_CAP   192u

/* --------- main 里 fault/提示 snprintf 缓冲（KEY 演示报错用）--------- */
#define CFG_MAIN_UI_MSG_CAP          80u

#endif /* GLOBAL_CONFIG_H */
