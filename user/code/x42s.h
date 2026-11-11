#ifndef X42S_H
#define X42S_H

#include <stdbool.h>
#include <stdint.h>

/*
 * X 固件：F1 配置内部位置轨迹；FC 以带符号 0.1 度为单位
 * 提供绝对目标。不要使用 EMM 脉冲单位。
 */

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file x42s.h
 * @brief 张大头 ZDT X42S（X 固件）串口驱动。
 *
 * 当前移植使用 SysConfig 管理的硬件 UART0：
 * - SysConfig 实例：PRINT_INST（名称来自旧工程，实际用途为 X42S）
 * - 引脚：PA0（单片机 TX）和 PA1（单片机 RX）
 * - 串口格式：115200、8 数据位、无校验、1 停止位
 * - 协议校验字节：固定为 0x6B
 *
 * 接线方法：
 * - PA0/TX -> X42S RX（电机接收端）
 * - PA1/RX <- X42S TX（电机发送端）
 * - 单片机 GND 与 X42S GND 必须共地
 *
 * 电机菜单必须设置为：
 * - FWType = FW_X
 * - CtrlMode = CR_VFOC（闭环）
 * - P_Serial = UART_FUN
 * - P_Pul = PUL_OFF
 * - UartBaud = 115200
 * - ID = 1（或与 X42S_Init() 参数一致）
 * - Check = 0x6B
 * - Response = Receive
 * - S_PosTDP = Disable
 *
 * 本驱动按 S_PosTDP = Disable 换算位置，命令位置分辨率为 0.1°。
 * 所有发送接口均为阻塞式 UART 发送，只能在主循环或普通任务中调用，
 * 不要在定时器中断、编码器中断等快速中断中调用。
 *
 * 当前移植范围：
 * - 已移植单电机使能、停止、解除保护、当前位置清零；
 * - 已移植限速力矩、限流速度、直通位置、梯形位置和快速位置控制；
 * - 已移植常用状态反馈、选项读取、定时反馈、心跳和闭环切换；
 * - 厂家 X_V2 中的编码器校准、恢复出厂、完整回零参数、DMX512、
 *   电机内部 PID 参数修改和多电机命令暂未公开，因为平衡水管单电机控制
 *   不依赖这些接口。后续确有需求时应继续按手册逐项移植，而不是直接复制
 *   厂家依赖固定 UART 和延时收帧的实现。
 */

/* 电机状态标志：对应 0x3A 状态反馈中的每一个位。 */
#define X42S_STATUS_ENABLED (1U << 0)          /**< 电机已经使能。 */
#define X42S_STATUS_POSITION_REACHED (1U << 1) /**< 位置已经到达目标窗口。 */
#define X42S_STATUS_STALL (1U << 2)            /**< 当前检测到堵转。 */
#define X42S_STATUS_STALL_PROTECTION (1U << 3) /**< 已经进入堵转保护。 */
#define X42S_STATUS_LEFT_LIMIT (1U << 4)       /**< 左限位输入有效。 */
#define X42S_STATUS_RIGHT_LIMIT (1U << 5)      /**< 右限位输入有效。 */
#define X42S_STATUS_POWER_LOSS (1U << 7)       /**< 检测到掉电或母线异常。 */

/* 电机选项标志：对应 0x1A 选项参数反馈中的每一个位。 */
#define X42S_OPTION_MOTOR_09_DEG (1U << 0)     /**< 置位表示 0.9° 电机。 */
#define X42S_OPTION_FIRMWARE_EMM (1U << 1)     /**< 置位表示 Emm 固件；X 固件应为 0。 */
#define X42S_OPTION_CLOSED_LOOP (1U << 2)      /**< 置位表示闭环模式。 */
#define X42S_OPTION_DIRECTION_CCW (1U << 4)    /**< 置位表示默认方向为逆时针。 */
#define X42S_OPTION_BUTTON_LOCKED (1U << 5)    /**< 置位表示面板按键已锁定。 */
#define X42S_OPTION_POSITION_001_DEG (1U << 7) /**< 置位表示位置分辨率为 0.01°。 */
#define X42S_OPTION_LOCK_LEVEL_MASK (3U << 8)  /**< 新版固件的参数锁定等级。 */

/* valid_mask 标志：用于判断 X42S_State 中哪些反馈值已经收到。 */
#define X42S_VALID_POSITION (1UL << 0)       /**< 实时位置有效。 */
#define X42S_VALID_SPEED (1UL << 1)          /**< 实时转速有效。 */
#define X42S_VALID_POSITION_ERROR (1UL << 2) /**< 位置误差有效。 */
#define X42S_VALID_PHASE_CURRENT (1UL << 3)  /**< 相电流有效。 */
#define X42S_VALID_BUS_VOLTAGE (1UL << 4)    /**< 母线电压有效。 */
#define X42S_VALID_BUS_CURRENT (1UL << 5)    /**< 母线电流有效。 */
#define X42S_VALID_TEMPERATURE (1UL << 6)    /**< 电机温度有效。 */
#define X42S_VALID_STATUS (1UL << 7)         /**< 状态标志有效。 */
#define X42S_VALID_OPTIONS (1UL << 8)        /**< 选项参数有效。 */

    /**
 * @brief 位置命令的参考方式。
 *
 * 快速位置参数设置后，后续快速位置命令会沿用这里选择的参考方式。
 */
    typedef enum
    {
        X42S_POSITION_RELATIVE_LAST_TARGET = 0, /**< 相对上一次目标位置。 */
        X42S_POSITION_ABSOLUTE = 1,             /**< 绝对位置。 */
        X42S_POSITION_RELATIVE_CURRENT = 2      /**< 相对电机当前实际位置。 */
    } X42S_PositionMode;

    /**
 * @brief 可主动读取或设置为定时返回的反馈类型。
 */
    typedef enum
    {
        X42S_FEEDBACK_BUS_VOLTAGE = 0x24,    /**< 母线电压，单位 mV。 */
        X42S_FEEDBACK_BUS_CURRENT = 0x26,    /**< 母线电流，单位 mA。 */
        X42S_FEEDBACK_PHASE_CURRENT = 0x27,  /**< 电机相电流，单位 mA。 */
        X42S_FEEDBACK_SPEED = 0x35,          /**< 电机实时转速，单位 RPM。 */
        X42S_FEEDBACK_POSITION = 0x36,       /**< 电机实时位置，单位 °。 */
        X42S_FEEDBACK_POSITION_ERROR = 0x37, /**< 电机位置误差，单位 °。 */
        X42S_FEEDBACK_TEMPERATURE = 0x39,    /**< 电机实时温度，单位 ℃。 */
        X42S_FEEDBACK_STATUS = 0x3A          /**< 电机状态标志。 */
    } X42S_Feedback;

    /**
 * @brief 电机对控制命令返回的结果码。
 */
    typedef enum
    {
        X42S_REPLY_NONE = 0x00,              /**< 尚未收到命令应答。 */
        X42S_REPLY_OK = 0x02,                /**< 命令接收成功。 */
        X42S_REPLY_ALREADY_AT_ORIGIN = 0x12, /**< 已经处于原点。 */
        X42S_REPLY_LIMIT_ACTIVE = 0x22,      /**< 限位输入有效。 */
        X42S_REPLY_REACHED = 0x9F,           /**< 运动目标已经到达。 */
        X42S_REPLY_PARAMETER_ERROR = 0xE2,   /**< 命令参数错误。 */
        X42S_REPLY_FORMAT_ERROR = 0xEE       /**< 命令格式错误。 */
    } X42S_Reply;

    /**
 * @brief 电机接收状态与串口诊断信息。
 *
 * 使用 X42S_GetState() 获得只读指针；读取具体反馈值前，应先检查
 * valid_mask 中相应的 X42S_VALID_xxx 标志。
 */
    typedef struct
    {
        float position_deg;        /**< 电机实时位置，单位 °。 */
        float speed_rpm;           /**< 电机实时转速，单位 RPM。 */
        float position_error_deg;  /**< 电机位置误差，单位 °。 */
        uint16_t phase_current_ma; /**< 电机相电流，单位 mA。 */
        uint16_t bus_voltage_mv;   /**< 母线电压，单位 mV。 */
        uint16_t bus_current_ma;   /**< 母线电流，单位 mA。 */
        int16_t temperature_c;     /**< 电机温度，单位 ℃。 */
        uint8_t status_flags;      /**< 电机状态位，使用 X42S_STATUS_xxx 判断。 */
        uint16_t option_flags;     /**< 电机选项位，兼容新版 10 位选项返回。 */
        uint8_t last_function;     /**< 最近收到的一帧功能码。 */
        uint8_t last_reply;        /**< 最近收到的控制命令结果码。 */
        uint32_t valid_mask;       /**< 已经收到并解析成功的反馈类型集合。 */
        uint32_t raw_bytes_received; /**< UART0 实际收到并从软件 FIFO 取出的字节总数。 */
        uint32_t frames_received;          /**< 校验正确且成功解析的帧总数。 */
        uint32_t option_frames_received;   /**< 成功解析的 0x1A 选项帧总数。 */
        uint32_t position_frames_received; /**< 成功解析的 0x36 实时位置帧总数。 */
        uint32_t frame_errors;             /**< 帧长度或结尾校验错误次数。 */
        uint32_t rx_overflows;             /**< 软件接收环形 FIFO 溢出次数。 */
        uint8_t first_rx_bytes[8]; /**< 清空状态后最先收到的 8 个原始字节。 */
        uint8_t first_rx_count;    /**< first_rx_bytes 中已经保存的字节数。 */
    } X42S_State;

    /**
 * @brief 初始化 X42S 驱动和 UART0 接收中断。
 * @param address 电机串口地址；传入 0 时自动使用地址 1。
 * @note 本函数不会使能电机，也不会让电机运动。
 */
    void X42S_Init(uint8_t address);

    /**
 * @brief 从软件接收 FIFO 取出字节并解析完整协议帧。
 * @note 应在主循环中频繁调用，建议调用间隔不大于 10 ms。
 */
    void X42S_Process(void);

    /**
 * @brief 清空 UART0 硬件 FIFO、软件接收 FIFO 和未完成的协议帧。
 *
 * 适用于重新开始通信测试前丢弃历史反馈数据。调用期间会短暂关闭
 * UART0 的 NVIC 中断，清空后立即恢复，不会修改电机地址和反馈状态。
 */
    void X42S_FlushRx(void);

    /**
 * @brief UART0 接收中断处理入口。
 *
 * 该函数只负责清空 MSPM0 UART 硬件 FIFO，并把字节写入驱动内部的
 * 256 字节软件环形 FIFO；具体协议解析由 X42S_Process() 在主循环完成。
 */
    void X42S_RxIRQ(void);

    /**
 * @brief 获取电机反馈与串口诊断状态。
 * @return 指向驱动内部只读状态的指针。
 */
    const X42S_State *X42S_GetState(void);

    /**
 * @brief 清空已解析的反馈和诊断计数。
 */
    void X42S_ClearState(void);

    /**
 * @brief 使能或失能电机。
 * @param enable true 为使能，false 为失能。
 * @return true 表示命令已写入 UART；不表示电机已经执行成功。
 */
    bool X42S_Enable(bool enable);

    /**
 * @brief 立即停止当前运动。
 * @return true 表示命令已写入 UART。
 */
    bool X42S_Stop(void);

    /**
 * @brief 解除电机堵转保护。
 * @return true 表示命令已写入 UART。
 */
    bool X42S_ClearProtection(void);

    /** 仅本次上电会话选择 EMM 256 微步运行。 */
    bool X42S_SetMicrostep256(void);

    /**
 * @brief 把电机当前实际位置设置为零点。
 * @return true 表示命令已写入 UART。
 */
    bool X42S_ZeroCurrentPosition(void);

    /**
 * @brief 执行电机存储的绝对坐标回零（9A/04）。
 * @return true 表示命令已写入 UART；实际到位须通过位置反馈确认。
 * @note 与 X42S_ZeroCurrentPosition() 不同，本函数不会把当前位置设为零。
 */
    bool X42S_ReturnAbsoluteZero(void);

    /**
 * @brief 力矩模式控制，并限制最大转速。
 * @param current_ma 目标电流，正负号决定方向，范围 -5000~5000 mA。
 * @param current_ramp_ma_s 电流斜坡，单位 mA/s。
 * @param max_speed_rpm 最大转速，单位 RPM，绝对值不能超过 3000。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool
    X42S_SetTorqueLimitedSpeed(float current_ma, uint16_t current_ramp_ma_s, float max_speed_rpm);

    /**
 * @brief 速度模式控制，并限制最大相电流。
 * @param speed_rpm 目标转速，正负号决定方向，范围 -3000~3000 RPM。
 * @param acceleration_rpm_s 加速度，单位 RPM/s。
 * @param max_current_ma 最大相电流，范围 0~5000 mA。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetVelocityLimitedCurrent(float speed_rpm,
                                        uint16_t acceleration_rpm_s,
                                        uint16_t max_current_ma);

    /** Emm 固件速度命令 (0xF6)。带符号的 RPM 值决定方向；
 * 加速度等级 0 按厂商说明直接启动/停止。 */
    bool X42S_SetEmmVelocity(float speed_rpm, uint8_t acceleration_grade);

    /**
 * @brief 直通位置模式控制，并限制速度和相电流。
 * @param position_deg 目标角度；正负号决定运动方向。
 * @param max_speed_rpm 最大转速，单位 RPM。
 * @param mode 位置参考方式。
 * @param max_current_ma 最大相电流，范围 0~5000 mA。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetBypassPosition(float position_deg,
                                float max_speed_rpm,
                                X42S_PositionMode mode,
                                uint16_t max_current_ma);

    /**
 * @brief 梯形加减速位置模式控制，并限制速度和相电流。
 * @param position_deg 目标角度；正负号决定运动方向。
 * @param acceleration_rpm_s 加速度，单位 RPM/s。
 * @param deceleration_rpm_s 减速度，单位 RPM/s。
 * @param max_speed_rpm 最大转速，单位 RPM。
 * @param mode 位置参考方式。
 * @param max_current_ma 最大相电流，范围 0~5000 mA。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetTrapezoidPosition(float position_deg,
                                   uint16_t acceleration_rpm_s,
                                   uint16_t deceleration_rpm_s,
                                   float max_speed_rpm,
                                   X42S_PositionMode mode,
                                   uint16_t max_current_ma);

    /**
 * @brief 设置快速梯形位置模式的公共运动参数。
 *
 * 参数只需在控制开始时设置一次。为避免快速 FC 帧丢失位置参考方式，
 * X42S_SetQuickPosition() 在本工程中会用完整 CD 帧重发这些参数和目标；
 * 这使每个平衡 PID 输出都是明确的绝对位置命令。
 *
 * @param acceleration_rpm_s 加速度，单位 RPM/s。
 * @param deceleration_rpm_s 减速度，单位 RPM/s。
 * @param max_speed_rpm 最大转速，单位 RPM。
 * @param mode 位置参考方式；平衡控制通常使用绝对位置。
 * @param max_current_ma 最大相电流，范围 0~5000 mA。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetQuickPositionParams(uint16_t acceleration_rpm_s,
                                     uint16_t deceleration_rpm_s,
                                     float max_speed_rpm,
                                     X42S_PositionMode mode,
                                     uint16_t max_current_ma);

    /**
 * @brief 更新平衡控制的目标角度（兼容旧接口，底层发送完整 CD 绝对位置帧）。
 * @param position_deg 目标角度，支持正负角度，单位 °。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetQuickPosition(float position_deg);

    /**
 * @brief 主动请求一次指定类型的电机反馈。
 * @param feedback 需要读取的反馈类型。
 * @return 反馈类型有效且命令成功写入 UART 时返回 true。
 */
    bool X42S_RequestFeedback(X42S_Feedback feedback);

    /**
 * @brief 读取电机固件、闭环模式、方向等选项状态。
 * @return true 表示命令已写入 UART。
 */
    bool X42S_RequestOptions(void);

    /**
 * @brief 设置电机定时返回某一种反馈。
 * @param feedback 需要定时返回的反馈类型。
 * @param period_ms 返回周期，单位 ms；主循环为 10 ms 时建议不小于 20 ms。
 * @return 参数合法且命令成功写入 UART 时返回 true。
 */
    bool X42S_SetTimedFeedback(X42S_Feedback feedback, uint16_t period_ms);

    /**
 * @brief 设置串口心跳保护超时时间。
 * @param timeout_ms 超时时间，单位 ms。
 * @param save true 表示保存到电机非易失存储器，false 表示仅本次上电有效。
 * @return true 表示命令已写入 UART。
 */
    bool X42S_SetHeartbeat(uint32_t timeout_ms, bool save);

    /**
 * @brief 通过串口切换开环或闭环模式。
 * @param closed_loop true 为闭环，false 为开环。
 * @param save true 表示保存到电机非易失存储器，false 表示仅本次上电有效。
 * @return true 表示命令已写入 UART。
 * @warning 运行中不要随意切换控制模式；首次配置建议通过电机菜单完成。
 */
    bool X42S_SetClosedLoop(bool closed_loop, bool save);

#ifdef __cplusplus
}
#endif

#endif
