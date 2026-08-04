#ifndef PUMP_DRIVE_H
#define PUMP_DRIVE_H

#include <stdint.h>
#include "bsp_usart.h"

/**
 * @file pump_drive.h
 * @brief ISC1000 微型一体化步进电机控制驱动器协议层。
 *
 * 本文件封装 ISC1000 的上位机控制协议：
 * - 发送方向：按照手册生成小写 ASCII 命令，并自动追加换行符 `\n`；
 * - 接收方向：解析驱动器返回的二进制反馈帧；
 * - 串口资源：固定使用 USART3，不占用 USART2 或其它串口。
 *
 * @note 底层 USART3 的 DMA + IDLE 接收由 bsp_usart 管理，pump_drive 只注册回调并解析数据。
 */

/** ISC1000 默认接收反馈的最大长度，不能超过 bsp_usart 的 USART_RXBUFF_LIMIT。 */
#define PUMP_DRIVE_RX_BUFFER_SIZE      256U

/** 保存握手、参数配置、离线时序脚本等文本反馈的最大长度。 */
#define PUMP_DRIVE_TEXT_MAX            220U

/** 调试用：保存最近一次 USART3 原始接收数据的最大字节数。 */
#define PUMP_DRIVE_RAW_RX_DEBUG_SIZE   32U

/** 调试用：保存最近一次 USART3 发送命令文本的最大字节数。 */
#define PUMP_DRIVE_LAST_TX_DEBUG_SIZE  48U

/** 当前设备最多挂接两台 ISC1000：泵1为 300ul，泵2为 100ul。 */
#define PUMP_DRIVE_MAX_INSTANCE        2U

/** 泵1：RS485 ID=1，对应 300ul 定量泵。 */
#define PUMP_DRIVE_PUMP1_DEVICE_ID     1U

/** 泵2：RS485 ID=2，对应 100ul 定量泵。 */
#define PUMP_DRIVE_PUMP2_DEVICE_ID     2U

/** 泵1满行程体积，单位 ul。 */
#define PUMP_DRIVE_PUMP1_FULL_STROKE_UL 300U

/** 泵2满行程体积，单位 ul。 */
#define PUMP_DRIVE_PUMP2_FULL_STROKE_UL 100U

/** 两台定量泵满行程命令步数。 */
#define PUMP_DRIVE_FULL_STROKE_STEPS   1600U

/**
 * 定量泵电机每转一圈对应的 ISC1000 in/out 命令步数。
 *
 * 这个常量只用于按角度/按圈点动；按体积发药仍使用满行程体积和满行程步数标定。
 * 当前现场 `in 10/out 10` 实测不到 10 度，先按 400 步/圈处理。
 * 如果 10 圈仍有偏差，只需要重新标定这个常量。
 */
#define PUMP_DRIVE_STEPS_PER_TURN      400U

/** ISC1000 单条 in/out 命令最大步数，超过该值需要业务层分段执行。 */
#define PUMP_DRIVE_COMMAND_MAX_STEPS    60000U

/** 定量泵运动中查询 `sta` 的间隔，单位 ms。 */
#define PUMP_DRIVE_STATUS_QUERY_INTERVAL_MS 100U

/** Busy 变为 0 后额外等待机械和流体稳定的时间，单位 ms。 */
#define PUMP_DRIVE_MOVE_STABLE_DELAY_MS  100U

/**
 * 定量泵运动完成兜底估算速度，单位 PPS。
 *
 * 正常流程优先等待 `sta` 返回 Busy=0；这个值只在 `sta` 没有被成功解析时使用，
 * 用一个偏慢的速度估算最大等待时间，避免整机流程永久卡住。
 */
#define PUMP_DRIVE_MOVE_TIMEOUT_ESTIMATE_PPS 200U

/** 定量泵运动完成兜底等待的额外余量，单位 ms。 */
#define PUMP_DRIVE_MOVE_TIMEOUT_MARGIN_MS 2000U

/** 定量泵运动完成兜底等待的最小时间，单位 ms。 */
#define PUMP_DRIVE_MOVE_TIMEOUT_MIN_MS 3000U

/**
 * 光电门每转一圈产生的下降沿数量。
 *
 * 当前先按“一圈一个遮光下降沿”处理；如果实测一圈有多个缺口，
 * 只需要改这个常量，圈数换算接口不用跟着改。
 */
#define PUMP_DRIVE_SENSOR_PULSES_PER_TURN 1U

/** RS485 一问一答保护超时时间，单位 ms。 */
#define PUMP_DRIVE_BUS_TIMEOUT_MS      100U

/** ISC1000 二进制反馈帧固定帧头，所有反馈帧都以 0xFF 开始。 */
#define PUMP_DRIVE_FRAME_HEADER        0xFFU

/** ISC1000 二进制反馈帧固定帧尾，除不完整数据外，所有反馈帧都以 0xFE 结束。 */
#define PUMP_DRIVE_FRAME_TAIL          0xFEU

/** 状态位 Bit0：传感器 1 是否触发，1 表示触发。 */
#define PUMP_DRIVE_STATUS_SENSOR1      (1U << 0)

/** 状态位 Bit1：电机是否忙碌运行，1 表示 Busy/运行中。 */
#define PUMP_DRIVE_STATUS_BUSY         (1U << 1)

/** 状态位 Bit2：传感器 2 是否触发，1 表示触发。 */
#define PUMP_DRIVE_STATUS_SENSOR2      (1U << 2)

/** 状态位 Bit3：离线时序是否正在运行，1 表示正在执行离线动作。 */
#define PUMP_DRIVE_STATUS_AUTORUN      (1U << 3)

/** 状态位 Bit4：电机是否使能，1 表示 Enable 已置位。 */
#define PUMP_DRIVE_STATUS_ENABLE       (1U << 4)

/** 状态位 Bit5：输出通道 1 / 阀 1 是否打开，1 表示打开。 */
#define PUMP_DRIVE_STATUS_VALVE1       (1U << 5)

/** 状态位 Bit6：输出通道 2 / 阀 2 是否打开，1 表示打开。 */
#define PUMP_DRIVE_STATUS_VALVE2       (1U << 6)

/**
 * @brief ISC1000 通讯总线模式。
 *
 * ISC1000 同一套 ASCII 命令可工作在 RS232 或 RS485 上：
 * - RS232：直接发送命令，例如 `hsk\n`；
 * - RS485：命令前追加设备 ID，例如 `1 hsk\n`。
 */
typedef enum
{
    PUMP_DRIVE_BUS_RS232 = 0, /**< RS232 点对点模式：发送命令时不添加设备 ID。 */
    PUMP_DRIVE_BUS_RS485,     /**< RS485 总线模式：发送命令时在最前面添加设备 ID 和空格。 */
} PumpDriveBusMode_e;

/**
 * @brief ISC1000 下位机反馈号。
 *
 * 反馈号位于二进制反馈帧的第 3 个字节，用于判断数据区的含义和解析方式。
 */
typedef enum
{
    PUMP_DRIVE_FB_NONE           = 0x00U, /**< 无有效反馈；驱动初始化或尚未解析成功时使用。 */
    PUMP_DRIVE_FB_HANDSHAKE      = 0x01U, /**< 握手反馈；数据区为型号、软件版本、发布日期等 ASCII 文本。 */
    PUMP_DRIVE_FB_STATUS         = 0x02U, /**< 运行状态反馈；数据区为 5 字节 7-bit 编码的 32 位状态值。 */
    PUMP_DRIVE_FB_PARAM_CONFIG   = 0x03U, /**< 参数配置反馈；数据区为 `id=... spd=...` 形式的 ASCII 文本。 */
    PUMP_DRIVE_FB_SEQUENCE_DATA  = 0x04U, /**< 离线时序步骤反馈；数据区为 `|` 分隔的 ASCII 文本，且此帧无 BCC。 */
    PUMP_DRIVE_FB_SEQUENCE_COUNT = 0x05U, /**< 时序运行次数反馈；数据区为 5 字节 7-bit 编码的 32 位计数值。 */
} PumpDriveFeedbackType_e;

/**
 * @brief ISC1000 本次运动方向。
 *
 * 方向只用于调试和状态记录，不参与协议解析：
 * - IN：吸入方向；
 * - OUT：排出方向。
 */
typedef enum
{
    PUMP_DRIVE_MOVE_DIR_NONE = 0, /**< 当前没有正在跟踪的运动。 */
    PUMP_DRIVE_MOVE_DIR_IN,       /**< 本次运动为吸入方向。 */
    PUMP_DRIVE_MOVE_DIR_OUT,      /**< 本次运动为排出方向。 */
} PumpDriveMoveDir_e;

/**
 * @brief ISC1000 本次运动完成状态。
 *
 * 运动完成不再只靠软件估算时间，而是由 `sta` 的 Busy 位确认：
 * - RUNNING：已下发 in/out，正在周期查询 Busy；
 * - STABILIZING：Busy 已经为 0，正在等待机械稳定延时；
 * - DONE：本次运动已确认结束，上层可以进入下一步。
 */
typedef enum
{
    PUMP_DRIVE_MOVE_IDLE = 0,  /**< 空闲，没有正在跟踪的运动。 */
    PUMP_DRIVE_MOVE_RUNNING,   /**< 运动中，等待 `sta` 返回 Busy=0。 */
    PUMP_DRIVE_MOVE_STABILIZING, /**< 电气停止，等待机械和管路压力稳定。 */
    PUMP_DRIVE_MOVE_DONE,      /**< 本次运动完成。 */
} PumpDriveMoveState_e;

/**
 * @brief ISC1000 反馈帧解析错误码。
 *
 * last_error 用于记录最近一次解析失败原因，便于调试线缆、校验、帧格式问题。
 */
typedef enum
{
    PUMP_DRIVE_ERR_NONE = 0,          /**< 无错误，最近一次反馈帧解析成功。 */
    PUMP_DRIVE_ERR_SHORT_FRAME,       /**< 数据长度不足，无法组成完整反馈帧。 */
    PUMP_DRIVE_ERR_BAD_TAIL,          /**< 未找到帧尾 0xFE，或帧尾位置不符合当前反馈类型。 */
    PUMP_DRIVE_ERR_BAD_BCC,           /**< BCC 异或校验失败，说明数据可能损坏或校验字节顺序不匹配。 */
    PUMP_DRIVE_ERR_UNSUPPORTED_TYPE,  /**< 反馈号不是当前驱动支持的 0x01~0x05。 */
} PumpDriveError_e;

/**
 * @brief `sta` 运行状态反馈解析结果。
 *
 * ISC1000 的 `sta` 指令返回反馈号 0x02，数据区为 5 字节编码的 32 位整数。
 * 低 16 位是状态位，高 16 位按手册说明预留。
 */
typedef struct
{
    uint16_t raw_flags;  /**< 原始低 16 位状态位，保留完整位图供上层做额外判断。 */
    int16_t reserved;    /**< 高 16 位预留字段，按手册说明保留为有符号值。 */
    uint8_t sensor1;     /**< 传感器 1 触发状态：0 未触发，1 已触发。 */
    uint8_t busy;        /**< 电机运行状态：0 空闲，1 忙碌/运行中。 */
    uint8_t sensor2;     /**< 传感器 2 触发状态：0 未触发，1 已触发。 */
    uint8_t autorun;     /**< 离线时序状态：0 未运行，1 正在运行。 */
    uint8_t enabled;     /**< 电机使能状态：0 脱机/失能，1 使能。 */
    uint8_t valve1;      /**< 输出通道 1 / 阀 1 状态：0 关闭，1 打开。 */
    uint8_t valve2;      /**< 输出通道 2 / 阀 2 状态：0 关闭，1 打开。 */
} PumpDriveStatus_s;

/**
 * @brief ISC1000 驱动实例。
 *
 * 当前工程用 USART3 挂一条 RS485 总线，总线上允许存在两台不同 ID 的 ISC1000。
 * 每个 PumpDrive_s 代表一台驱动器，接收回调会按反馈帧中的设备 ID 分发状态。
 */
typedef struct
{
    USARTInstance *usart;                     /**< bsp_usart 注册得到的 USART3 实例。 */
    uint8_t device_id;                        /**< RS485 设备地址；RS232 模式下仅用于记录反馈 ID。 */
    PumpDriveBusMode_e bus_mode;              /**< 当前通讯模式：RS232 或 RS485。 */
    PumpDriveFeedbackType_e last_type;        /**< 最近一次成功解析到的反馈号。 */
    PumpDriveError_e last_error;              /**< 最近一次解析错误码，成功时为 PUMP_DRIVE_ERR_NONE。 */
    PumpDriveStatus_s status;                 /**< 最近一次 `sta` 指令反馈解析出的运行状态。 */
    uint32_t last_value;                      /**< 最近一次 5 字节数值反馈的原始 32 位值。 */
    uint32_t sequence_count;                  /**< `act 0 rdcnt` 返回的离线时序运行次数。 */
    uint8_t frame_ready;                      /**< 成功解析到一帧后置 1，上层处理完可手动清 0。 */
    uint8_t last_rx_id;                       /**< 最近一次反馈帧中的设备 ID。 */
    uint32_t raw_rx_count;                    /**< USART3 原始接收次数计数，用于判断驱动器是否真的有回包。 */
    uint16_t last_raw_rx_total_len;           /**< 最近一次 USART3 实际收到的总长度，可能大于调试缓存。 */
    uint16_t last_raw_rx_len;                 /**< last_raw_rx 中实际保存的字节数。 */
    uint8_t last_raw_rx[PUMP_DRIVE_RAW_RX_DEBUG_SIZE]; /**< 最近一次 USART3 原始接收字节，调试帧头/帧尾/ID 用。 */
    uint16_t last_tx_len;                     /**< 最近一次尝试发送的命令长度，包含 RS485 ID 和换行。 */
    uint8_t last_tx_result;                   /**< 最近一次命令是否真正发出：1 已发送，0 被参数/总线状态拦截。 */
    uint32_t last_volume_ul;                  /**< 最近一次按体积运动的目标体积，单位 ul。 */
    uint32_t last_volume_steps;               /**< 最近一次按体积运动换算出的目标步数。 */
    char last_tx_command[PUMP_DRIVE_LAST_TX_DEBUG_SIZE]; /**< 最近一次命令文本，例如 `2 in 1600\n`。 */
    uint32_t full_stroke_ul;                  /**< 该泵满行程体积，单位 ul。 */
    uint32_t full_stroke_steps;               /**< 该泵满行程命令步数。 */
    volatile uint32_t photo_count;            /**< 光电门下降沿累计计数，在 EXTI 中断里递增。 */
    uint32_t move_start_photo_count;          /**< 本次运动开始时的光电门计数，用于计算本次转过多少圈。 */
    uint32_t move_photo_count;                /**< 本次运动完成时的光电门计数差值。 */
    uint32_t move_target_steps;               /**< 本次 in/out 下发的目标步数。 */
    uint32_t move_start_ms;                   /**< 本次运动命令成功发送的起始时间。 */
    uint32_t move_timeout_ms;                 /**< 按目标步数估算出的最大等待时间。 */
    uint32_t move_last_query_ms;              /**< 最近一次查询 `sta` 的时间。 */
    uint32_t move_stable_start_ms;            /**< Busy 变为 0 后进入机械稳定延时的起始时间。 */
    PumpDriveMoveDir_e move_dir;              /**< 本次运动方向，仅用于调试显示和后续扩展。 */
    PumpDriveMoveState_e move_state;          /**< 本次运动完成状态，由 Busy 轮询和稳定延时维护。 */
    uint8_t move_timeout_fallback;            /**< 本次运动是否因为超过最大等待时间而兜底完成。 */
    uint8_t sensor_level_configured;          /**< 上电后是否已发送 `set stl=0`。 */
    uint16_t text_len;                        /**< text_feedback 中的有效字符数量，不含结尾 `\0`。 */
    char text_feedback[PUMP_DRIVE_TEXT_MAX];  /**< 握手、配置、离线时序脚本等 ASCII 文本反馈。 */
} PumpDrive_s;

/**
 * @brief 初始化 ISC1000 驱动，并固定注册 USART3。
 *
 * @param pump 驱动实例指针，函数内部会清零并初始化该结构体。
 * @param mode 通讯模式：RS232 不添加 ID，RS485 会在命令前添加 ID。
 * @param device_id RS485 设备 ID，范围 1~99；广播发送可使用 0，RS232 模式下只记录该值。
 * @return 1 表示初始化成功；0 表示参数为空、USART3 已被注册或底层 USART 注册失败。
 *
 * @note 本函数只注册 USART3，不会占用 USART2 或其它串口。
 */
uint8_t PumpDrive_Init(PumpDrive_s *pump, PumpDriveBusMode_e mode, uint8_t device_id);

/**
 * @brief 初始化本机两台定量泵。
 *
 * @return 1 表示两台泵都已经注册完成；0 表示 USART3 注册失败或 ID 冲突。
 *
 * @note 当前硬件约定：泵1=ID1=300ul，泵2=ID2=100ul，两台泵共用 USART3 RS485 总线。
 */
uint8_t PumpDrive_BoardInit(void);

/**
 * @brief 获取泵1实例。
 *
 * @return 初始化完成后返回泵1实例；未初始化时返回 NULL。
 *
 * @note 泵1对应上位机 OBJ=PUMP_1，硬件为 RS485 ID=1 的 300ul 定量泵。
 */
PumpDrive_s *PumpDrive_GetPump1(void);

/**
 * @brief 获取泵2实例。
 *
 * @return 初始化完成后返回泵2实例；未初始化时返回 NULL。
 *
 * @note 泵2对应上位机 OBJ=PUMP_2，硬件为 RS485 ID=2 的 100ul 定量泵。
 */
PumpDrive_s *PumpDrive_GetPump2(void);

/**
 * @brief 按 RS485 设备 ID 获取定量泵实例。
 *
 * @param device_id RS485 设备 ID。
 * @return 找到时返回对应泵实例；找不到时返回 NULL。
 */
PumpDrive_s *PumpDrive_GetByDeviceId(uint8_t device_id);

/**
 * @brief ISC1000 总线周期维护。
 *
 * @note 需要在 ModuleTask 中周期调用，用于释放 RS485 等待、配置 stl、轮询 Busy 并完成机械稳定延时。
 */
void PumpDrive_Process(void);

/**
 * @brief 光电门 EXTI 中断分发入口。
 *
 * @param GPIO_Pin HAL 传入的触发引脚。
 *
 * @note 当前硬件约定：
 *       - PB15 接泵1白线；
 *       - PB14 接泵2白线；
 *       - 白线遮光时为低电平，因此 GPIO 配置为下降沿计数。
 */
void PumpDrive_PhotoSensorIrqHandler(uint16_t GPIO_Pin);

/**
 * @brief 发送一条 ISC1000 原始 ASCII 命令。
 *
 * @param pump 驱动实例指针，必须先调用 PumpDrive_Init()。
 * @param command 不包含结尾换行符的命令正文，例如 `"hsk"`、`"set spd=2000"`。
 * @return 1 表示命令已交给 USART3 阻塞发送；0 表示参数错误或命令过长。
 *
 * @note 函数会自动追加 `\n`。RS485 模式下还会自动添加 `device_id` 前缀。
 */
uint8_t PumpDrive_SendCommand(PumpDrive_s *pump, const char *command);

/**
 * @brief 发送握手命令 `hsk`，用于读取型号、版本和发布日期。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Handshake(PumpDrive_s *pump);

/**
 * @brief 发送状态查询命令 `sta`，用于读取传感器、电机忙碌、使能和阀状态。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryStatus(PumpDrive_s *pump);

/**
 * @brief 发送参数查询命令 `cfg`，用于读取当前全部参数配置文本。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_QueryConfig(PumpDrive_s *pump);

/**
 * @brief 发送保存命令 `sav`，将当前参数写入驱动器 Flash。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 手册说明保存耗时约 10ms；id、bdr 等参数保存后通常需要重启生效。
 */
uint8_t PumpDrive_SaveConfig(PumpDrive_s *pump);

/**
 * @brief 设置传感器 1 触发电平 `stl`。
 *
 * @param pump 驱动实例指针。
 * @param trigger_high_level 0 表示遮光/触发为低电平；1 表示遮光/触发为高电平。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 当前接的是白线，遮光时 ON=低电平，所以应发送 `set stl=0`。
 *       本函数只写驱动器 RAM，不会发送 `sav` 写 Flash。
 */
uint8_t PumpDrive_SetSensorTriggerLevel(PumpDrive_s *pump, uint8_t trigger_high_level);

/**
 * @brief 发送电机使能命令 `on`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Enable(PumpDrive_s *pump);

/**
 * @brief 发送电机脱机命令 `off`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Disable(PumpDrive_s *pump);

/**
 * @brief 发送停止命令 `stp`。
 * @param pump 驱动实例指针。
 * @param emergency 停止方式：0 表示减速停止；非 0 表示立即急停。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_Stop(PumpDrive_s *pump, uint8_t emergency);

/**
 * @brief 发送复位归零命令 `rst`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 驱动器内部使用 Sensor1 执行原点搜索、脱离原点区和安全偏置动作。
 */
uint8_t PumpDrive_ResetHome(PumpDrive_s *pump);

/**
 * @brief 发送吸入/反向运动命令 `in N`。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveIn(PumpDrive_s *pump, uint32_t steps);

/**
 * @brief 发送排出/正向运动命令 `out N`。
 * @param pump 驱动实例指针。
 * @param steps 运动整步数，手册范围 0~60000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_MoveOut(PumpDrive_s *pump, uint32_t steps);

/**
 * @brief 判断最近一次 in/out 运动是否已经完成。
 *
 * @param pump 驱动实例指针。
 * @return 1 表示空闲或本次运动已完成；0 表示仍在运动或机械稳定延时中。
 */
uint8_t PumpDrive_IsMoveDone(PumpDrive_s *pump);

/**
 * @brief 读取光电门下降沿累计计数。
 *
 * @param pump 驱动实例指针。
 * @return 从上电或最近一次清零以来累计到的下降沿数量。
 */
uint32_t PumpDrive_GetPhotoCount(PumpDrive_s *pump);

/**
 * @brief 清零光电门累计计数。
 *
 * @param pump 驱动实例指针。
 */
void PumpDrive_ResetPhotoCount(PumpDrive_s *pump);

/**
 * @brief 读取本次运动产生的光电门计数差值。
 *
 * @param pump 驱动实例指针。
 * @return 最近一次完成的 in/out 动作期间累计到的下降沿数量。
 */
uint32_t PumpDrive_GetMovePhotoCount(PumpDrive_s *pump);

/**
 * @brief 读取本次运动圈数，结果放大 1000 倍。
 *
 * @param pump 驱动实例指针。
 * @return 圈数 * 1000，例如返回 2500 表示 2.500 圈。
 */
uint32_t PumpDrive_GetMoveTurnsX1000(PumpDrive_s *pump);

/**
 * @brief 设置泵体标定参数。
 *
 * @param pump 驱动实例指针。
 * @param full_stroke_ul 满行程体积，单位 ul。
 * @param full_stroke_steps 满行程命令步数。
 *
 * @note 当前默认：泵1=300ul/1600步，泵2=100ul/1600步。
 */
void PumpDrive_SetCalibration(PumpDrive_s *pump, uint32_t full_stroke_ul, uint32_t full_stroke_steps);

/**
 * @brief 将吸入/排出体积换算成命令步数。
 *
 * @param pump 驱动实例指针。
 * @param volume_ul 目标体积，单位 ul。
 * @return 四舍五入后的命令步数。
 */
uint32_t PumpDrive_VolumeUlToSteps(PumpDrive_s *pump, uint32_t volume_ul);

/**
 * @brief 将旋转角度换算成命令步数。
 *
 * @param pump 驱动实例指针。
 * @param angle_deg_x10 角度放大 10 倍，例如 900 表示 90.0 度。
 * @return 四舍五入后的命令步数。
 */
uint32_t PumpDrive_AngleDegX10ToSteps(PumpDrive_s *pump, uint32_t angle_deg_x10);

/**
 * @brief 将转速换算成 ISC1000 的 spd 参数。
 *
 * @param pump 驱动实例指针。
 * @param rpm_x10 转速放大 10 倍，例如 3000 表示 300.0 RPM。
 * @return 四舍五入后的 PPS。
 */
uint32_t PumpDrive_RpmX10ToPps(PumpDrive_s *pump, uint32_t rpm_x10);

/**
 * @brief 按 RPM 设置运行速度。
 *
 * @param pump 驱动实例指针。
 * @param rpm_x10 转速放大 10 倍，例如 3000 表示 300.0 RPM。
 * @return 1 表示发送成功；0 表示参数错误或总线忙。
 */
uint8_t PumpDrive_SetSpeedRpmX10(PumpDrive_s *pump, uint32_t rpm_x10);

/**
 * @brief 按体积吸入液体。
 *
 * @param pump 驱动实例指针。
 * @param volume_ul 目标体积，单位 ul。
 * @return 1 表示发送成功；0 表示参数错误或总线忙。
 */
uint8_t PumpDrive_MoveInVolumeUl(PumpDrive_s *pump, uint32_t volume_ul);

/**
 * @brief 按体积排出液体。
 *
 * @param pump 驱动实例指针。
 * @param volume_ul 目标体积，单位 ul。
 * @return 1 表示发送成功；0 表示参数错误或总线忙。
 */
uint8_t PumpDrive_MoveOutVolumeUl(PumpDrive_s *pump, uint32_t volume_ul);

/**
 * @brief 按角度执行吸入方向运动。
 *
 * @param pump 驱动实例指针。
 * @param angle_deg_x10 角度放大 10 倍。
 * @return 1 表示发送成功；0 表示参数错误或总线忙。
 */
uint8_t PumpDrive_MoveInAngleDegX10(PumpDrive_s *pump, uint32_t angle_deg_x10);

/**
 * @brief 按角度执行排出方向运动。
 *
 * @param pump 驱动实例指针。
 * @param angle_deg_x10 角度放大 10 倍。
 * @return 1 表示发送成功；0 表示参数错误或总线忙。
 */
uint8_t PumpDrive_MoveOutAngleDegX10(PumpDrive_s *pump, uint32_t angle_deg_x10);

/**
 * @brief 控制输出通道 1 / 阀 1。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve1(PumpDrive_s *pump, uint8_t on);

/**
 * @brief 控制输出通道 2 / 阀 2。
 * @param pump 驱动实例指针。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetValve2(PumpDrive_s *pump, uint8_t on);

/**
 * @brief 设置一个整数型参数，发送格式为 `set key=value`。
 * @param pump 驱动实例指针。
 * @param key 参数键名，例如 `"spd"`、`"acc"`、`"dec"`、`"id"`。
 * @param value 参数值，按手册范围传入。
 * @return 1 表示发送成功；0 表示参数错误或发送失败。
 */
uint8_t PumpDrive_SetParamInt(PumpDrive_s *pump, const char *key, int32_t value);

/**
 * @brief 设置目标运行速度 `spd`。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetSpeed(PumpDrive_s *pump, uint32_t speed_pps);

/**
 * @brief 同时设置目标速度、加速度和减速度。
 * @param pump 驱动实例指针。
 * @param speed_pps 目标速度，单位 PPS，手册范围 1~8000。
 * @param acc_pps2 加速度，单位 PPS^2，手册范围 1~80000。
 * @param dec_pps2 减速度，单位 PPS^2，手册范围 1~80000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_SetMotion(PumpDrive_s *pump, uint32_t speed_pps, uint32_t acc_pps2, uint32_t dec_pps2);

/**
 * @brief 设置运行电流 `crn` 和保持电流 `crh`。
 * @param pump 驱动实例指针。
 * @param run_current_ca 运行电流，单位厘安培；150 表示 1.50A。
 * @param hold_current_ca 保持电流，单位厘安培；20 表示 0.20A。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 使用厘安培是为了避免在嵌入式 printf 中启用浮点格式化。
 */
uint8_t PumpDrive_SetCurrentCentiAmp(PumpDrive_s *pump, uint16_t run_current_ca, uint16_t hold_current_ca);

/**
 * @brief 启动离线时序运行，发送 `act 0 start`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActStart(PumpDrive_s *pump);

/**
 * @brief 设置离线时序循环运行次数，发送 `act 0 times N`。
 * @param pump 驱动实例指针。
 * @param times 循环次数，手册范围 1~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActTimes(PumpDrive_s *pump, uint32_t times);

/**
 * @brief 清空离线时序脚本，发送 `act 0 clear`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActClear(PumpDrive_s *pump);

/**
 * @brief 读取离线时序脚本，发送 `act 0 get`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 *
 * @note 成功反馈为 0x04，文本中多个步骤用 `|` 分隔，且该反馈帧无 BCC 校验字节。
 */
uint8_t PumpDrive_ActGet(PumpDrive_s *pump);

/**
 * @brief 读取离线时序已运行次数，发送 `act 0 rdcnt`。
 * @param pump 驱动实例指针。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActReadCount(PumpDrive_s *pump);

/**
 * @brief 设置离线时序运行计数起始值，发送 `act 0 wrcnt A`。
 * @param pump 驱动实例指针。
 * @param count 起始计数值，手册范围 0~10000000。
 * @return 1 表示发送成功；0 表示发送失败。
 */
uint8_t PumpDrive_ActWriteCount(PumpDrive_s *pump, uint32_t count);

/**
 * @brief 添加/修改离线时序中的复位步骤，发送 `act X reset`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepReset(PumpDrive_s *pump, uint8_t step_index);

/**
 * @brief 添加/修改离线时序中的延时步骤，发送 `act X delay A`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param delay_ms 延时时间，单位 ms，手册范围 1~60000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepDelay(PumpDrive_s *pump, uint8_t step_index, uint32_t delay_ms);

/**
 * @brief 添加/修改离线时序中的吸入步骤，发送 `act X in step=A speed=B`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 吸入整步数，手册范围 1~60000。
 * @param speed_pps 运动速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveIn(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps);

/**
 * @brief 添加/修改离线时序中的排出步骤，发送 `act X out step=A speed=B`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param steps 排出整步数，手册范围 1~60000。
 * @param speed_pps 运动速度，单位 PPS，手册范围 1~8000。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepMoveOut(PumpDrive_s *pump, uint8_t step_index, uint32_t steps, uint32_t speed_pps);

/**
 * @brief 添加/修改离线时序中的阀 1 控制步骤，发送 `act X v1 Switch`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve1(PumpDrive_s *pump, uint8_t step_index, uint8_t on);

/**
 * @brief 添加/修改离线时序中的阀 2 控制步骤，发送 `act X v2 Switch`。
 * @param pump 驱动实例指针。
 * @param step_index 步骤序号，范围 1~60。
 * @param on 0 表示关闭；非 0 表示打开。
 * @return 1 表示发送成功；0 表示步骤序号非法或发送失败。
 */
uint8_t PumpDrive_ActStepValve2(PumpDrive_s *pump, uint8_t step_index, uint8_t on);

/**
 * @brief 解析一段来自 ISC1000 的二进制反馈流。
 * @param pump 驱动实例指针。
 * @param data 接收到的数据缓冲区，可以包含噪声、半帧或多帧数据。
 * @param len 数据长度，单位 byte。
 * @return 成功解析出的反馈帧数量。
 *
 * @note 解析成功后会更新 pump->last_type、pump->status、pump->text_feedback 等字段。
 */
uint8_t PumpDrive_ParseStream(PumpDrive_s *pump, const uint8_t *data, uint16_t len);

/**
 * @brief 将 ISC1000 的 5 字节 7-bit 数据流还原为 32 位无符号整数。
 * @param raw_5bytes 连续 5 字节编码流，每个字节只使用低 7 位。
 * @return 解码后的 32 位数值。
 */
uint32_t PumpDrive_Decode5ByteToUint32(const uint8_t raw_5bytes[5]);

/**
 * @brief 计算 ISC1000 反馈帧 BCC 异或校验。
 * @param data 参与校验的数据起始地址，不包含帧头 0xFF 和帧尾 0xFE。
 * @param len 参与校验的数据长度，单位 byte。
 * @return 1 字节 BCC 原始值；反馈帧中拆成高 1bit 和低 7bit 两个字节。
 */
uint8_t PumpDrive_CalcBcc(const uint8_t *data, uint16_t len);

#endif //PUMP_DRIVE_H
