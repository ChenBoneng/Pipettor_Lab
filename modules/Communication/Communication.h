#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

/**
 * @file Communication.h
 * @brief 分药仪上位机 CAN 通信协议层。
 *
 * 本模块负责上位机 CAN 协议帧收发、授权握手、命令缓存和通用 ACK 封装。
 * 模块本身不直接控制电机、泵、阀门或业务流程，业务层读取
 * CommunicationHostCommand_s 后再决定具体动作。
 *
 * 协议约定：
 * - 标准帧，11 bit CAN ID；
 * - DLC 固定 8 字节；
 * - 多字节整数统一小端，低字节在前；
 * - 0x100 / 0x101 命令收到后返回 0x180 ACK；
 * - 未授权动作类命令由本模块直接返回 UNAUTHORIZED ACK；
 * - 授权握手使用 0x090~0x093，不再使用旧版 0xA0~0xA4 数据格式。
 */

/** CAN 协议固定数据长度。 */
#define COMMUNICATION_CAN_FRAME_LEN              8U

/** 下位机周期状态帧建议发送周期，单位 ms。 */
#define COMMUNICATION_STATUS_PERIOD_MS           500U

/** 上位机授权心跳建议周期，单位 ms。 */
#define COMMUNICATION_HEARTBEAT_PERIOD_MS        10000U

/** 授权会话超时时间，超过该时间未收到合法心跳后重新锁定。 */
#define COMMUNICATION_SESSION_TIMEOUT_MS         30000U

/** STM32F103 96-bit 唯一 ID 起始地址。 */
#define COMMUNICATION_STM32_UID_BASE             0x1FFFF7E8UL

/** STM32F103 UID 长度，单位字节。 */
#define COMMUNICATION_STM32_UID_SIZE             12U

/** 上位机 -> 下位机：授权请求或授权心跳。 */
#define COMMUNICATION_CAN_ID_AUTH_REQUEST        0x090U

/** 下位机 -> 上位机：授权挑战，BoardID + nonce。 */
#define COMMUNICATION_CAN_ID_AUTH_CHALLENGE      0x091U

/** 上位机 -> 下位机：授权响应。 */
#define COMMUNICATION_CAN_ID_AUTH_RESPONSE       0x092U

/** 下位机 -> 上位机：授权结果。 */
#define COMMUNICATION_CAN_ID_AUTH_RESULT         0x093U

/** 上位机 -> 下位机：控制命令。 */
#define COMMUNICATION_CAN_ID_CONTROL             0x100U

/** 上位机 -> 下位机：参数设置。 */
#define COMMUNICATION_CAN_ID_PARAM               0x101U

/** 上位机 -> 下位机：查询命令。 */
#define COMMUNICATION_CAN_ID_QUERY               0x102U

/** 下位机 -> 上位机：命令应答。 */
#define COMMUNICATION_CAN_ID_ACK                 0x180U

/** 下位机 -> 上位机：周期状态。 */
#define COMMUNICATION_CAN_ID_STATUS              0x181U

/** 下位机 -> 上位机：报警信息。 */
#define COMMUNICATION_CAN_ID_ALARM               0x182U

/** 下位机 -> 上位机：查询数据返回。 */
#define COMMUNICATION_CAN_ID_DATA                0x183U

/** 0x090 授权请求子命令：请求 BoardID 和 nonce。 */
#define COMMUNICATION_AUTH_MSG_REQUEST           0x01U

/** 0x090 授权心跳子命令：授权成功后周期刷新会话。 */
#define COMMUNICATION_AUTH_MSG_HEARTBEAT         0x02U

/** 正式授权类型。licenseType 会参与 LicenseKey 和 AuthCode 计算。 */
#define COMMUNICATION_LICENSE_TYPE_NORMAL        0x01U

/** 试用授权类型，当前预留。 */
#define COMMUNICATION_LICENSE_TYPE_TRIAL         0x02U

/** 维护授权类型，当前预留。 */
#define COMMUNICATION_LICENSE_TYPE_SERVICE       0x03U

/** 授权标志位，当前未使用。 */
#define COMMUNICATION_AUTH_FLAGS_NONE            0x00U

/**
 * @brief CAN 协议功能码。
 *
 * 功能码固定放在命令帧 Byte0。0xA0~0xA4 是旧版保留枚举，
 * 当前正式授权握手由 CAN ID 0x090~0x093 处理。
 */
typedef enum
{
    COMMUNICATION_CMD_START_PROCESS = 0x01U,      /**< 启动流程，Byte2 为流程号。 */
    COMMUNICATION_CMD_STOP_PROCESS = 0x02U,       /**< 停止整个流程，Byte2 为停止方式。 */
    COMMUNICATION_CMD_RESET_ERROR = 0x03U,        /**< 清除报警或复位错误状态。 */
    COMMUNICATION_CMD_MOVE_STEPPER = 0x04U,       /**< 单轴步进电机运动，SEQ 在 Byte7。 */
    COMMUNICATION_CMD_VALVE_CONTROL = 0x05U,      /**< 阀门控制，OBJ 指定阀门。 */
    COMMUNICATION_CMD_PUMP_CONTROL = 0x06U,       /**< 泵控制，OBJ 指定定量泵或抽水泵。 */
    COMMUNICATION_CMD_READ_ACTIVITY = 0x07U,      /**< 读取活度计。 */
    COMMUNICATION_CMD_QUERY_STATUS = 0x08U,       /**< 查询设备状态。 */
    COMMUNICATION_CMD_SET_PARAM = 0x09U,          /**< 参数设置，OBJ 指定参数类型。 */
    COMMUNICATION_CMD_HEARTBEAT = 0x0AU,          /**< 普通心跳，当前主要使用授权心跳。 */
    COMMUNICATION_CMD_MOVE_STEPPER_BOTH = 0x0BU,  /**< A/B 两轴同步运动，预留。 */
    COMMUNICATION_CMD_QUERY_VERSION = 0x0CU,      /**< 查询硬件、固件和协议版本。 */
    COMMUNICATION_CMD_STOP_OBJECT = 0x0DU,        /**< 停止指定对象。 */
    COMMUNICATION_CMD_AUTH_REQUEST = 0xA0U,       /**< 旧版保留：授权请求。 */
    COMMUNICATION_CMD_AUTH_CHALLENGE = 0xA1U,     /**< 旧版保留：授权挑战。 */
    COMMUNICATION_CMD_AUTH_RESPONSE = 0xA2U,      /**< 旧版保留：授权响应。 */
    COMMUNICATION_CMD_AUTH_RESULT = 0xA3U,        /**< 旧版保留：授权结果。 */
    COMMUNICATION_CMD_AUTH_CHALLENGE_EXT = 0xA4U  /**< 旧版保留：挑战扩展帧。 */
} CommunicationCommand_e;

/**
 * @brief CAN 协议对象 ID。
 *
 * 对象 ID 固定放在命令帧 Byte1。流程类、心跳、版本查询等系统级命令使用
 * COMMUNICATION_OBJ_SYSTEM。
 */
typedef enum
{
    COMMUNICATION_OBJ_SYSTEM = 0x00U,              /**< 系统整体对象。 */
    COMMUNICATION_OBJ_MOTOR_A = 0x01U,             /**< A 轴步进电机。 */
    COMMUNICATION_OBJ_MOTOR_B = 0x02U,             /**< B 轴步进电机。 */
    COMMUNICATION_OBJ_PUMP_1 = 0x03U,              /**< 定量泵 1。 */
    COMMUNICATION_OBJ_VALVE_1 = 0x04U,             /**< 阀门 1。 */
    COMMUNICATION_OBJ_VALVE_2 = 0x05U,             /**< 阀门 2。 */
    COMMUNICATION_OBJ_WATER_PUMP = 0x06U,          /**< 抽水泵。 */
    COMMUNICATION_OBJ_ACTIVITY_METER = 0x07U,      /**< 活度计。 */
    COMMUNICATION_OBJ_PUMP_2 = 0x08U,              /**< 定量泵 2。 */
    COMMUNICATION_OBJ_PREPARE_PARAM = 0x10U,       /**< 配药目标参数。 */
    COMMUNICATION_OBJ_DISPENSE_PARAM = 0x11U,      /**< 发药参数。 */
    COMMUNICATION_OBJ_PREPARE_RESULT = 0x12U,      /**< 配药结果返回对象。 */
    COMMUNICATION_OBJ_PREPARE_VOLUME_PARAM = 0x13U /**< 配药体积参数。 */
} CommunicationObject_e;

/**
 * @brief 命令 ACK 结果码。
 *
 * 结果码放在 0x180 应答帧 Byte2。上位机需要同时匹配 CMD、OBJ 和 SEQ。
 */
typedef enum
{
    COMMUNICATION_RESULT_OK = 0x00U,              /**< 命令已接受或执行成功。 */
    COMMUNICATION_RESULT_BUSY = 0x01U,            /**< 目标对象忙。 */
    COMMUNICATION_RESULT_BAD_PARAM = 0x02U,       /**< 参数非法。 */
    COMMUNICATION_RESULT_FAILED = 0x03U,          /**< 执行失败。 */
    COMMUNICATION_RESULT_UNSUPPORTED = 0x04U,     /**< 不支持该命令或对象。 */
    COMMUNICATION_RESULT_ALARM = 0x05U,           /**< 当前处于报警状态。 */
    COMMUNICATION_RESULT_UNAUTHORIZED = 0x06U,    /**< 未认证或会话失效。 */
    COMMUNICATION_RESULT_TIMEOUT = 0x07U,         /**< 执行或通信超时。 */
    COMMUNICATION_RESULT_LIMIT_TRIGGERED = 0x08U  /**< 限位触发。 */
} CommunicationResult_e;

/**
 * @brief 协议错误码。
 *
 * 错误码在 ACK 和报警帧中均按 uint16 小端发送。
 */
typedef enum
{
    COMMUNICATION_ERROR_NONE = 0x0000U,              /**< 无错误。 */
    COMMUNICATION_ERROR_ESTOP = 0x0001U,             /**< 急停触发。 */
    COMMUNICATION_ERROR_MOTOR_A_TIMEOUT = 0x0002U,   /**< A 轴运动超时。 */
    COMMUNICATION_ERROR_MOTOR_B_TIMEOUT = 0x0003U,   /**< B 轴运动超时。 */
    COMMUNICATION_ERROR_ACTIVITY_TIMEOUT = 0x0004U,  /**< 活度计通信超时。 */
    COMMUNICATION_ERROR_CAN_TIMEOUT = 0x0005U,       /**< CAN 通信超时。 */
    COMMUNICATION_ERROR_BAD_PARAM = 0x0006U,         /**< 参数非法。 */
    COMMUNICATION_ERROR_PROCESS_FAILED = 0x0007U,    /**< 流程执行失败。 */
    COMMUNICATION_ERROR_UNAUTHORIZED = 0x0008U,      /**< 未认证。 */
    COMMUNICATION_ERROR_AUTH_FAILED = 0x0009U,       /**< 认证失败。 */
    COMMUNICATION_ERROR_SESSION_TIMEOUT = 0x000AU,   /**< 会话心跳超时。 */
    COMMUNICATION_ERROR_LIMIT_TRIGGERED = 0x000BU,   /**< 限位触发。 */
    COMMUNICATION_ERROR_STATE_NOT_ALLOWED = 0x000CU  /**< 当前状态不允许执行。 */
} CommunicationError_e;

/** 系统状态，放在 0x181 周期状态帧 Byte0。 */
typedef enum
{
    COMMUNICATION_SYS_INIT = 0x00U,        /**< 系统初始化中。 */
    COMMUNICATION_SYS_IDLE = 0x01U,        /**< 空闲。 */
    COMMUNICATION_SYS_RUNNING = 0x02U,     /**< 运行中。 */
    COMMUNICATION_SYS_PAUSED = 0x03U,      /**< 暂停。 */
    COMMUNICATION_SYS_ALARM = 0x04U,       /**< 报警。 */
    COMMUNICATION_SYS_ESTOP = 0x05U,       /**< 急停。 */
    COMMUNICATION_SYS_AUTH_LOCKED = 0x06U  /**< 未认证锁定。 */
} CommunicationSystemState_e;

/** 活度计通信状态，放在 0x181 周期状态帧 Byte5。 */
typedef enum
{
    COMMUNICATION_ACTIVITY_NOT_READ = 0x00U,      /**< 尚未读取。 */
    COMMUNICATION_ACTIVITY_OK = 0x01U,            /**< 读取成功。 */
    COMMUNICATION_ACTIVITY_TIMEOUT = 0x02U,       /**< 通信超时。 */
    COMMUNICATION_ACTIVITY_CRC_ERROR = 0x03U,     /**< CRC 错误。 */
    COMMUNICATION_ACTIVITY_BAD_RESPONSE = 0x04U   /**< 响应格式错误。 */
} CommunicationActivityState_e;

/** 板卡授权状态。 */
typedef enum
{
    COMMUNICATION_AUTH_LOCKED = 0x00U,   /**< 未认证或会话超时。 */
    COMMUNICATION_AUTH_UNLOCKED = 0x01U  /**< 认证通过且心跳未超时。 */
} CommunicationAuthState_e;

/**
 * @brief 授权结果码，放在 0x093 授权结果帧 Byte1。
 */
typedef enum
{
    COMMUNICATION_AUTH_RESULT_OK = 0x00U,             /**< 授权成功。 */
    COMMUNICATION_AUTH_RESULT_BAD_KEY = 0x01U,        /**< 授权码错误。 */
    COMMUNICATION_AUTH_RESULT_EXPIRED = 0x02U,        /**< 授权过期，当前暂不启用。 */
    COMMUNICATION_AUTH_RESULT_BOARD_MISMATCH = 0x03U, /**< BoardID 不匹配，当前预留。 */
    COMMUNICATION_AUTH_RESULT_BAD_NONCE = 0x04U,      /**< nonce 无效。 */
    COMMUNICATION_AUTH_RESULT_INTERNAL_ERROR = 0x05U  /**< 内部错误。 */
} CommunicationAuthResult_e;

/**
 * @brief 上位机命令帧缓存。
 *
 * 本结构体保存最近一帧已经通过授权检查的上位机命令。不同命令的数据区
 * 含义不同，业务层可根据 cmd/obj/id 再按协议解析 data[]。
 */
typedef struct
{
    uint16_t id;                               /**< CAN 标准帧 ID。 */
    uint8_t cmd;                               /**< Byte0 功能码。 */
    uint8_t obj;                               /**< Byte1 对象 ID。 */
    uint8_t seq;                               /**< 命令序号，用于匹配 ACK。 */
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN]; /**< 原始 8 字节数据。 */
    uint8_t updated;                           /**< 收到新命令后置 1。 */
    uint32_t frame_count;                      /**< 成功接收的上位机命令帧计数。 */
} CommunicationHostCommand_s;

/** 0x181 周期状态帧数据模型。 */
typedef struct
{
    uint8_t sys_state;      /**< Byte0：系统状态。 */
    uint8_t step;           /**< Byte1：当前流程步骤。 */
    uint8_t motor_state;    /**< Byte2：电机/泵忙状态位。 */
    uint8_t output_state;   /**< Byte3：阀门/水泵/蜂鸣器输出状态位。 */
    uint8_t sensor_state;   /**< Byte4：输入/限位状态位。 */
    uint8_t activity_state; /**< Byte5：活度计通信状态。 */
    uint16_t alarm;         /**< Byte6~Byte7：当前报警码。 */
} CommunicationStatus_s;

/** 0x182 报警帧数据模型。 */
typedef struct
{
    uint16_t alarm;  /**< Byte0~Byte1：报警码。 */
    uint8_t source;  /**< Byte2：报警来源。 */
    uint8_t level;   /**< Byte3：报警等级。 */
    uint8_t detail0; /**< Byte4：报警附加信息 0。 */
    uint8_t detail1; /**< Byte5：报警附加信息 1。 */
    uint8_t seq;     /**< Byte6：报警序号。 */
} CommunicationAlarm_s;

/**
 * @brief 授权模块运行状态。
 *
 * 业务层一般只需要读取 state 和 board_id。其它字段主要用于调试和状态上报。
 */
typedef struct
{
    uint32_t board_id;       /**< 当前板卡识别号，由 STM32 UID + 产品盐值计算得到。 */
    uint32_t nonce;          /**< 最近一次授权挑战随机数。 */
    uint32_t last_heartbeat; /**< 最近一次合法授权心跳时间，单位 HAL_GetTick() ms。 */
    uint8_t state;           /**< 当前授权状态，见 CommunicationAuthState_e。 */
    uint8_t fail_count;      /**< 连续授权失败次数。 */
    uint8_t license_type;    /**< 最近一次授权成功的 LicenseKey 类型。 */
    uint8_t flags;           /**< 最近一次授权成功的标志位。 */
} CommunicationAuthContext_s;

/**
 * @brief 初始化上位机 CAN 通信模块。
 *
 * 本函数完成三件事：
 * 1. 计算当前板卡 BoardID；
 * 2. 初始化授权上下文，默认进入未授权锁定状态；
 * 3. 注册上位机可能发送的 5 个 CAN ID：
 *    0x090、0x092、0x100、0x101、0x102。
 *
 * 注意：
 * 这里不主动发送任何 CAN 帧，也不执行任何业务动作。
 *
 * @return 1 表示初始化成功；0 表示 CAN 注册失败。
 */
uint8_t Communication_Init(void);

/**
 * @brief 周期维护通信模块。
 *
 * 当前主要用于检查授权心跳是否超时。
 * 授权成功后，如果超过 COMMUNICATION_SESSION_TIMEOUT_MS 没有收到
 * 0x090 Byte0=0x02 授权心跳，则重新回到 COMMUNICATION_AUTH_LOCKED。
 *
 * 推荐调用位置：
 * ModuleTask() 中 10~100ms 周期调用即可，当前工程放在 2ms 底层维护任务里。
 */
void Communication_Process(void);

/** 判断 communication 模块是否已经初始化完成。 */
uint8_t Communication_IsReady(void);

/** 判断是否收到过尚未清除的新命令。 */
uint8_t Communication_HasNewCommand(void);

/**
 * @brief 读取最近一次上位机 CAN 命令。
 *
 * 本函数只复制最近一帧“已经通过通信层检查”的命令。
 * 授权帧、授权心跳、未授权被拒绝的动作命令都不会写入该缓存。
 *
 * 注意：
 * 读取后不会自动清除 updated 标志，业务层处理完成后再调用
 * Communication_ClearNewCommandFlag()。
 *
 * @param command 输出参数，用于保存最新命令内容。
 * @return 1 表示读取成功；0 表示参数为空。
 */
uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command);

/**
 * @brief 清除新命令标志。
 *
 * 只清除 updated，不清空最近一次命令内容。
 * 这样 LCD 或调试代码仍可读取上一帧命令含义。
 */
void Communication_ClearNewCommandFlag(void);

/**
 * @brief 发送一帧标准 CAN 协议数据。
 *
 * 本函数只负责把 8 字节标准帧发送到 CAN 总线，不检查业务含义。
 * 上层建议优先使用 SendAck / SendStatus / SendAlarm / SendActivity 等封装。
 *
 * @param std_id 标准帧 ID，范围 0x000~0x7FF。
 * @param data 8 字节数据区。
 * @return 1 表示发送成功；0 表示模块未初始化、参数错误或发送邮箱忙。
 */
uint8_t Communication_SendFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);

/**
 * @brief 发送 0x180 命令应答帧。
 *
 * ACK 表示“命令是否被通信层/业务层接受”，不一定表示动作已经完成。
 * 例如步进电机运动命令收到 ACK OK 后，电机可能仍在运行中，
 * 上位机应结合 0x181 状态帧判断忙闲状态。
 */
uint8_t Communication_SendAck(uint8_t cmd,
                              uint8_t obj,
                              uint8_t result,
                              uint16_t error,
                              uint8_t exec_state,
                              uint8_t seq);

/**
 * @brief 发送 0x181 周期状态帧。
 *
 * 状态帧用于让上位机持续知道下位机在线、流程阶段、输出状态和报警码。
 * 即使未授权，也可以发送状态帧，上位机可据此显示 AUTH_LOCKED。
 */
uint8_t Communication_SendStatus(const CommunicationStatus_s *status);

/**
 * @brief 发送 0x182 报警帧。
 *
 * 报警帧是事件触发型，不需要等待周期状态帧。
 * 周期状态帧 Byte6~Byte7 仍应保留当前报警码，方便上位机刷新页面。
 */
uint8_t Communication_SendAlarm(const CommunicationAlarm_s *alarm);

/**
 * @brief 发送 0x183 活度计活度值子帧。
 *
 * 帧格式：
 * Byte0 = READ_ACTIVITY；
 * Byte1 = 0x01，表示活度值子帧；
 * Byte2~Byte5 = float32 活度值，小端 IEEE754；
 * Byte6 = 单位；
 * Byte7 = 上位机查询 SEQ。
 */
uint8_t Communication_SendActivity(float activity, uint8_t unit, uint8_t seq);

/**
 * @brief 发送 0x183 活度计附加信息子帧。
 *
 * 该帧用于补充核素、本底扣除、通道号和活度计通信状态。
 * 通常在 Communication_SendActivity() 之后发送。
 */
uint8_t Communication_SendActivityInfo(uint16_t isotope,
                                       uint8_t background_subtracted,
                                       uint8_t channel,
                                       uint8_t status,
                                       uint8_t seq);

/**
 * @brief 发送 0x183 配药结果帧。
 *
 * 用于配药完成后返回实际最终体积和实测总活度。
 * 上位机可据此计算实测浓度：
 * 实测浓度 = measured_activity / actual_total_volume。
 *
 * 如果暂时没有实际最终体积，可按协议先填 0，由上位机退回使用理论最终体积。
 */
uint8_t Communication_SendPrepareResult(uint16_t actual_total_volume_x100,
                                        uint16_t measured_activity_x100,
                                        uint8_t seq);

/**
 * @brief 发送 0x183 版本信息帧。
 *
 * 该帧响应上位机 QUERY_VERSION。
 * 版本号只做协议显示，不参与授权计算。
 */
uint8_t Communication_SendVersion(uint8_t hw_major,
                                  uint8_t hw_minor,
                                  uint8_t fw_major,
                                  uint8_t fw_minor,
                                  uint8_t proto_major,
                                  uint8_t proto_minor,
                                  uint8_t seq);

/**
 * @brief 发送 0x091 授权挑战帧。
 *
 * 新版授权协议中，0x091 不携带 CMD，也不回传 SEQ。
 * 8 字节数据固定为：
 * Byte0~Byte3 = BoardID，小端 uint32；
 * Byte4~Byte7 = nonce，小端 uint32。
 */
uint8_t Communication_SendAuthChallenge(uint32_t board_id, uint32_t nonce);

/**
 * @brief 发送 0x093 授权结果帧。
 *
 * 授权结果帧用于告诉上位机本次 0x092 是否通过。
 * 授权成功时 auth_state=UNLOCKED、result=OK；
 * 授权失败时 auth_state=LOCKED，result 指明 BAD_KEY 或 BAD_NONCE。
 */
uint8_t Communication_SendAuthResult(uint8_t auth_state,
                                     uint8_t result,
                                     uint8_t fail_count,
                                     uint8_t license_type,
                                     uint16_t session_timeout_sec,
                                     uint8_t seq);

/** 获取当前授权状态。 */
uint8_t Communication_GetAuthState(void);

/** 返回 1 表示当前已授权且心跳未超时。 */
uint8_t Communication_IsUnlocked(void);

/** 获取当前板卡 BoardID。上位机界面建议显示为 8 位大写十六进制。 */
uint32_t Communication_GetBoardId(void);

/** 获取最近一次 nonce，主要用于调试。 */
uint32_t Communication_GetNonce(void);

/** 获取授权上下文，只读使用，不要在业务层修改内容。 */
const CommunicationAuthContext_s *Communication_GetAuthContext(void);

/**
 * @brief 判断该命令当前是否允许执行。
 *
 * 本函数先根据 Communication_CommandRequiresAuth() 判断命令是否属于动作类命令。
 * 不需要授权的命令直接允许；需要授权的命令只有在当前 UNLOCKED 时允许。
 */
uint8_t Communication_IsCommandAllowed(uint8_t cmd, uint16_t std_id);

/**
 * @brief 判断 CAN ID 是否属于上位机发来的命令 ID。
 *
 * 只接受上位机方向 ID：
 * 0x090、0x092、0x100、0x101、0x102。
 * 下位机自己发送的 0x180~0x183 不应进入业务命令解析。
 */
uint8_t Communication_IsHostCommandId(uint16_t std_id);

/**
 * @brief 判断命令是否必须认证通过后才能执行。
 *
 * 文档约定不要求授权的命令包括：
 * 授权请求、授权响应、授权心跳、查询状态、查询版本、停止流程、复位错误。
 *
 * 其它命令默认需要授权，例如启动流程、设置参数、电机、泵、阀门、
 * 读取关键业务数据等。
 */
uint8_t Communication_CommandRequiresAuth(uint8_t cmd, uint16_t std_id);

/**
 * @brief 计算当前板卡 LicenseKey。
 *
 * 该函数主要用于和上位机/授权工具保持同一套算法。
 * 输入顺序必须和文档一致：
 * BoardID 小端 + licenseType + FactorySecret。
 */
uint32_t Communication_CalculateLicenseKey(uint32_t board_id, uint8_t license_type);

/**
 * @brief 计算授权响应 AuthCode。
 *
 * AuthCode 加入 nonce，用于避免简单录制重放旧的授权响应。
 * 输入顺序必须和文档一致：
 * BoardID 小端 + nonce 小端 + LicenseKey 小端 + licenseType + flags + ProtocolSecret。
 */
uint32_t Communication_CalculateAuthCode(uint32_t board_id,
                                         uint32_t nonce,
                                         uint32_t license_key,
                                         uint8_t license_type,
                                         uint8_t flags);

/**
 * @brief 根据 STM32 UID 计算 BoardID。
 *
 * BoardID 是公开设备识别号，不是密钥。
 * 当前算法读取 STM32 UID 12 字节原始内存顺序，再追加产品盐值计算 CRC32。
 */
uint32_t Communication_CalculateBoardIdFromUid(void);

/** 从协议数据中读取小端 uint16。 */
uint16_t Communication_ReadU16LE(const uint8_t *data);

/** 从协议数据中读取小端 uint32。 */
uint32_t Communication_ReadU32LE(const uint8_t *data);

/** 向协议数据中写入小端 uint16。 */
void Communication_WriteU16LE(uint8_t *data, uint16_t value);

/** 向协议数据中写入小端 uint32。 */
void Communication_WriteU32LE(uint8_t *data, uint32_t value);

#endif //COMMUNICATION_H
