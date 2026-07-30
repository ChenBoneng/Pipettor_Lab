#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>

/**
 * @file Communication.h
 * @brief 分药仪上位机 CAN 通信协议层。
 *
 * 本模块只负责 CAN 协议帧的收发、字段解析和通用应答封装，不直接控制电机、
 * 泵、阀门或业务流程。业务层读取 CommunicationHostCommand_s 后，再根据
 * cmd/obj/data 决定是否执行具体动作。
 *
 * 协议约定：
 * - 标准帧，11 bit CAN ID；
 * - DLC 固定 8 字节；
 * - 多字节整数统一小端，低字节在前；
 * - 0x100 / 0x101 控制类命令必须返回 0x180 ACK；
 * - 下位机通过 0x181 周期上报状态，通过 0x182 事件上报警报。
 */

/** CAN 协议固定数据长度。 */
#define COMMUNICATION_CAN_FRAME_LEN              8U

/** 下位机周期状态帧建议发送周期，单位 ms。 */
#define COMMUNICATION_STATUS_PERIOD_MS           500U

/** 上位机心跳建议周期，单位 ms。 */
#define COMMUNICATION_HEARTBEAT_PERIOD_MS        500U

/** 认证会话超时时间，超过该时间未收到心跳后重新锁定。 */
#define COMMUNICATION_SESSION_TIMEOUT_MS         2000U

/** 上位机 -> 下位机：认证请求。 */
#define COMMUNICATION_CAN_ID_AUTH_REQUEST        0x090U

/** 下位机 -> 上位机：认证挑战。 */
#define COMMUNICATION_CAN_ID_AUTH_CHALLENGE      0x091U

/** 上位机 -> 下位机：认证响应。 */
#define COMMUNICATION_CAN_ID_AUTH_RESPONSE       0x092U

/** 下位机 -> 上位机：认证结果。 */
#define COMMUNICATION_CAN_ID_AUTH_RESULT         0x093U

/** 上位机 -> 下位机：控制命令。 */
#define COMMUNICATION_CAN_ID_CONTROL             0x100U

/** 上位机 -> 下位机：参数设置。 */
#define COMMUNICATION_CAN_ID_PARAM               0x101U

/** 上位机 -> 下位机：查询和心跳命令。 */
#define COMMUNICATION_CAN_ID_QUERY               0x102U

/** 下位机 -> 上位机：命令应答。 */
#define COMMUNICATION_CAN_ID_ACK                 0x180U

/** 下位机 -> 上位机：周期状态。 */
#define COMMUNICATION_CAN_ID_STATUS              0x181U

/** 下位机 -> 上位机：报警信息。 */
#define COMMUNICATION_CAN_ID_ALARM               0x182U

/** 下位机 -> 上位机：查询数据返回。 */
#define COMMUNICATION_CAN_ID_DATA                0x183U

/**
 * @brief CAN 协议功能码。
 *
 * 功能码固定放在命令帧 Byte0。认证相关命令虽然使用独立 CAN ID，仍保留
 * CMD 字段，方便上位机和下位机统一解析。
 */
typedef enum
{
    COMMUNICATION_CMD_START_PROCESS = 0x01U,      /**< 启动流程，Byte2 为流程号。 */
    COMMUNICATION_CMD_STOP_PROCESS = 0x02U,       /**< 停止整个流程，Byte2 为停止方式。 */
    COMMUNICATION_CMD_RESET_ERROR = 0x03U,        /**< 清除报警或复位错误状态。 */
    COMMUNICATION_CMD_MOVE_STEPPER = 0x04U,       /**< 单轴步进电机运动，OBJ 指定 A/B 轴。 */
    COMMUNICATION_CMD_VALVE_CONTROL = 0x05U,      /**< 阀门控制，OBJ 指定阀门。 */
    COMMUNICATION_CMD_PUMP_CONTROL = 0x06U,       /**< 定量泵控制，OBJ 指定泵 1/泵 2。 */
    COMMUNICATION_CMD_READ_ACTIVITY = 0x07U,      /**< 读取活度计。 */
    COMMUNICATION_CMD_QUERY_STATUS = 0x08U,       /**< 查询设备状态。 */
    COMMUNICATION_CMD_SET_PARAM = 0x09U,          /**< 参数设置，OBJ 指定参数类型。 */
    COMMUNICATION_CMD_HEARTBEAT = 0x0AU,          /**< 上位机心跳。 */
    COMMUNICATION_CMD_MOVE_STEPPER_BOTH = 0x0BU,  /**< A/B 两轴同步运动。 */
    COMMUNICATION_CMD_QUERY_VERSION = 0x0CU,      /**< 查询硬件、固件和协议版本。 */
    COMMUNICATION_CMD_STOP_OBJECT = 0x0DU,        /**< 停止指定对象。 */
    COMMUNICATION_CMD_AUTH_REQUEST = 0xA0U,       /**< 认证请求。 */
    COMMUNICATION_CMD_AUTH_CHALLENGE = 0xA1U,     /**< 认证挑战。 */
    COMMUNICATION_CMD_AUTH_RESPONSE = 0xA2U,      /**< 认证响应。 */
    COMMUNICATION_CMD_AUTH_RESULT = 0xA3U,        /**< 认证结果。 */
    COMMUNICATION_CMD_AUTH_CHALLENGE_EXT = 0xA4U  /**< 认证挑战扩展帧。 */
} CommunicationCommand_e;

/**
 * @brief CAN 协议对象 ID。
 *
 * 对象 ID 固定放在命令帧 Byte1。流程类、心跳、版本查询等系统级命令使用
 * COMMUNICATION_OBJ_SYSTEM。
 */
typedef enum
{
    COMMUNICATION_OBJ_SYSTEM = 0x00U,          /**< 系统整体对象。 */
    COMMUNICATION_OBJ_MOTOR_A = 0x01U,         /**< A 轴步进电机。 */
    COMMUNICATION_OBJ_MOTOR_B = 0x02U,         /**< B 轴步进电机。 */
    COMMUNICATION_OBJ_PUMP_1 = 0x03U,          /**< 定量泵 1。 */
    COMMUNICATION_OBJ_VALVE_1 = 0x04U,         /**< 阀门 1。 */
    COMMUNICATION_OBJ_VALVE_2 = 0x05U,         /**< 阀门 2。 */
    COMMUNICATION_OBJ_WATER_PUMP = 0x06U,      /**< 抽水泵。 */
    COMMUNICATION_OBJ_ACTIVITY_METER = 0x07U,  /**< 活度计。 */
    COMMUNICATION_OBJ_PUMP_2 = 0x08U,          /**< 定量泵 2。 */
    COMMUNICATION_OBJ_PREPARE_PARAM = 0x10U,   /**< 配药参数对象。 */
    COMMUNICATION_OBJ_DISPENSE_PARAM = 0x11U   /**< 发药参数对象。 */
} CommunicationObject_e;

/**
 * @brief 命令 ACK 结果码。
 *
 * 结果码放在 0x180 应答帧 Byte2。上位机需要同时匹配 CMD、OBJ 和 SEQ，
 * 不应只根据收到第一帧 ACK 判断命令完成。
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

/** 板卡认证状态。 */
typedef enum
{
    COMMUNICATION_AUTH_LOCKED = 0x00U,   /**< 未认证或会话超时。 */
    COMMUNICATION_AUTH_UNLOCKED = 0x01U  /**< 认证通过且心跳未超时。 */
} CommunicationAuthState_e;

/**
 * @brief 上位机命令帧缓存。
 *
 * 本结构体保存最近一帧上位机命令的原始数据和关键字段。不同命令的数据区
 * 含义不同，业务层可根据 cmd/obj/id 再按协议解析 data[]。
 */
typedef struct
{
    uint16_t id;                                      /**< CAN 标准帧 ID。 */
    uint8_t cmd;                                      /**< Byte0 功能码。 */
    uint8_t obj;                                      /**< Byte1 对象 ID。 */
    uint8_t seq;                                      /**< 命令序号，用于匹配 ACK。 */
    uint8_t data[COMMUNICATION_CAN_FRAME_LEN];        /**< 原始 8 字节数据。 */
    uint8_t updated;                                  /**< 收到新命令后置 1。 */
    uint32_t frame_count;                             /**< 成功接收的上位机命令帧计数。 */
} CommunicationHostCommand_s;

/**
 * @brief 0x181 周期状态帧数据模型。
 */
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

/**
 * @brief 0x182 报警帧数据模型。
 */
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
 * @brief 初始化上位机 CAN 通信模块。
 *
 * @return 1 表示初始化成功；0 表示 CAN 注册失败。
 *
 * @note 本函数会注册 V1.0 协议中所有上位机发给下位机的 CAN ID：
 *       0x090、0x092、0x100、0x101、0x102。
 */
uint8_t Communication_Init(void);

/**
 * @brief 判断 communication 模块是否已经初始化完成。
 *
 * @return 1 表示已初始化；0 表示未初始化或初始化失败。
 */
uint8_t Communication_IsReady(void);

/**
 * @brief 判断是否收到过尚未清除的新命令。
 *
 * @return 1 表示有新命令；0 表示没有新命令。
 */
uint8_t Communication_HasNewCommand(void);

/**
 * @brief 读取最近一次上位机 CAN 命令。
 *
 * @param command 输出参数，用于保存最新命令内容。
 * @return 1 表示读取成功；0 表示参数为空。
 *
 * @note 读取不会自动清除 updated 标志，业务层处理完成后再调用
 *       Communication_ClearNewCommandFlag()。
 */
uint8_t Communication_GetHostCommand(CommunicationHostCommand_s *command);

/**
 * @brief 清除新命令标志。
 *
 * @note 只清除 updated，不会清空最近一次命令内容。
 */
void Communication_ClearNewCommandFlag(void);

/**
 * @brief 发送一帧标准 CAN 协议数据。
 *
 * @param std_id 标准帧 ID，范围 0x000~0x7FF。
 * @param data 8 字节数据区。
 * @return 1 表示发送成功；0 表示模块未初始化、参数错误或发送邮箱忙。
 */
uint8_t Communication_SendFrame(uint16_t std_id, const uint8_t data[COMMUNICATION_CAN_FRAME_LEN]);

/**
 * @brief 发送 0x180 命令应答帧。
 */
uint8_t Communication_SendAck(uint8_t cmd,
                              uint8_t obj,
                              uint8_t result,
                              uint16_t error,
                              uint8_t exec_state,
                              uint8_t seq);

/**
 * @brief 发送 0x181 周期状态帧。
 */
uint8_t Communication_SendStatus(const CommunicationStatus_s *status);

/**
 * @brief 发送 0x182 报警帧。
 */
uint8_t Communication_SendAlarm(const CommunicationAlarm_s *alarm);

/**
 * @brief 发送 0x183 活度计活度值子帧。
 */
uint8_t Communication_SendActivity(float activity, uint8_t unit, uint8_t seq);

/**
 * @brief 发送 0x183 活度计附加信息子帧。
 */
uint8_t Communication_SendActivityInfo(uint16_t isotope,
                                       uint8_t background_subtracted,
                                       uint8_t channel,
                                       uint8_t status,
                                       uint8_t seq);

/**
 * @brief 发送 0x183 版本信息帧。
 */
uint8_t Communication_SendVersion(uint8_t hw_major,
                                  uint8_t hw_minor,
                                  uint8_t fw_major,
                                  uint8_t fw_minor,
                                  uint8_t proto_major,
                                  uint8_t proto_minor,
                                  uint8_t seq);

/**
 * @brief 发送认证挑战帧，包含 board_id 和 nonce 低 16 位。
 */
uint8_t Communication_SendAuthChallenge(uint32_t board_id, uint16_t nonce_low, uint8_t seq);

/**
 * @brief 发送认证挑战扩展帧，包含 nonce 高 16 位。
 */
uint8_t Communication_SendAuthChallengeExt(uint16_t nonce_high, uint8_t seq);

/**
 * @brief 发送认证结果帧。
 */
uint8_t Communication_SendAuthResult(uint8_t success,
                                     uint8_t lock_state,
                                     uint8_t fail_count,
                                     uint16_t session_timeout_sec,
                                     uint8_t seq);

/**
 * @brief 判断 CAN ID 是否属于上位机发来的命令 ID。
 */
uint8_t Communication_IsHostCommandId(uint16_t std_id);

/**
 * @brief 判断命令是否必须认证通过后才能执行。
 */
uint8_t Communication_CommandRequiresAuth(uint8_t cmd, uint16_t std_id);

/** 从协议数据中读取小端 uint16。 */
uint16_t Communication_ReadU16LE(const uint8_t *data);

/** 从协议数据中读取小端 uint32。 */
uint32_t Communication_ReadU32LE(const uint8_t *data);

/** 向协议数据中写入小端 uint16。 */
void Communication_WriteU16LE(uint8_t *data, uint16_t value);

/** 向协议数据中写入小端 uint32。 */
void Communication_WriteU32LE(uint8_t *data, uint32_t value);

#endif //COMMUNICATION_H
