#pragma once

#include <cstdint>
#include <cstring>

// ==============================================================================
// 核心原则：使用内存紧凑排列，禁用编译器优化对齐，确保 C++ 和
// Android(Java/Kotlin) 两端的字节解析完全一致。
// ==============================================================================
#pragma pack(push, 1)

// --------------------------------------------------------------------------
// 1. 协议头与消息类型定义
// --------------------------------------------------------------------------
constexpr uint16_t MAGIC_HEADER = 0xAA55;

enum class MsgId : uint16_t {
    // [上行: DJI -> C++]
    TELEMETRY_HIGH_HZ = 0x01,  // 高频遥测 (姿态、速度、位置)
    TELEMETRY_LOW_HZ = 0x02,   // 低频遥测 (电量、GPS星数)
    STATUS_TEXT = 0x03,        // 报错文本
    PARAM_RESPONSE = 0x04,     // 参数读取响应

    // [下行: C++ -> DJI]
    CMD_VELOCITY = 0x10,   // 虚拟摇杆/速度控制 (高频)
    CMD_BASIC = 0x11,      // 基础指令 (起飞/降落/解锁)
    CMD_SET_MODE = 0x12,   // 切模式
    CMD_PARAM_REQ = 0x13,  // 读写参数请求
    CMD_RTCM = 0x14        // RTK 数据
};

// 所有 UDP 数据包的通用头部
struct PacketHeader {
    uint16_t magic;        // 固定为 MAGIC_HEADER (校验错包)
    uint16_t msg_id;       // 对应 MsgId
    uint32_t seq;          // 序列号 (用于统计丢包率)
    uint16_t payload_len;  // 负载长度
};

// --------------------------------------------------------------------------
// 2. [上行] 无人机遥测数据 (替代 MavlinkFeature 中的各种订阅)
// 建议发送频率: 20Hz - 50Hz
// --------------------------------------------------------------------------
struct TelemetryHighHz {
    uint8_t is_connected;
    uint8_t is_armed;
    uint8_t flight_mode;  // 建议映射一个 Enum, 比如 1:GUIDED, 2:AUTO, 3:RTL

    // ENU 本地位置 (米)
    float pos_enu_x;
    float pos_enu_y;
    float pos_enu_z;

    // ENU 速度 (米/秒)
    float vel_enu_x;
    float vel_enu_y;
    float vel_enu_z;

    // ENU_FLU 姿态四元数
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    // 机体系角速度 (弧度/秒)
    float ang_vel_x;
    float ang_vel_y;
    float ang_vel_z;

    // 全球坐标 (经纬度必须用 double 保证精度)
    double latitude;
    double longitude;
    float relative_alt;
};

// 低频数据，不需要每秒发50次，节省带宽 (建议 1Hz)
struct TelemetryLowHz {
    float battery_percent;
    uint8_t gps_fix_type;
    uint8_t gps_nsats;
    uint32_t sensor_health_bitmask;
};

// 事件驱动文本
struct StatusTextPayload {
    uint8_t severity;  // 0:Info, 1:Warning, 2:Error, 3:Critical
    char text[64];     // C风格字符串，自动截断
};

// --------------------------------------------------------------------------
// 3. [下行] 实时控制数据 (替代 IMavlink::cmd_vel)
// 建议发送频率: 20Hz - 50Hz
// --------------------------------------------------------------------------
struct CmdVelocity {
    float vx;        // 前后 (米/秒)
    float vy;        // 左右 (米/秒)
    float vz;        // 上下 (米/秒)
    float yaw_rate;  // 偏航角速度 (弧度/秒)
    uint8_t frame;  // 0: 机体坐标系(FLU), 1: 东北天大地坐标系(ENU)
};

// --------------------------------------------------------------------------
// 4. [下行] 基础指令 (替代 IMavlink 杂项函数)
// --------------------------------------------------------------------------
enum class BasicCmdType : uint8_t {
    ARM = 1,
    DISARM,
    TAKEOFF,
    LAND,
    REBOOT_FCU,
    PREARM_CHECK
};

struct CmdBasic {
    uint8_t cmd_type;  // 对应 BasicCmdType
    float param1;      // 通用参数，比如 takeoff 时的 altitude
};

struct CmdSetMode {
    char mode_name[16];  // 替代 FixedString64，16字节足够存 "GUIDED" 等
};

// --------------------------------------------------------------------------
// 5. [双向] 参数系统 (替代 get_param / set_param)
// --------------------------------------------------------------------------
struct ParamData {
    uint8_t is_set;  // 0: Get 请求, 1: Set 请求 / 响应
    char param_name[16];
    uint8_t value_type;  // 0: int, 1: double, 2: string

    // 使用 union 共用体完美替代 std::variant，节省极大的内存和序列化开销
    union {
        int i_val;
        double d_val;
        char s_val[64];
    } value;
};

// --------------------------------------------------------------------------
// 6. [下行] RTCM 数据 (变长数据)
// --------------------------------------------------------------------------
// RTCM 不固定长度，通常只需要一个头部，然后紧跟 raw data
struct CmdRtcmHeader {
    // payload_len 记录在 PacketHeader 里，直接读取其后的字节即可
};

#pragma pack(pop)