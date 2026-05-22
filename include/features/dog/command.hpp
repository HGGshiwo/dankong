#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

// -------------------------- 核心枚举 --------------------------
enum class CommandType : uint32_t {
    MANUAL_HEARTBEAT = 0x21040001,
    ENTER_L_MODE = 0x21010220,
    EXIT_L_MODE = 0x21010221,
    TOGGLE_STAND_DOWN = 0x21010202,
    SET_AIM_POSE = 0x010B06,
    RESET_AIM_POSE = 59,
    MOTION_MODE_MANUAL = 0x21010C02,
    MOTION_MODE_NAVIGATION = 0x21010C04,
    MOVE_X_AXIS = 0x21010130,
    MOVE_Y_AXIS = 0x21010131,
    MOVE_YAW_AXIS = 0x21010135,
    AXIS_COMMAND_NO_DEAD_ZONE = 0x21010140,
    SET_PLATFORM_HEIGHT = 0x21010406,
    GAIT_WALK = 0x21010420,
    GAIT_RUN = 0x21010423,
    SWITCH_SPEED_GEAR = 0x31010F11,
    EMERGENCY_STOP = 0x21010C0E,
    CAMERA_CONTROL = 0x21010F12,
    UPPER_POWER_SWITCH = 0x80110801,
    DRIVER_JOINT_DATA_SWITCH = 0x80110201,
    STATE_DATA_SWITCH = 0x80110202,

    RUN_STATUS_REPORT = 0x1008,
    MOTION_STATE_REPORT = 0x1009,
    SENSOR_DATA_REPORT = 0x100A,
    CONTROLLER_SAFE_DATA_REPORT = 0x100B,
    BATTERY_LEVEL_REPORT = 0x11050F01,
    BATTERY_CHARGE_STATE_REPORT = 0x11050F21,
    ERROR_CODE_REPORT = 0x12050F01
};

// 编译期获取命令类型 (0=基本, 1=扩展)，省去 std::map 查找开销
constexpr uint32_t get_command_extension_type(CommandType type) {
    switch (type) {
        case CommandType::SET_AIM_POSE:
        case CommandType::AXIS_COMMAND_NO_DEAD_ZONE:
        case CommandType::RUN_STATUS_REPORT:
        case CommandType::MOTION_STATE_REPORT:
        case CommandType::SENSOR_DATA_REPORT:
        case CommandType::CONTROLLER_SAFE_DATA_REPORT:
            return 1;
        default:
            return 0;
    }
}

// -------------------------- 内存紧凑型数据结构 (Zero-Copy 基础)
// --------------------------
#pragma pack(push, 1)  // 强制 1 字节对齐，严格匹配网络协议包

struct CommandHead {
    uint32_t command_id;
    uint32_t parameter_size;
    uint32_t command_type;
};
static_assert(sizeof(CommandHead) == 12, "Size mismatch");

struct FireAim {
    uint32_t roll;
    uint32_t pitch;
    uint32_t yaw;
};
static_assert(sizeof(FireAim) == 12, "Size mismatch");

struct AxisCommand {
    int32_t left_x;
    int32_t left_y;
    int32_t right_x;
    int32_t right_y;
};
static_assert(sizeof(AxisCommand) == 16, "Size mismatch");

struct ImuSensorData {
    int32_t timestamp;
    float roll, pitch, yaw;
    float omega_x, omega_y, omega_z;
    float acc_x, acc_y, acc_z;
};
static_assert(sizeof(ImuSensorData) == 40, "Size mismatch");

struct LegJointData {
    float fl_hipx, fl_hipy, fl_knee;
    float fr_hipx, fr_hipy, fr_knee;
    float hl_hipx, hl_hipy, hl_knee;
    float hr_hipx, hr_hipy, hr_knee;
};
static_assert(sizeof(LegJointData) == 48, "Size mismatch");

struct CpuInfo {
    float temperature;
    float frequency;
};
static_assert(sizeof(CpuInfo) == 8, "Size mismatch");

struct RcsData {
    char robot_name[15];  // 替代 std::string 避免堆分配
    int32_t current_milege;
    int32_t total_milege;
    int64_t current_run_time;
    int64_t total_run_time;
    int64_t current_motion_time;
    int64_t total_motion_time;
    float joystick_lx, joystick_ly, joystick_rx, joystick_ry;
    uint8_t is_nav_mode;
    uint8_t reserved_padding[9];
    uint32_t error_bits;

    // C++ 安全位操作替代 Python 的掩码
    bool imu_error() const { return (error_bits >> 31) & 1; }
    bool wifi_error() const { return (error_bits >> 0) & 1; }
    bool driver_heat_warn() const { return (error_bits >> 1) & 1; }
    bool driver_error() const { return (error_bits >> 2) & 1; }
    bool motor_heat_warn() const { return (error_bits >> 3) & 1; }
    bool battery_low_warn() const { return (error_bits >> 4) & 1; }

    void set_imu_error(bool v) {
        v ? error_bits |= (1U << 31) : error_bits &= ~(1U << 31);
    }
};
static_assert(sizeof(RcsData) == 85, "Size mismatch");

struct MotionStateData {
    uint8_t basic_state;
    uint8_t gait_state;
    float max_forward_vel;
    float max_backward_vel;
    float pos_x, pos_y, pos_yaw;
    float vel_x, vel_y, vel_yaw;
    float robot_distance;
    uint32_t touch_state;
    uint32_t control_state;  // 预留
    uint8_t auto_charge_state;
    uint8_t pos_ctrl_state;
    uint8_t task_reserved[8];
};
static_assert(sizeof(MotionStateData) == 56, "Size mismatch");

struct ControllerSensorData {
    ImuSensorData imu_data;
    LegJointData joint_pos;
    LegJointData joint_vel;
    LegJointData joint_tau;
};
static_assert(sizeof(ControllerSensorData) == 184, "Size mismatch");

struct ControllerSafeData {
    float motor_temperatures[12];     // 替代 std::vector
    uint8_t driver_temperatures[12];  // 替代 std::vector
    CpuInfo cpu_info;
};
static_assert(sizeof(ControllerSafeData) == 68, "Size mismatch");

#pragma pack(pop)

// -------------------------- 封包 (Serialization) --------------------------

// 基础指令打包 (无 Payload)
inline std::vector<uint8_t> pack_cmd(CommandType cmd_type,
                                     uint32_t parameter_size = 0) {
    std::vector<uint8_t> buffer(sizeof(CommandHead));
    auto* head = reinterpret_cast<CommandHead*>(buffer.data());
    head->command_id = static_cast<uint32_t>(cmd_type);
    head->parameter_size = parameter_size;
    head->command_type = 0;
    return buffer;
}

// 泛型结构体打包 (有 Payload，单次内存分配 + 内存拷贝，速度极快)
template <typename PayloadType>
std::vector<uint8_t> pack_cmd(CommandType cmd_type,
                              const PayloadType& payload) {
    std::vector<uint8_t> buffer(sizeof(CommandHead) + sizeof(PayloadType));

    auto* head = reinterpret_cast<CommandHead*>(buffer.data());
    head->command_id = static_cast<uint32_t>(cmd_type);
    head->parameter_size = sizeof(PayloadType);
    head->command_type = get_command_extension_type(cmd_type);

    std::memcpy(buffer.data() + sizeof(CommandHead), &payload,
                sizeof(PayloadType));
    return buffer;
}

// -------------------------- 零拷贝解包 (Deserialization)
// --------------------------

class UdpPacketView {
   private:
    const uint8_t* data_ptr_;  // 指向数据的常量指针
    size_t size_;              // 数据长度
   public:
    // 支持直接传入 std::vector
    explicit UdpPacketView(const std::vector<uint8_t>& packet)
        : data_ptr_(packet.data()), size_(packet.size()) {
        if (size_ < sizeof(CommandHead)) {
            throw std::invalid_argument("Packet too small for header");
        }
    }
    // 重载：支持传入原生数组或 socket 接收的 raw buffer，扩展性更好
    UdpPacketView(const uint8_t* data, size_t size)
        : data_ptr_(data), size_(size) {
        if (size_ < sizeof(CommandHead)) {
            throw std::invalid_argument("Packet too small for header");
        }
    }
    // 获取 Header 指针
    const CommandHead* header() const {
        return reinterpret_cast<const CommandHead*>(data_ptr_);
    }
    // 获取指令类型枚举
    CommandType command() const {
        return static_cast<CommandType>(header()->command_id);
    }
    // 泛型提取 Payload 指针 (Zero-Copy 提取)
    template <typename PayloadType>
    const PayloadType* get_payload() const {
        if (header()->parameter_size != sizeof(PayloadType) ||
            size_ < sizeof(CommandHead) + sizeof(PayloadType)) {
            return nullptr;  // 数据不匹配或被截断
        }
        return reinterpret_cast<const PayloadType*>(data_ptr_ +
                                                    sizeof(CommandHead));
    }
    // 针对电池电量的特殊提取 (数据存在 parameter_size 里)
    uint32_t get_battery_level() const { return header()->parameter_size; }

    // 针对电池充电状态的特殊提取
    struct ChargeState {
        uint32_t level;
        bool is_charging;
    };
    ChargeState get_charge_state() const {
        return {(header()->parameter_size >> 8) & 0xFFFFFF,
                (header()->parameter_size & 0xFF) == 1};
    }
};

// -------------------------- 摇杆数据转换 --------------------------
inline int32_t speed_to_cmd_value(char axis, float speed, float v_max = 1.0f) {
    float cmd_value = 0.0f;
    switch (axis) {
        case 'x':
            cmd_value = speed != 0 ? (speed / v_max) * 26215.0f +
                                         (speed > 0 ? -6553.0f : 6553.0f)
                                   : 0;
            break;
        case 'y':
            cmd_value = speed != 0 ? -((speed / v_max) * 8554.0f +
                                       (speed > 0 ? 24576.0f : -24576.0f))
                                   : 0;
            break;
        case 'w':
            cmd_value = -speed * 32768.0f;
            break;
        default:
            throw std::invalid_argument("Axis must be 'x', 'y', or 'w'");
    }
    return std::clamp(static_cast<int32_t>(std::round(cmd_value)), -32767,
                      32767);
}