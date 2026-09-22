#pragma once
#include <plugins/telemetry/telemetry.h>

#include <Eigen/Dense>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>

#include "dk/adapters/mavsdk.hpp"
#include "utils/fixed_string64.hpp"

using ApmParam = std::variant<int, double, std::string>;

class IMavlink {
   public:
    VehicleType vehicle_type = VehicleType::Unknown;
    virtual bool set_mode(const mavsdk::Telemetry::FlightMode& mode) = 0;
    virtual bool cmd_vel(Eigen::Vector4d vel) = 0;  // 只要求实现body的速度控制

    // MAV_CMD_RUN_PREARM_CHECKS
    virtual bool run_prearm_checks() = 0;
    virtual bool reboot_fcu() = 0;
    virtual bool set_stream_rate(int stream_id, int rate) = 0;
    virtual bool set_msg_interval(int stream_id, int rate) = 0;
    virtual bool arm() = 0;
    virtual bool disarm() = 0;
    virtual bool takeoff(double alt) = 0;
    virtual ApmParam get_param(const std::string& name,
                               const ApmParam& value) = 0;
    virtual bool set_param(const std::string& name, const ApmParam& value) = 0;
    virtual nlohmann::json get_all_params() = 0;  // 加载全部参数
    virtual bool pull_params() = 0;
    virtual void send_rtcm_data(const uint8_t* data, size_t size) = 0;
    virtual void set_target_type(VehicleType type) = 0;
    virtual bool is_prearm_msg(const std::string& text) = 0;
    virtual bool check_sensor_health(uint32_t sensor_health) = 0;

    // COMMAND_LONG (#76) 的执行结果, mav_result 对应 MAV_RESULT 枚举
    struct CmdLongResult {
        bool success = false;
        uint8_t mav_result = 4;  // MAV_RESULT_FAILED
    };

    // SET_POSITION_TARGET_LOCAL_NED (#84), pos/vel 为 NED, yaw 为 NED 航向
    // (rad), yaw_rate 为 NED 角速度 (rad/s)。 mavros 兼容桥使用
    virtual bool send_position_target(uint8_t coordinate_frame,
                                      uint16_t type_mask,
                                      const Eigen::Vector3d& pos_ned,
                                      const Eigen::Vector3d& vel_ned, float yaw,
                                      float yaw_rate) {
        return false;
    }

    // 透传 COMMAND_LONG 给 FCU 并等待 ACK, mavros /mavros/cmd/command 服务使用
    virtual CmdLongResult send_command_long(uint16_t command,
                                            uint8_t confirmation, float p1,
                                            float p2, float p3, float p4,
                                            float p5, float p6, float p7) {
        return {};
    }
    // 获取实际的数据
    template <typename Type>
    static Type unpack(const ApmParam& param) {
        return std::get<Type>(param);
    }
};
