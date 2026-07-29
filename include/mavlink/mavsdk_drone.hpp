#pragma once
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/rtk/rtk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <tinyxml2.h>

#include <memory>
#include <nlohmann/json.hpp>

#include "dk/adapters/mavsdk.hpp"
#include "imavlink.hpp"

// APM 旋翼机 (ArduCopter) 模式枚举映射
inline float get_copter_mode_param(mavsdk::Telemetry::FlightMode mode) {
    float param = -1.0f;
    switch (mode) {
        case mavsdk::Telemetry::FlightMode::Stabilized:
            param = 0.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Acro:
            param = 1.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Altctl:
            param = 2.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Mission:
            param = 3.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Takeoff:
            param = 4.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Offboard:
            param = 4.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Hold:
            param = 5.0f;
            break;
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            param = 6.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Land:
            param = 9.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Posctl:
            param = 16.0f;
            break;
        default:
            param = -1.0f;
            break;
    }
    return param;
}

// APM 车/船 (ArduRover) 模式枚举映射
inline float get_rover_mode_param(mavsdk::Telemetry::FlightMode mode) {
    float param = -1.0f;
    switch (mode) {
        case mavsdk::Telemetry::FlightMode::Manual:
            param = 0.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Acro:
            param = 1.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Hold:
            param = 4.0f;
            break;
        case mavsdk::Telemetry::FlightMode::FollowMe:
            param = 6.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Mission:
            param = 10.0f;
            break;
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            param = 11.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Offboard:
            param = 15.0f;
            break;
        case mavsdk::Telemetry::FlightMode::Takeoff:
            param = 15.0f;
            break;
        default:
            param = -1.0f;
            break;
    }
    return param;
}

class MavsdkDrone : public IMavlink {
   private:
    std::shared_ptr<mavsdk::System> system_;
    std::shared_ptr<mavsdk::Action> action_;
    std::shared_ptr<mavsdk::Param> param_;
    std::shared_ptr<mavsdk::Offboard> offboard_;
    std::shared_ptr<mavsdk::Telemetry> telemetry_;
    std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
    std::shared_ptr<mavsdk::Rtk> rtk_;
    nlohmann::json pdef_;
    VehicleType vehicle_type_ = VehicleType::Unknown;
    std::function<float(mavsdk::Telemetry::FlightMode)> get_mode_param_ =
        get_copter_mode_param;

    void load_pdef(const std::string& path);

   public:
    MavsdkDrone(std::shared_ptr<mavsdk::System> system);
    void set_target_type(VehicleType type) override {
        vehicle_type_ = type;
        // 根据当前机型选择对应的模式字典指针

        if (this->vehicle_type_ == VehicleType::Rover) {
            get_mode_param_ = get_rover_mode_param;
        } else {
            get_mode_param_ = get_copter_mode_param;
        }
    }

    bool arm() override;
    bool disarm() override;
    bool takeoff(double alt) override;
    bool set_mode(const mavsdk::Telemetry::FlightMode& mode) override;
    bool run_prearm_checks() override;
    bool set_stream_rate(int stream_id, int rate) override;
    bool set_msg_interval(int stream_id, int rate) override;
    ApmParam get_param(const std::string& name, const ApmParam& value) override;
    bool set_param(const std::string& name, const ApmParam& value) override;
    bool pull_params() override;
    nlohmann::json get_all_params() override;
    bool reboot_fcu() override;
    bool cmd_vel(Eigen::Vector4d vel) override;
    void send_rtcm_data(const uint8_t* data, size_t size) override;
    bool is_prearm_msg(const std::string& text) override;
    bool check_sensor_health(uint32_t sensor_health) override;
};