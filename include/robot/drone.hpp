#pragma once
#include <any>
#include <atomic>
#include <memory>

#include "core/global_config.hpp"
#include "dk/utils.hpp"
#include "mavlink/imavlink.hpp"
#include "plugins/telemetry/telemetry.h"
#include "robot/irobot.hpp"

// 无人机硬件逻辑实现
// 依赖注入 DroneSpecificData，check_hover()/land() 直接读取内部数据，零 cast
class Drone : public IRobot {
   public:
    explicit Drone(std::shared_ptr<IMavlink> mavlink) : IRobot(mavlink) {}

    bool inner_check_hover(bool arm, double throttle) override {
        return arm && throttle > GlobalConfig.GetConfig().throttle_thresh;
    }

    bool cmd_vel(Eigen::Vector4d vel) override {
        mavlink_->cmd_vel(vel);
        return true;
    };

    static constexpr std::string_view ROBOT_TYPE = "DRONE";

    bool inner_is_landed(bool arm, double throttle,
                         double rangefinder) override {
        bool disarmed = !arm;
        bool ground_check = throttle >= 0 && throttle < 0.01 &&
                            rangefinder >= 0 && rangefinder < 0.5;
        return disarmed || ground_check;
    }

    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).norm();
    }

    bool is_prearm_enable() override { return true; }

    bool is_alt_enable() override { return true; }

    bool land() override {
        return mavlink_->set_mode(mavsdk::Telemetry::FlightMode::Land);
    }

    bool loiter() override {
        return mavlink_->set_mode(mavsdk::Telemetry::FlightMode::Hold);
    }

    bool takeoff(double alt) override { return mavlink_->takeoff(alt); }
};
