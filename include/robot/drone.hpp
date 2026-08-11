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

    bool should_arm_before_enter(const StateFlags& flags) override {
        return flags.is_takeoff || flags.is_waypoint || flags.is_follow ||
               flags.is_posvel || flags.is_hover || flags.is_land;
    }

    bool inner_check_hover(HoverArgs args) override {
        return args.arm.value() &&
               args.throttle.value() > GlobalConfig.GetConfig().throttle_thresh;
    }

    bool cmd_vel(Eigen::Vector4d vel) override {
        mavlink_->cmd_vel(vel);
        return true;
    };

    static constexpr std::string_view ROBOT_TYPE = "DRONE";

    bool inner_is_landed(HoverArgs args) override {
        bool disarmed = !args.arm.value();
        bool ground_check = args.throttle.value() >= 0 &&
                            args.throttle.value() < 0.01 &&
                            args.rangefinder_alt.value() >= 0 &&
                            args.rangefinder_alt.value() < 0.5;
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
