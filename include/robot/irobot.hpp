#pragma once
#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "core/base_tracker.hpp"
#include "dk/future.hpp"
#include "dk/state.hpp"
#include "mavlink/imavlink.hpp"
#include "plugins/telemetry/telemetry.h"

struct HoverArgs {
    std::optional<unsigned int> dog_state = std::nullopt;
    std::optional<bool> arm = std::nullopt;
    std::optional<double> throttle = std::nullopt;
    std::optional<unsigned int> gear = std::nullopt;
    std::optional<double> rangefinder_alt = std::nullopt;
};

struct StateFlags {
    bool is_takeoff = false;
    bool is_waypoint = false;
    bool is_follow = false;
    bool is_posvel = false;
    bool is_hover = false;
    bool is_land = false;
    bool local = false;
};

// IRobot: 纯虚接口，不依赖任何 Context 类型
// 每个具体机器人在构造时注入其专属数据，方法内部直接读取，无需 Context 参数
class IRobot : public ITrackerRuntime {
   protected:
    std::shared_ptr<IMavlink> mavlink_;

   public:
    IRobot(std::shared_ptr<IMavlink> mavlink) : mavlink_(mavlink) {}
    virtual ~IRobot() = default;

    virtual bool should_arm_before_enter(const StateFlags& flags) = 0;

    // 计算目标距离（无人机与机器狗在 Z 轴上的处理不同）
    virtual double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) = 0;

    // 降落（无人机切换 LAND 模式，机器狗发送蹲下指令）
    virtual bool land() = 0;

    virtual bool is_prearm_enable() = 0;

    virtual bool is_alt_enable() = 0;

    virtual bool loiter() = 0;

    virtual bool takeoff(double alt) = 0;

    virtual bool cmd_vel(Eigen::Vector4d) = 0;

    template <typename T>
    bool check_hover(T& ctx) {
        constexpr std::string_view robot_type = T::ROBOT_TYPE;
        HoverArgs args;
        if constexpr (robot_type == "DOG") {
            args.dog_state = ctx.dog_state;
        } else if constexpr (robot_type == "DRONE") {
            args.arm = ctx.arm.load();
            args.throttle = ctx.throttle.load();
        } else if constexpr (robot_type == "CAR") {
            args.gear = ctx.gear.load();
        } else if constexpr (robot_type == "GO2") {
        } else {
            return false;
        }
        return inner_check_hover(args);
    }

    template <typename T>
    bool is_landed(T& ctx) {
        constexpr std::string_view robot_type = T::ROBOT_TYPE;
        HoverArgs args;
        if constexpr (robot_type == "DOG") {
            args.dog_state = ctx.dog_state;
        } else if constexpr (robot_type == "DRONE") {
            args.arm = ctx.arm.load();
            args.throttle = ctx.throttle.load();
            args.rangefinder_alt = ctx.rangefinder_alt.load();
        } else if constexpr (robot_type == "CAR") {
            args.gear = ctx.gear.load();
        } else if constexpr (robot_type == "GO2") {
        } else {
            return false;
        }
        return inner_is_landed(args);
    }

    virtual bool inner_check_hover(HoverArgs) = 0;

    virtual bool inner_is_landed(HoverArgs) = 0;

    bool set_mode(const mavsdk::Telemetry::FlightMode& mode) {
        return mavlink_->set_mode(mode);
    }
    void set_target_type(VehicleType type) { mavlink_->set_target_type(type); }

    template <typename T>
    T get_param(std::string name, T value) {
        ApmParam data = mavlink_->get_param(name, value);
        return IMavlink::unpack<T>(data);
    }

    virtual bool set_stream_rate(int stream_id, int rate) {
        return mavlink_->set_stream_rate(stream_id, rate);
    }

    virtual bool set_msg_interval(int stream_id, int rate) {
        return mavlink_->set_msg_interval(stream_id, rate);
    }

    virtual bool arm() { return mavlink_->arm(); }

    virtual bool disarm() { return mavlink_->disarm(); }

    virtual bool reboot_fcu() { return mavlink_->reboot_fcu(); }

    virtual bool run_prearm_checks() { return mavlink_->run_prearm_checks(); }

    virtual bool pull_params() { return mavlink_->pull_params(); }

    virtual void send_rtcm_data(const uint8_t* data, size_t size) {
        mavlink_->send_rtcm_data(data, size);
    }

    virtual nlohmann::json get_all_params() {
        return mavlink_->get_all_params();
    }

    virtual bool set_param(std::string name, ApmParam value) {
        return mavlink_->set_param(name, value);
    }

    virtual bool is_prearm_msg(const std::string& text) {
        return mavlink_->is_prearm_msg(text);
    }

    virtual bool check_sensor_health(uint32_t sensor_health) {
        return mavlink_->check_sensor_health(sensor_health);
    }
};
