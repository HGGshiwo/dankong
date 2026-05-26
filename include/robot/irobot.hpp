#pragma once
#include <Eigen/Dense>
#include <memory>

#include "core/base_tracker.hpp"
#include "dk/future.hpp"
#include "mavlink/imavlink.hpp"

// IRobot: 纯虚接口，不依赖任何 Context 类型
// 每个具体机器人在构造时注入其专属数据，方法内部直接读取，无需 Context 参数
class IRobot : public ITrackerRuntime {
   protected:
    std::shared_ptr<IMavlink> mavlink_;

   public:
    IRobot(std::shared_ptr<IMavlink> mavlink) : mavlink_(mavlink) {}
    virtual ~IRobot() = default;

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
        if constexpr (robot_type == "DOG") {
            return inner_check_hover(ctx.dog_state);
        } else if constexpr (robot_type == "DRONE") {
            return inner_check_hover(ctx.arm.load(), ctx.throttle.load());
        } else {
            return false;
        }
    }

    template <typename T>
    bool is_landed(T& ctx) {
        constexpr std::string_view robot_type = T::ROBOT_TYPE;
        if constexpr (robot_type == "DOG") {
            return inner_is_landed(ctx.dog_state);
        } else if constexpr (robot_type == "DRONE") {
            return inner_is_landed(ctx.arm.load(), ctx.throttle.load(),
                                   ctx.rangefinder_alt.load());
        }
        return false;
    }

    virtual bool inner_check_hover(bool arm, double throttle) { return false; };

    virtual bool inner_check_hover(unsigned int state) { return false; };

    virtual bool inner_is_landed(unsigned int state) { return false; };

    virtual bool inner_is_landed(bool arm, double throttle,
                                 double rangefinder) {
        return false;
    };

    bool set_mode(const FixedString64& mode) {
        return mavlink_->set_mode(mode);
    }

    template <typename T>
    T get_param(std::string name, T value) {
        ApmParam data = mavlink_->get_param(name, value);
        return IMavlink::unpack<T>(data);
    }

    bool set_stream_rate(int rate) { return mavlink_->set_stream_rate(rate); }

    bool arm() { return mavlink_->arm(); }

    bool disarm() { return mavlink_->disarm(); }

    bool reboot_fcu() { return mavlink_->reboot_fcu(); }

    bool run_prearm_checks() { return mavlink_->run_prearm_checks(); }

    bool pull_params() { return mavlink_->pull_params(); }

    nlohmann::json get_all_params() { return mavlink_->get_all_params(); }

    bool set_param(std::string name, ApmParam value) {
        return mavlink_->set_param(name, value);
    }
};
