#pragma once
#include <Eigen/Dense>
#include <memory>

#include "dk/future.hpp"
#include "mavlink/imavlink.hpp"

class IContext;

class IRobot {
   protected:
    std::shared_ptr<IMavlink> mavlink_;

   public:
    IRobot(std::shared_ptr<IMavlink> mavlink) : mavlink_(mavlink) {}

    virtual bool check_hover(IContext& ctx) = 0;

    // 计算目标距离（车和机有高度上区别）
    virtual double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) = 0;

    virtual bool land(IContext& ctx) = 0;

    virtual bool is_prearm_enable() = 0;

    virtual bool is_alt_enable() = 0;

    virtual bool loiter() = 0;

    virtual bool takeoff(double alt) = 0;

    virtual bool send_cmd(std::optional<Eigen::Vector3d> pos,
                          std::optional<Eigen::Vector3d> vel,
                          std::optional<Eigen::Vector3d> acc,
                          std::optional<double> yaw,
                          std::optional<double> yaw_rate, CmdFrame frame) {
        return mavlink_->send_cmd(pos, vel, acc, yaw, yaw_rate, frame);
    }

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