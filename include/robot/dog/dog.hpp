#pragma once
#include <Eigen/Dense>
#include <bitset>
#include <memory>
#include <optional>

#include "Eigen/src/Core/Matrix.h"
#include "data.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/future.hpp"
#include "mavlink/mavros.hpp"
#include "robot/robot_base.hpp"
#include "robot/utils.hpp"
#include "robot_base.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"
#include "utils.hpp"

class Dog : public IRobot, ITrackerRuntime {
    std::bitset<24> hover_state_;
    std::shared_ptr<ThreadedTracker> tracker_;
    RobotContext& ctx_;
    UdpClient client_;

   public:
    Dog(std::shared_ptr<IMavlink> mavlink, RobotContext& ctx,
        std::string udp_host, uint port)
        : IRobot(mavlink), ctx_(ctx), client_(UdpClient(udp_host, port)) {
        for (int pos : {1, 2, 3, 4, 0x10}) {
            hover_state_.set(pos);
        }
        tracker_ = std::make_shared<ThreadedTracker>(TrackingConfig(), this);
    }

    RobotContext& get_context() override { return ctx_; }

    bool send_cmd(std::optional<Eigen::Vector3d> pos,
                  std::optional<Eigen::Vector3d> vel,
                  std::optional<Eigen::Vector3d> acc, std::optional<double> yaw,
                  std::optional<double> yaw_rate, CmdFrame frame) override;

    void cmd_vel(Eigen::Vector4d vel) override;

    bool check_hover(IContext& ctx) override;

    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override;

    bool is_prearm_enable() override;

    bool is_alt_enable() override;

    bool land(IContext& ctx) override;

    bool loiter() override;

    bool takeoff(double alt) override;
};

inline double HOVER_THRESH = 1.0;

double Dog::get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
    return (pos - goal).head<2>().norm();
}

bool Dog::is_prearm_enable() {
    return false;
}

bool Dog::is_alt_enable() {
    return false;
}

bool Dog::check_hover(IContext& ctx) {
    auto dog_ctx = dynamic_cast<DogContext*>(&ctx);
    // 【必须判空】！确保它真的是 DogContext
    if (dog_ctx != nullptr) {
        return hover_state_.test(dog_ctx->dog_state);
    } else {
        // 如果不是 DogContext，你需要决定返回什么，或者打一条报错日志
        std::cerr << "[Error] 传入了错误的 Context 类型！\n";
        return false;
    }
}

bool Dog::loiter() {
    tracker_->send_cmd(std::nullopt, Eigen::Vector3d::Zero(), std::nullopt,
                       std::nullopt, CmdFrame::BODY);
}

bool Dog::land(IContext& ctx) {
    loiter();
    auto data = pack_cmd(CommandType::TOGGLE_STAND_DOWN);
    client_.send(data);
    return true;
}

bool Dog::takeoff(double alt) {
    auto data = pack_cmd(CommandType::TOGGLE_STAND_DOWN);
    client_.send(data);
    return true;
}

bool Dog::send_cmd(std::optional<Eigen::Vector3d> pos,
                   std::optional<Eigen::Vector3d> vel,
                   std::optional<Eigen::Vector3d> acc,
                   std::optional<double> yaw, std::optional<double> yaw_rate,
                   CmdFrame frame) {
    tracker_->send_cmd(pos, vel, yaw, yaw_rate, frame);
    return true;
}