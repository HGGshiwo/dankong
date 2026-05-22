#pragma once
#include <bitset>
#include <memory>
#include <optional>

#include "context_config.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/future.hpp"
#include "dk/report.hpp"
#include "features/dog/command.hpp"
#include "features/tracker/tracker.hpp"
#include "mavlink/mavros.hpp"
#include "robot/irobot.hpp"
#include "utils/fixed_string64.hpp"

// 机器狗硬件逻辑实现
class Dog : public IRobot {
   private:
    std::bitset<24> hover_state_;
    RobotContext& ctx_;

   public:
    Dog(std::shared_ptr<IMavlink> mavlink, std::string udp_host, uint port,
        RobotContext& ctx)
        : IRobot(mavlink), ctx_(ctx) {
        for (int pos : {1, 2, 3, 4, 0x10}) {
            hover_state_.set(pos);
        }
    }

    bool cmd_vel(Eigen::Vector4d vel) override {
        // x,y好像是反的注意一下
        auto payload = vel_to_axis(vel.y(), vel.x(), vel.w());
        auto cmd = pack_cmd(CommandType::AXIS_COMMAND_NO_DEAD_ZONE, payload);
        ctx_.udp_client->send(cmd);
        return true;
    };

    bool inner_check_hover(unsigned int dog_state) override {
        return hover_state_.test(dog_state);
    }

    bool inner_is_landed(unsigned int dog_state) override {
        // 机器狗没有 throttle/rangefinder，通过 dog_state 判断
        return !inner_check_hover(dog_state);
    }

    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).head<2>().norm();
    }

    bool is_prearm_enable() override { return false; }

    bool is_alt_enable() override { return false; }

    bool land() override {
        loiter();
        auto data = pack_cmd(CommandType::TOGGLE_STAND_DOWN);
        ctx_.udp_client->send(data);
        return true;
    }

    bool loiter() override {
        mavlink_->cmd_vel(Eigen::Vector4d::Zero());
        return true;
    }

    bool takeoff(double alt) override {
        auto data = pack_cmd(CommandType::TOGGLE_STAND_DOWN);
        ctx_.udp_client->send(data);
        return true;
    }

   private:
    AxisCommand vel_to_axis(double vx_body, double vy_body, double yaw_rate,
                            double max_vel = 1.0, double max_yaw_rate = 1.0) {
        yaw_rate = -yaw_rate;  // yaw_rate是反的？实际是顺时针为正？？？

        double vel_scale = 1000.0 / std::max(max_vel, 1e-6);
        double yaw_scale = 1000.0 / std::max(max_yaw_rate, 1e-6);
        int left_x =
            static_cast<int>(std::clamp(vx_body * vel_scale, -1000.0, 1000.0));
        int left_y =
            static_cast<int>(std::clamp(vy_body * vel_scale, -1000.0, 1000.0));
        int right_x =
            static_cast<int>(std::clamp(yaw_rate * yaw_scale, -1000.0, 1000.0));
        return {left_x, left_y, right_x};
    }
};
