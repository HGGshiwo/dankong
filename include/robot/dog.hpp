#pragma once
#include <bitset>
#include <memory>
#include <optional>

#include "Eigen/Dense"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/future.hpp"
#include "features/dog/command.hpp"
#include "features/tracker/tracker.hpp"
#include "mavlink/mavsdk_drone.hpp"
#include "robot/irobot.hpp"
#include "robot_context.hpp"
#include "utils/fixed_string64.hpp"

// 机器狗硬件逻辑实现
class Dog : public IRobot {
   private:
    std::bitset<24> hover_state_;
    RobotContext& ctx_;
    Eigen::Vector4d cmd_vel_ = Eigen::Vector4d::Zero();

   public:
    Dog(std::shared_ptr<IMavlink> mavlink, std::string udp_host, uint port,
        RobotContext& ctx)
        : IRobot(mavlink), ctx_(ctx) {
        for (int pos : {1, 2, 3, 4, 0x10}) {
            hover_state_.set(pos);
        }
    }

    bool should_arm_before_enter(const StateFlags& flags) override {
        return flags.is_waypoint && !flags.local;
    }

    bool cmd_vel(Eigen::Vector4d vel) override {
        // x,y好像是反的注意一下
        auto payload = vel_to_axis(vel.y(), vel.x(), vel.w());
        auto cmd = pack_cmd(CommandType::AXIS_COMMAND_NO_DEAD_ZONE, payload);
        ctx_.udp_client->send(cmd);
        cmd_vel_ = vel;  // 判断是否真的停止
        return true;
    };

    bool inner_check_hover(HoverArgs args) override {
        return hover_state_.test(args.dog_state.value());
    }

    bool inner_is_landed(HoverArgs args) override {
        // 机器狗没有 throttle/rangefinder，通过 dog_state 判断
        return !inner_check_hover(args);
    }

    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).head<2>().norm();
    }

    bool is_prearm_enable() override { return false; }

    bool is_alt_enable() override { return false; }

    bool land() override {
        // 必须要减速到0之后才触发降落
        spdlog::info("[Dog] land trigger");
        loiter();
        double TIMEOUT = 500;  // 最多等待500秒
        double interval = 0.1;
        double times = TIMEOUT / interval;
        double min_times = 2.0 / interval;
        ctx_.engine->call_every(
            interval, times,
            [udp_client = ctx_.udp_client, this, times,
             min_times](int idx) -> bool {
                // 如果超时了，还是调用一次
                if (!cmd_vel_.isZero() && idx != times - 1 || idx <= min_times)
                    return true;
                auto data = pack_cmd(CommandType::TOGGLE_STAND_DOWN);
                udp_client->send(data);
                spdlog::info("[Dog] land done!");
                return false;
            });

        return true;
    }

    bool loiter() override {
        ctx_.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                   std::nullopt, CmdFrame::BODY);
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
