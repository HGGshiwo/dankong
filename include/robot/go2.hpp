#pragma once
#include <geometry_msgs/TwistStamped.h>

#include <boost/beast/http/verb.hpp>
#include <memory>
#include <optional>
#include <string>

#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/future.hpp"
#include "features/dog/command.hpp"
#include "features/tracker/tracker.hpp"
#include "nlohmann/json.hpp"
#include "robot/irobot.hpp"
#include "robot_context.hpp"
#include "utils/exception.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/request.hpp"

// 机器狗硬件逻辑实现
class Go2 : public IRobot {
   private:
    RobotContext& ctx_;
    std::atomic<bool> standing_{false};
    ros::Publisher cmd_vel_pub_;

   public:
    Go2(std::shared_ptr<IMavlink> mavlink, RobotContext& ctx)
        : IRobot(mavlink), ctx_(ctx) {
        // ctx.set_waypoint_goal = [this](Eigen::Vector3d target_enu,
        //                                std::optional<double> vel) {
        //     this->push_message(fmt::format("前往地图点{}米{}米",
        //     target_enu.x(),
        //                                    target_enu.y()));
        // };
        ros::NodeHandle nh;
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    bool should_arm_before_enter(const StateFlags& flags) override {
        return flags.is_waypoint || flags.is_follow || flags.is_posvel ||
               flags.is_hover;
    }

    void push_message(std::string msg) {
        auto future = send_request(
            ctx_.engine, http::verb::post,
            GlobalConfig.GetConfig().go2_server_url.get(),
            nlohmann::json{
                {"content", msg}, {"sender", "user"}, {"target", "legacy"}},
            true);
        future
            .then([](HttpResponse res) {
                spdlog::info("[Push message] success={}, data={}, error={}",
                             res.success(), res.body, res.error_msg);
            })
            .catch_error([](std::exception_ptr exp) {
                spdlog::error("[Push message]: {}", get_error_message(exp));
            });
    }

    // 计算目标距离（无人机与机器狗在 Z 轴上的处理不同）
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).head<2>().norm();
    };

    // 降落（无人机切换 LAND 模式，机器狗发送蹲下指令）
    bool land() override {
        push_message("趴下");
        standing_.store(false);
        return true;
    };

    bool is_prearm_enable() override { return false; };

    bool is_alt_enable() override { return false; };

    bool loiter() override { return true; };

    bool takeoff(double alt) override {
        push_message("站起来");
        standing_.store(true);
        return true;
    };

    bool cmd_vel(Eigen::Vector4d data) override {
        auto msg = geometry_msgs::Twist();
        msg.linear.x = data.x();
        msg.linear.y = data.y();
        msg.linear.z = data.z();
        msg.angular.z = data.w();

        cmd_vel_pub_.publish(msg);
        return true;
    };

    bool inner_check_hover(HoverArgs args) override {
        bool a = standing_.load();
        return a;
    };

    bool inner_is_landed(HoverArgs args) override {
        return !inner_check_hover(args);
    };

    bool arm() override {
        push_message("解锁");
        ctx_.arm.store(true);
        ctx_.engine->dispatch_internal(ArmEvent{true});

        return true;
    }

    bool disarm() override {
        push_message("锁定");
        ctx_.arm.store(false);
        ctx_.engine->dispatch_internal(ArmEvent{false});
        return true;
    }
};
