#pragma once
#include <geometry_msgs/TwistStamped.h>

#include <boost/beast/http/verb.hpp>
#include <cstdint>
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
#include "ros/publisher.h"
#include "std_msgs/String.h"
#include "utils/exception.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/request.hpp"

// 机器狗硬件逻辑实现
class Go2 : public IRobot {
   private:
    RobotContext& ctx_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher action_pub_;

   public:
    Go2(std::shared_ptr<IMavlink> mavlink, RobotContext& ctx)
        : IRobot(mavlink), ctx_(ctx) {
        ros::NodeHandle nh;
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        action_pub_ = nh.advertise<std_msgs::String>("/dank/action", 1);
    }

    bool should_arm_before_enter(const StateFlags& flags) override {
        return flags.is_waypoint || flags.is_follow || flags.is_posvel ||
               flags.is_hover;
    }

    // 计算目标距离（无人机与机器狗在 Z 轴上的处理不同）
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).head<2>().norm();
    };

    // 降落（无人机切换 LAND 模式，机器狗发送蹲下指令）
    bool land() override {
        std_msgs::String data;
        data.data = "趴下";
        action_pub_.publish(data);
        return true;
    };

    bool is_prearm_enable() override { return false; };

    bool is_alt_enable() override { return false; };

    bool loiter() override {
        ctx_.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                   std::nullopt, CmdFrame::BODY);
        return true;
    };

    bool takeoff(double alt) override {
        std_msgs::String data;
        data.data = "站立";
        action_pub_.publish(data);
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
        auto data = ctx_.go2_state.load();
        return data == 1;
    };

    bool inner_is_landed(HoverArgs args) override {
        return !inner_check_hover(args);
    };

    bool arm() override {
        std_msgs::String data;
        data.data = "解锁";
        action_pub_.publish(data);
        return true;
    }

    bool disarm() override {
        std_msgs::String data;
        data.data = "锁定";
        action_pub_.publish(data);
        return true;
    }
};
