#pragma once
#include "states/state_utils.hpp"
#ifdef USE_ROS
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <array>
#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "./events.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"

// 和控制相关的事件监听器
class PlandEventListener
    : public dk::BaseEventListener<RobotContext, PlandEventListener> {
   public:
    PlandEventListener() {
        // 话题名与 pland_controller 的注入订阅保持一致 (读它的参数, 默认同值)
        nh_.param<std::string>("/pland_controller/inject_target_pose_topic",
                               inject_pose_topic_, "/pland/inject_target_pose");
        nh_.param<std::string>("/pland_controller/inject_target_vel_topic",
                               inject_vel_topic_, "/pland/inject_target_vel");
        inject_pose_pub_ =
            nh_.advertise<sensor_msgs::NavSatFix>(inject_pose_topic_, 10);
        inject_vel_pub_ =
            nh_.advertise<geometry_msgs::TwistStamped>(inject_vel_topic_, 10);
    }

    using AllowedEvents = std::tuple<StartPlandDetectEvent, SetPlandTarget,
                                     StartOffsetEstimate, StopOffsetEstimate>;

    void on_event(const StartPlandDetectEvent& event, RobotContext& ctx) {
        ctx.land_detector->start(30);
        event.resolve({"success", "OK"});
    }

    void on_event(const StartOffsetEstimate& event, RobotContext& ctx) {
        ctx.land_detector->start(30, event.x, event.y, event.z);
        event.resolve({"success", "OK"});
    }

    void on_event(const StopOffsetEstimate& event, RobotContext& ctx) {
        auto out = ctx.land_detector->stop(event.save);
        event.resolve({"success", out});
    }

    void on_event(const SetPlandTarget& event, RobotContext& ctx) {
        Eigen::Vector3d pos;
        if (!event.position.has_value()) {
            pos = ctx.pos_enu.load();
        } else {
            if (!ctx.odom_ok) {
                event.resolve({"success", "Odom NOT OK"});
                return;
            }
            auto lla = event.position.value();
            pos = state_utils::gps_to_enu(ctx.lon_lat_alt.load(),
                                          ctx.pos_enu.load(), lla);
        }
        spdlog::info("[Pland] Set Target: [{:.2f}, {:.2f}, {:.2f}]", pos.x(),
                     pos.y(), pos.z());

        ctx.pland_target.store(PlandTarget{
            ctx.engine->get_time_provider()->now(), pos, event.velocity});

        // 转发原始注入 GPS 给 pland_controller 的 inject 通道,
        // 坐标转换由下游完成 (与它从其他来源收到注入 GPS 的语义一致)
        Eigen::Vector3d lla = event.position.value_or(ctx.lon_lat_alt.load());
        sensor_msgs::NavSatFix gps;
        gps.header.stamp = ros::Time::now();
        gps.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        gps.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        gps.longitude = lla.x();
        gps.latitude = lla.y();
        gps.altitude = lla.z();
        inject_pose_pub_.publish(gps);

        if (event.velocity.has_value()) {
            geometry_msgs::TwistStamped vel;
            vel.header.stamp = ros::Time::now();
            vel.twist.linear.x = event.velocity->x();
            vel.twist.linear.y = event.velocity->y();
            vel.twist.linear.z = event.velocity->z();
            inject_vel_pub_.publish(vel);
        }

        event.resolve({"success", "OK"});
    }

   private:
    ros::NodeHandle nh_;
    ros::Publisher inject_pose_pub_, inject_vel_pub_;
    std::string inject_pose_topic_, inject_vel_topic_;
};
#endif