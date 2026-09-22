#pragma once

#ifdef USE_ROS1
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <string>

#include "core/flight_mode.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"

// MAVROS 兼容桥: 让 dankong 自身对外提供 mavros 风格的话题/服务,
// 供 pland_controller 等按 mavros 接口编写的下游节点直接使用, 无需真实 mavros。
//
// 发布 (数据来自 MAVSDK 遥测写入的 ctx):
//   /mavros/local_position/odom           nav_msgs/Odometry    (pose/twist 均为
//   ENU) /mavros/global_position/global        sensor_msgs/NavSatFix (altitude
//   为 AMSL) /mavros/global_position/rel_alt       std_msgs/Float64
//   /mavros/distance_sensor/rangefinder_pub sensor_msgs/Range
// 订阅 (直通转发给 FCU):
//   /mavros/setpoint_raw/local            mavros_msgs/PositionTarget
// 服务:
//   /mavros/set_mode                      mavros_msgs/SetMode
//   /mavros/cmd/command                   mavros_msgs/CommandLong
class MavrosBridge {
   public:
    explicit MavrosBridge(RobotContext& ctx) : ctx_(ctx), stopping_(false) {
        ros::NodeHandle pnh("~");

        pnh.param<std::string>("odom_topic", odom_topic_,
                               "/mavros/local_position/odom");
        pnh.param<std::string>("global_topic", global_topic_,
                               "/mavros/global_position/global");
        pnh.param<std::string>("rel_alt_topic", rel_alt_topic_,
                               "/mavros/global_position/rel_alt");
        pnh.param<std::string>("range_topic", range_topic_,
                               "/mavros/distance_sensor/rangefinder_pub");
        pnh.param<std::string>("setpoint_topic", setpoint_topic_,
                               "/mavros/setpoint_raw/local");
        pnh.param<std::string>("frame_id", frame_id_, "map");
        pnh.param<std::string>("child_frame_id", child_frame_id_, "base_link");
        pnh.param<double>("telemetry_rate", telemetry_rate_, 30.0);
        pnh.param<double>("range_min", range_min_, 0.1);
        pnh.param<double>("range_max", range_max_, 20.0);
        pnh.param<double>("range_fov", range_fov_, 0.1);
        pnh.param<int>("range_radiation_type", range_radiation_type_,
                       sensor_msgs::Range::INFRARED);

        telemetry_rate_ = std::max(1.0, telemetry_rate_);

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
        global_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(global_topic_, 10);
        rel_alt_pub_ = nh_.advertise<std_msgs::Float64>(rel_alt_topic_, 10);
        range_pub_ = nh_.advertise<sensor_msgs::Range>(range_topic_, 10);

        setpoint_sub_ = nh_.subscribe<mavros_msgs::PositionTarget>(
            setpoint_topic_, 10, &MavrosBridge::setpoint_callback, this);

        set_mode_srv_ = nh_.advertiseService("/mavros/set_mode",
                                             &MavrosBridge::set_mode_cb, this);
        command_srv_ = nh_.advertiseService("/mavros/cmd/command",
                                            &MavrosBridge::command_cb, this);

        timer_ = nh_.createTimer(ros::Duration(1.0 / telemetry_rate_),
                                 &MavrosBridge::telemetry_timer, this);

        spdlog::info(
            "[MavrosBridge] started, publishing at {:.0f} Hz: {}, {}, "
            "{}, {}",
            telemetry_rate_, odom_topic_, global_topic_, rel_alt_topic_,
            range_topic_);
    }

    // 先停掉所有 ros 回调入口再析构, 避免关闭期间 spinner 线程访问已析构的 ctx
    ~MavrosBridge() {
        stopping_.store(true, std::memory_order_acquire);
        timer_.stop();
        setpoint_sub_.shutdown();
        set_mode_srv_.shutdown();
        command_srv_.shutdown();
        odom_pub_.shutdown();
        global_pub_.shutdown();
        rel_alt_pub_.shutdown();
        range_pub_.shutdown();
    }

   private:
    // ---------------- 遥测发布 ----------------
    void telemetry_timer(const ros::TimerEvent&) {
        if (stopping_.load(std::memory_order_acquire)) return;
        publish_odom();
        publish_global();
        publish_rel_alt();
        publish_range();
    }

    void publish_odom() {
        if (!ctx_.fcu_connected.load()) return;

        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame_id_;
        msg.child_frame_id = child_frame_id_;

        // mavros 约定: local_position/odom 的 pose 与 twist 都是 ENU
        auto pos_enu = ctx_.pos_enu.load();
        msg.pose.pose.position.x = pos_enu.x();
        msg.pose.pose.position.y = pos_enu.y();
        msg.pose.pose.position.z = pos_enu.z();

        auto q = ctx_.orientation.load();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        auto vel_enu = ctx_.vel_enu.load();
        msg.twist.twist.linear.x = vel_enu.x();
        msg.twist.twist.linear.y = vel_enu.y();
        msg.twist.twist.linear.z = vel_enu.z();

        odom_pub_.publish(msg);
    }

    void publish_global() {
        if (!ctx_.fcu_connected.load()) return;

        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        msg.status.status = ctx_.gps_fix_type.load() >= 3
                                ? sensor_msgs::NavSatStatus::STATUS_FIX
                                : sensor_msgs::NavSatStatus::STATUS_NO_FIX;

        // ctx.lon_lat_alt 的布局为 (lon, lat, rel_alt)
        auto lon_lat_alt = ctx_.lon_lat_alt.load();
        msg.longitude = lon_lat_alt.x();
        msg.latitude = lon_lat_alt.y();
        msg.altitude = ctx_.amsl_alt.load();

        global_pub_.publish(msg);
    }

    void publish_rel_alt() {
        if (!ctx_.fcu_connected.load()) return;

        std_msgs::Float64 msg;
        msg.data = ctx_.lon_lat_alt.load().z();
        rel_alt_pub_.publish(msg);
    }

    void publish_range() {
        double range = ctx_.rangefinder_alt.load();
        if (range < 0) return;  // 尚无有效测距数据

        sensor_msgs::Range msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = child_frame_id_;
        msg.radiation_type = static_cast<uint8_t>(range_radiation_type_);
        msg.field_of_view = static_cast<float>(range_fov_);
        msg.min_range = static_cast<float>(range_min_);
        msg.max_range = static_cast<float>(range_max_);
        msg.range = static_cast<float>(range);

        range_pub_.publish(msg);
    }

    // ---------------- setpoint 直通转发 ----------------
    static double normalize_yaw(double yaw) {
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        return yaw;
    }

    // mavros 约定: ROS 侧输入 ENU/FLU, 发送给 FCU 前转成 NED/FRD
    void setpoint_callback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
        if (stopping_.load(std::memory_order_acquire)) return;
        if (!ctx_.robot) return;

        Eigen::Vector3d pos_ned = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_ned = Eigen::Vector3d::Zero();
        double yaw = 0, yaw_rate = 0;

        switch (msg->coordinate_frame) {
            case mavros_msgs::PositionTarget::FRAME_LOCAL_NED:
                pos_ned = {msg->position.y, msg->position.x, -msg->position.z};
                vel_ned = {msg->velocity.y, msg->velocity.x, -msg->velocity.z};
                yaw = normalize_yaw(M_PI_2 - msg->yaw);
                yaw_rate = -msg->yaw_rate;
                break;
            case mavros_msgs::PositionTarget::FRAME_BODY_NED:
                pos_ned = {msg->position.x, -msg->position.y, -msg->position.z};
                vel_ned = {msg->velocity.x, -msg->velocity.y, -msg->velocity.z};
                yaw = normalize_yaw(-msg->yaw);
                yaw_rate = -msg->yaw_rate;
                break;
            default:
                spdlog::warn("[MavrosBridge] unsupported setpoint frame: {}",
                             msg->coordinate_frame);
                return;
        }

        // MAVSDK 的 offboard 消息里 yaw 始终为 active; 上游忽略 yaw 时
        // 填当前 NED 航向, 保证不引起额外转头
        if (msg->type_mask & 1024 /*IGNORE_YAW*/) {
            yaw = ctx_.yaw_ned.load();
        }

        ctx_.robot->send_position_target(
            msg->coordinate_frame, msg->type_mask, pos_ned, vel_ned,
            static_cast<float>(yaw), static_cast<float>(yaw_rate));
    }

    // ---------------- 服务 ----------------
    bool set_mode_cb(mavros_msgs::SetMode::Request& req,
                     mavros_msgs::SetMode::Response& res) {
        if (stopping_.load(std::memory_order_acquire)) {
            res.mode_sent = false;
            return true;
        }
        if (!ctx_.robot) {
            res.mode_sent = false;
            return true;
        }
        try {
            auto mode = FlightMode::create(req.custom_mode);
            res.mode_sent = ctx_.robot->set_mode(mode.mode_raw);
        } catch (const std::exception& e) {
            spdlog::warn("[MavrosBridge] set_mode '{}': {}", req.custom_mode,
                         e.what());
            res.mode_sent = false;
        }
        return true;
    }

    bool command_cb(mavros_msgs::CommandLong::Request& req,
                    mavros_msgs::CommandLong::Response& res) {
        if (stopping_.load(std::memory_order_acquire)) {
            res.success = false;
            res.result = 4;  // MAV_RESULT_FAILED
            return true;
        }
        if (!ctx_.robot) {
            res.success = false;
            res.result = 4;  // MAV_RESULT_FAILED
            return true;
        }
        auto r = ctx_.robot->send_command_long(
            req.command, req.confirmation, req.param1, req.param2, req.param3,
            req.param4, req.param5, req.param6, req.param7);
        res.success = r.success;
        res.result = r.mav_result;
        return true;
    }

    RobotContext& ctx_;
    std::atomic<bool> stopping_;
    ros::NodeHandle nh_;

    std::string odom_topic_, global_topic_, rel_alt_topic_, range_topic_,
        setpoint_topic_;
    std::string frame_id_, child_frame_id_;
    double telemetry_rate_ = 30.0;
    double range_min_ = 0.1, range_max_ = 20.0, range_fov_ = 0.1;
    int range_radiation_type_ = sensor_msgs::Range::INFRARED;

    ros::Publisher odom_pub_, global_pub_, rel_alt_pub_, range_pub_;
    ros::Subscriber setpoint_sub_;
    ros::ServiceServer set_mode_srv_, command_srv_;
    ros::Timer timer_;
};
#endif  // USE_ROS1
