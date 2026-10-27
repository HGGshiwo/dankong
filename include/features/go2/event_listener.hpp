#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include "dk/event_listener.hpp"
#include "events.hpp"
#include "robot_context.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "utils/state_registry.hpp"

class Go2EventListener
    : public dk::BaseEventListener<RobotContext, Go2EventListener> {
   public:
    using AllowedEvents = std::tuple<ToggleLocOutputEvent>;

    // 位姿来自 MAVLink -> Context，这里按开关定频输出 /loc_base 和
    // odom->base_link TF (默认关闭, 经 HTTP /loc_output/enable 控制)
    void on_tick(double dt, RobotContext& ctx) override {
        if (!is_hz(kPublishHz)) return;
        if (!ctx.loc_output_enable.load()) return;
        if (!ctx.odom_ok.load()) return;

        if (!pub_) {
            ros::NodeHandle nh;
            pub_ = nh.advertise<nav_msgs::Odometry>("/loc_base", 10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        }

        auto pos_enu = ctx.pos_enu.load();
        auto q = ctx.orientation.load();  // ENU_FLU
        auto vel_enu = ctx.vel_enu.load();
        auto stamp = ros::Time::now();

        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = pos_enu.x();
        msg.pose.pose.position.y = pos_enu.y();
        msg.pose.pose.position.z = pos_enu.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        // 保持 /loc_base 原有约定：twist.linear 为 ENU 世界系速度
        msg.twist.twist.linear.x = vel_enu.x();
        msg.twist.twist.linear.y = vel_enu.y();
        msg.twist.twist.linear.z = vel_enu.z();

        pub_.publish(msg);

        geometry_msgs::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = pos_enu.x();
        tf.transform.translation.y = pos_enu.y();
        tf.transform.translation.z = pos_enu.z();
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(tf);
    }

    void on_event(const ToggleLocOutputEvent& event, RobotContext& ctx) {
        ctx.loc_output_enable.store(event.enable);
        event.resolve({"success", event.enable ? "loc_output enable"
                                               : "loc_output disable"});
    }

   private:
    static constexpr double kPublishHz = 30.0;
    ros::Publisher pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
