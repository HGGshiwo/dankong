#pragma once
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <optional>

#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "features/tracker/tracker.hpp"
#include "landing_detector.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_context.hpp"
#include "ros/publisher.h"
#include "spdlog/spdlog.h"

class PlandController {
    int invalid_time_ = 0;

    ros::Publisher pnp_pub_;
    ros::Publisher los_pub_;
    ros::Publisher fused_pub_;

   public:
    PlandController() {
        ros::NodeHandle nh;
        pnp_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/pnp", 10);
        los_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/los", 10);
        fused_pub_ = nh.advertise<nav_msgs::Odometry>("/pland/fused", 10);
    }

    void control_step(RobotContext& ctx, const DetectorResult& result) {
        double max_vel_z = 0;
        double z_body_target = 0;
        double current_z = ctx.rangefinder_alt.load();
        Eigen::Vector3d ff_vel =
            Eigen::Vector3d::Zero();  // result.velocity.template head<3>();

        double dynamic_xy_thresh = 0.3 + 0.2 * current_z;
        Eigen::Vector4d error = result.relative_pos;

        auto pos_enu = ctx.pos_enu.load();

        // 动态计算 FOV 保护限速 (宏观底线)
        double limit_start_z =
            GlobalConfig.GetConfig().pland_limit_start_z.get();
        double min_cruise_speed =
            GlobalConfig.GetConfig().pland_min_cruise_speed.get();
        double cruise_speed_xy =
            GlobalConfig.GetConfig().pland_cruise_speed_xy.get();

        if (current_z > 0 && current_z < limit_start_z) {
            cruise_speed_xy =
                min_cruise_speed + (cruise_speed_xy - min_cruise_speed) *
                                       (current_z / limit_start_z);
        } else if (current_z <= 0) {
            cruise_speed_xy = min_cruise_speed;
        }

        // 动态计算 Kp 衰减系数 gamma (微观柔性)
        double gamma = GlobalConfig.GetConfig().pland_gamma.get();
        double decay_start_z =
            GlobalConfig.GetConfig().pland_decay_start_z.get();
        double min_gamma = GlobalConfig.GetConfig().pland_min_gamma.get();
        if (current_z > 0 && current_z < decay_start_z) {
            gamma = min_gamma + (1.0 - min_gamma) * (current_z / decay_start_z);
        } else if (current_z <= 0) {
            gamma = min_gamma;
        }

        if (std::abs(current_z) <=
            GlobalConfig.GetConfig().touchdown_z_thresh) {
            max_vel_z = GlobalConfig.GetConfig().touchdown_velocity;
            z_body_target = -current_z - 0.5;
            ctx.tracker->send_pos_cmd(
                {result.relative_pos.x(), result.relative_pos.y(),
                 z_body_target},
                result.relative_pos.w(), std::nullopt,
                Eigen::Vector3d{ff_vel.x(), ff_vel.y(), 0.0}, cruise_speed_xy,
                max_vel_z, CmdFrame::BODY, {gamma, 1.0, gamma},
                GlobalConfig.GetConfig().pland_max_acc_xy.get(),
                GlobalConfig.GetConfig().pland_max_devel_xy.get());
            return;
        }

        if (!result.is_valid) {
            invalid_time_ += 1;
        } else {
            invalid_time_ = 0;
        };
        if (invalid_time_ > 20) {
            // 如果无效则接近到lost_target_alt高度
            ctx.tracker->send_pos_cmd(
                {pos_enu.x(), pos_enu.y(),
                 GlobalConfig.GetConfig().lost_target_alt},
                std::nullopt, std::nullopt, std::nullopt, std::nullopt, 0.5,
                CmdFrame::ENU);
            return;
        }

        std::vector<std::optional<Eigen::Vector3d>> pose{
            result.pnp_pos, result.los_pos, result.fused_pos};
        std::vector<ros::Publisher> pub{pnp_pub_, los_pub_, fused_pub_};
        for (int i = 0; i < 3; ++i) {
            std::optional<Eigen::Vector3d> ps = pose[i];
            auto pb = pub[i];
            if (ps.has_value()) {
                Eigen::Vector3d p = ps.value();
                nav_msgs::Odometry pnp_odom;
                pnp_odom.header.frame_id = "base_link";
                pnp_odom.pose.pose.position.x = p.x();
                pnp_odom.pose.pose.position.y = p.y();
                pnp_odom.pose.pose.position.z = -ctx.pos_enu.load().z();
                pnp_odom.twist.twist.linear.x = result.velocity.x();
                pnp_odom.twist.twist.linear.x = result.velocity.y();
                pnp_odom.twist.twist.linear.x = result.velocity.z();
                pb.publish(pnp_odom);
            }
        }

        bool xy_aligned = (std::abs(error.x()) <= dynamic_xy_thresh) &&
                          (std::abs(error.y()) <= dynamic_xy_thresh) &&
                          (std::abs(error.w()) <=
                           GlobalConfig.GetConfig().yaw_align_thresh.get());

        if (xy_aligned) {
            double takeoff_alt = ctx.takeoff_lon_lat_alt.load().z();
            double x = std::min(1.0, std::abs(error.z() / takeoff_alt));
            max_vel_z = GlobalConfig.GetConfig().pland_vz0 *
                        std::log10(1.0 + GlobalConfig.GetConfig().pland_a * x +
                                   GlobalConfig.GetConfig().pland_b * x * x);

            z_body_target = -current_z - 0.5;  // 稍微往下点
        } else {
            max_vel_z = 0.0;
        }

        // [修改 3] 把算好的 cruise_speed_xy 和 gamma 塞进指令中
        ctx.tracker->send_pos_cmd(
            {error.x(), error.y(), z_body_target}, result.relative_pos.w(),
            std::nullopt, Eigen::Vector3d{ff_vel.x(), ff_vel.y(), 0.0},
            cruise_speed_xy, max_vel_z, CmdFrame::BODY, {gamma, 1.0, gamma},
            GlobalConfig.GetConfig().pland_max_acc_xy.get(),
            GlobalConfig.GetConfig().pland_max_devel_xy.get());
    }
};

class PlandFeature {
   public:
    static void init(RobotContext& ctx) {
        ros::NodeHandle nh;
        ros::Publisher image_pub_;
        image_pub_ = nh.advertise<sensor_msgs::Image>(
            GlobalConfig.GetConfig().pland_detect_topic.get(), 10);

        auto pland_controller = std::make_shared<PlandController>();
        ctx.land_detector = std::make_shared<LandingDetector>(
            GlobalConfig.GetConfig(), ctx.orientation, ctx.rangefinder_alt,
            ctx.pland_image, ctx.pland_image_stamp,
            [&ctx, pland_controller, image_pub_](const DetectorResult& result,
                                                 cv::Mat& detect) {
                sensor_msgs::ImagePtr msg =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8", detect)
                        .toImageMsg();
                image_pub_.publish(msg);
                pland_controller->control_step(ctx, result);
            });
    }

    template <typename RosAdapter>
    static void register_ros(std::shared_ptr<RosAdapter>& ros) {
        ros->bind_context(
            GlobalConfig.GetConfig().pland_image_topic,
            [](const sensor_msgs::ImageConstPtr& msg, RobotContext& ctx) {
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                        msg, sensor_msgs::image_encodings::BGR8);

                    ctx.pland_image.store(cv_ptr->image);
                    ctx.pland_image_stamp.emplace(msg->header.stamp);
                } catch (cv_bridge::Exception& e) {
                    spdlog::error("[Pland] convert image error: {}", e.what());
                }
            });
    }
};