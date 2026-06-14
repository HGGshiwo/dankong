#pragma once
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <memory>
#include <optional>

#include "./landing_controller.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "event_listener.hpp"
#include "features/algo/events.hpp"
#include "features/tracker/tracker.hpp"
#include "landing_detector.hpp"
#include "nav_msgs/Odometry.h"
#include "robot_context.hpp"
#include "ros/message_traits.h"
#include "ros/publisher.h"
#include "ros/time.h"
#include "spdlog/spdlog.h"
#include "std_msgs/Float64.h"

class PlandFeature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        ros::NodeHandle nh;
        ros::Publisher image_pub_ = nh.advertise<sensor_msgs::Image>(
            GlobalConfig.GetConfig().pland_detect_topic.get(), 10);

        ctx.land_controller = std::make_shared<PlandController>(ctx);

        ctx.land_detector = std::make_shared<LandingDetector>(
            GlobalConfig.GetConfig(), ctx,
            [&ctx, image_pub_](const DetectorResult& result, cv::Mat& detect) {
                if (!detect.empty()) {  // 可能没有被检测更新(比如时间戳相同)
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();
                    sensor_msgs::ImagePtr msg =
                        cv_bridge::CvImage(header, "bgr8", detect).toImageMsg();
                    image_pub_.publish(msg);
                }

                // 只更新观测状态，不执行控制！
                ctx.land_controller->update_observation(result);
            });
    }

    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        ros->bind_context(
            GlobalConfig.GetConfig().pland_image_topic.get(),
            [](const sensor_msgs::ImageConstPtr& msg, RobotContext& ctx) {
                try {
                    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                        msg, sensor_msgs::image_encodings::BGR8);

                    ctx.pland_image.store(cv_ptr->image);
                    ctx.pland_image_stamp.store(msg->header.stamp.toSec());
                } catch (cv_bridge::Exception& e) {
                    spdlog::error("[Pland] convert image error: {}", e.what());
                }
            });

        ros->bind_context(
            GlobalConfig.GetConfig().gimbal_roll_topic,
            [](const std_msgs::Float64::ConstPtr& msg, RobotContext& ctx) {
                double data = msg->data * (M_PI / 180.0);
                ctx.gimbal_roll.store(data);
                ctx.pose_history.push_gimbal_roll(
                    ctx.engine->get_time_provider()->now(), data);
            });
        ros->bind_context(
            GlobalConfig.GetConfig().gimbal_pitch_topic,
            [](const std_msgs::Float64::ConstPtr& msg, RobotContext& ctx) {
                double data = msg->data * (M_PI / 180.0);
                ctx.gimbal_pitch.store(data);
                ctx.pose_history.push_gimbal_pitch(
                    ctx.engine->get_time_provider()->now(), data);
            });
        ros->bind_context(
            GlobalConfig.GetConfig().gimbal_yaw_topic,
            [](const std_msgs::Float64::ConstPtr& msg, RobotContext& ctx) {
                double data = msg->data * (M_PI / 180.0);
                ctx.gimbal_yaw.store(data);
                ctx.pose_history.push_gimbal_yaw(
                    ctx.engine->get_time_provider()->now(), data);
            });
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        web->template register_route<StartPlandDetectEvent, EventResult>(
            boost::beast::http::verb::post, "/start_pland_detect");
        web->template register_route<SetPlandTarget, EventResult>(
            boost::beast::http ::verb::post, "/pland_target/set");
        // 该接口只是调试时候使用
        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/pland", 5000,
            [](SetWaypointEvent& event) {
                event.land_target_id = 0;
                event.finish_action = FinishAction::LAND;
            });
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<PlandEventListener>();
        engine->add_listener(listener);
    }
};
