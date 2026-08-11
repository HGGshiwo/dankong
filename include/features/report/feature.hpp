#pragma once
#include <memory>
#include <optional>

#include "./event_listener.hpp"
#include "core/empty_context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/mqtt.hpp"
#include "robot_context.hpp"
#if USE_ROS
#include "dk/adapters/ros.hpp"
#endif
// 数据上报
class ReportFeature {
   public:
    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<ReportEventListener>(
            engine->get_context(), engine->get_context().ws_manager);
        engine->add_listener(listener);
    }

    static void register_topic(
        RobotContext& ctx,
        std::shared_ptr<dk::MqttClientAdapter<RobotContext, Engine>> adapter) {
        auto deviceCode = GlobalConfig.GetConfig().device_code.get();
        if (!deviceCode.has_value()) return;
        ctx.mqtt_client->register_publish_handler<TaskEvent>(
            fmt::format("device/{}/task", deviceCode.value()),
            [&ctx, adapter](const TaskEvent& event) {
                // 没有加锁可能不够安全
                SetWaypointEvent wp_event;
                if (event.mapCode.has_value()) {
                    ctx.mapCode = event.mapCode;
                    wp_event.local = true;
                } else {
                    ctx.mapCode = std::nullopt;
                }
                ctx.serialNo = event.serialNo;
                ctx.taskId = event.taskId;
                wp_event.nodeEventList = event.pathEventInfo;
                wp_event.waypoint = event.pathInfo;

                auto res = adapter->dispatch_async(wp_event);
                std::move(res).catch_error([](const std::exception_ptr& ex) {
                    spdlog::error("[Task] reject: {}", get_error_message(ex));
                });
            },
            1);

        ctx.mqtt_client->register_publish_handler<VideoEvent>(
            fmt::format("device/{}/video", deviceCode.value()),
            [](const VideoEvent&) {

            },
            1);

        ctx.mqtt_client->register_publish_handler<SpeakEvent>(
            fmt::format("device/{}/speak", deviceCode.value()),
            [](const SpeakEvent&) {

            },
            1);
    }

    static void setup(
        TagMqtt, RobotContext& ctx,
        std::shared_ptr<dk::MqttClientAdapter<RobotContext, Engine>>& mqtt) {
        auto device_code = GlobalConfig.GetConfig().device_code.get();

        if (device_code.has_value()) {
            spdlog::info("[Mqtt] use existing code: {}", device_code.value());
            mqtt->connect(device_code.value());
            register_topic(ctx, mqtt);
        } else {
            mqtt->register_publish_handler<RegisterEvent>(
                "$exclusive/register",
                [&ctx, mqtt](const RegisterEvent& data) -> void {
                    auto& config = GlobalConfig.GetConfig();
                    if (!config.device_code.get().has_value()) {
                        std::string device_code = data.deviceCode;
                        config.device_code.set(
                            std::make_optional<std::string>(device_code));
                        GlobalConfig.save();
                        mqtt->connect(device_code);
                        register_topic(ctx, mqtt);
                        spdlog::info("[Mqtt] register with code: {}",
                                     device_code);
                        // 第一次注册时候发送全量数据
                        ReportEventListener::publish_full_state(ctx);
                    }
                });

            mqtt->connect();  // 先注册
        }
    }

#if USE_ROS1
    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        ros->bind_event(
            "/abnormal",
            [](const std_msgs::String::ConstPtr& msg) -> AbnormalEvent {
                AbnormalEvent e;
                e.data = msg->data;
                return e;
            });
    }
#endif
};