#pragma once
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "./event_listener.hpp"
#include "./events.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/control/events.hpp"
#include "features/control/mavros_bridge.hpp"
#include "robot/go2.hpp"
#include "robot_context.hpp"

class Go2Feature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        auto mavlink = std::make_shared<MavsdkDrone>(
            ctx.engine->get_context().mavsdk_system);

        ctx.robot =
            std::make_shared<Go2>(std::make_shared<MavsdkDrone>(
                                      ctx.engine->get_context().mavsdk_system),
                                  ctx);
        // 位姿由 MAVLink 写入 Context，再经 Go2EventListener 定频发布到
        // /loc_base
        ctx.use_fcu_enu.store(true);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        engine->add_listener(std::make_shared<Go2EventListener>());
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        // 定位输出开关: 同时控制 /loc_base 发布与 odom->base_link TF
        web->template register_route<ToggleLocOutputEvent, EventResult>(
            boost::beast::http::verb::post, "/loc_output/enable", 5000);
    }

    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
#ifdef USE_ROS1
        // 启动 mavros 兼容桥 (默认开启, 通过 ~enable_mavros_bridge 参数关闭),
        // 对外提供 /mavros/* 风格话题与服务
        bool enable_bridge = true;
        ros::NodeHandle pnh("~");
        pnh.param("enable_mavros_bridge", enable_bridge, true);
        if (enable_bridge) {
            auto& ctx = ros->get_engine()->get_context();
            ctx.mavros_bridge = std::make_shared<MavrosBridge>(ctx);
        }
#endif

        ros->bind_event(
            "/task/feedback",
            [](const std_msgs::String::ConstPtr& msg) -> WpArriveEvent {
                return WpArriveEvent{};
            });

        ros->bind_json_context(
            "/dank/status",
            [](const Go2StateData& msg, RobotContext& ctx) -> void {
                // ctx.arm 以飞控 MAVLink 遥测 (subscribe_armed) 为唯一来源,
                // 此处不再用 go2 工作模式改写 arm, 否则两者互相覆盖导致
                // /mavros/state 的 armed 来回抖动
                ctx.go2_state.store(msg.mode);
                ctx.battery_remaining.store(msg.battery);
            });
    };
};