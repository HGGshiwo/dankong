#pragma once

#include "./context.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/udp/udp.hpp"
// 确保包含你的 mavsdk_adapter 头文件
#include "dk/adapters/mavsdk.hpp"
#include "robot/drone.hpp"
#include "robot_context.hpp"

struct DroneFeature {
    static void setup(TagInit, RobotContext& ctx) {
        // 使用 MAVSDK 实例初始化无人机控制端
        ctx.robot = std::make_shared<Drone>(std::make_shared<MavsdkDrone>(
            ctx.engine->get_context().mavsdk_system));
    }

    // 新增 MAVSDK 的数据绑定
    static void setup(
        TagMavsdk,
        std::shared_ptr<dk::MavsdkAdapter<RobotContext, Engine>>& adapter) {
        // 1. 替代 /mavros/distance_sensor/rangefinder_pub
        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_distance_sensor,
            [](mavsdk::Telemetry::DistanceSensor dist,
               RobotContext& ctx) -> void {
                // 判断数据是否有效
                if (dist.current_distance_m >= dist.minimum_distance_m &&
                    dist.current_distance_m <= dist.maximum_distance_m)
                    ctx.rangefinder_alt = dist.current_distance_m;
            });

        // 2. 替代 /mavros/vfr_hud 中的 throttle
        adapter->bind_passthrough_context(
            74,  // MAVLINK_MSG_ID_VFR_HUD
            [](const mavlink_message_t& msg, RobotContext& ctx) {
                mavlink_vfr_hud_t hud;
                mavlink_msg_vfr_hud_decode(&msg, &hud);
                ctx.throttle = (double)hud.throttle / 100;  // uint16_t (0-100)
            });
    }
};