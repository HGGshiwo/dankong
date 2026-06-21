#pragma once

#include <array>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"

// 和控制相关的事件监听器
class NtripEventListener
    : public dk::BaseEventListener<RobotContext, NtripEventListener> {
    RateLimiter rate_{0.1};

   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    void on_event(dk::TickEvent event, RobotContext& ctx) {
        if (!ctx.ntrip_client->is_running()) {
            auto config = GlobalConfig.GetConfig();
            if (!config.ntrip_enable.get() || ctx.gps_fix_type.load() < 3 ||
                ctx.gps_nsats.load() < config.ntrip_nstats.get()) {
                return;
            }
            spdlog::info("GPS ready. Starting NTRIP...");

            auto lon_lat_alt = ctx.lon_lat_alt.load();
            // 启动 NTRIP！
            ctx.ntrip_client->run(
                config.ntrip_ip.get(),
                config.ntrip_port.get(),  // 千寻的 IP 和端口
                config.ntrip_username.get(), config.ntrip_password.get(),
                config.ntrip_mountpoint.get(),     // 挂载点
                lon_lat_alt.y(), lon_lat_alt.x(),  // 准的初始坐标！
                [robot = ctx.robot](const uint8_t* data,
                                    size_t size) {  // 回调函数
                    robot->send_rtcm_data(data, size);
                });
        } else if (rate_.check_and_update(
                       ctx.engine->get_time_provider()->now())) {
            // NTRIP 已经在运行了，CORS/千寻 要求我们大约每 10~15 秒更新一次坐标
            auto lon_lat_alt = ctx.lon_lat_alt.load();
            ctx.ntrip_client->send_location_update(lon_lat_alt.y(),
                                                   lon_lat_alt.x());
        }
    }
};
