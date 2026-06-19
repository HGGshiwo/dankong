#pragma once
#include <atomic>
#include <memory>
#include <string_view>

#include "dk/adapters/udp/udp_client.hpp"
#include "mavlink/mavsdk_drone.hpp"
#include "robot_context.hpp"

struct DroneContext {
    std::shared_ptr<IRobot> robot;

    // =========================================================================
    // 纯净的数据载体 (Data Model)
    // =========================================================================
    std::atomic<double> throttle{-1.0};
    std::atomic<double> rangefinder_alt{-1.0};

   public:
    // =========================================================================
    // 外部上报绑定 (Telemetry Binding)
    // =========================================================================
    explicit DroneContext(StateRegistry& reg) {
        // 构造函数保留 reg 传参，保持与系统其他 Context 签名一致。
        // （注：目前 throttle 和 rangefinder_alt 是
        // std::atomic，所以没有注册上报。
        //  如果后续需要将它们自动上报，只需将其改为
        //  DirtyVar<double>，然后在此处调用 reg.bind 即可）
    }

    static constexpr std::string_view ROBOT_TYPE = "DRONE";
};