#pragma once
#include <atomic>
#include <cstdint>
#include <memory>
#include <string_view>

#include "mavlink/mavsdk_drone.hpp"
#include "robot/irobot.hpp"
#include "utils/state_registry.hpp"

struct MavrosBridge;  // features/control/mavros_bridge.hpp, 由 Go2Feature 创建

struct Go2Context {
    std::shared_ptr<IRobot> robot;
    std::shared_ptr<MavrosBridge> mavros_bridge;
    std::atomic<uint8_t> go2_state;
    // 定位输出开关 (/loc_base 发布 + odom->base_link TF), HTTP 控制, 默认关闭
    std::atomic<bool> loc_output_enable{false};

   public:
    explicit Go2Context(StateRegistry& reg) {}

    static constexpr std::string_view ROBOT_TYPE = "GO2";
};