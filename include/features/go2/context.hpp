#pragma once
#include <atomic>
#include <memory>
#include <string_view>

#include "mavlink/mavsdk_drone.hpp"
#include "robot/irobot.hpp"
#include "utils/state_registry.hpp"

struct Go2Context {
    std::shared_ptr<IRobot> robot;

   public:
    explicit Go2Context(StateRegistry& reg) {}

    static constexpr std::string_view ROBOT_TYPE = "GO2";
};