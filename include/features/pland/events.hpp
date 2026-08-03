#pragma once
#include <optional>
#ifdef USE_ROS
#include "Eigen/Dense"
#include "core/event_result.hpp"
#include "dk/engine.hpp"

// @JSON_ENABLE
struct StartPlandDetectEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct SetPlandTarget : dk::AsyncEvent<EventResult> {
    std::optional<Eigen::Vector3d> position = std::nullopt;
    std::optional<Eigen::Vector3d> velocity = std::nullopt;
};

// @JSON_ENABLE
struct StartOffsetEstimate : dk::AsyncEvent<EventResult> {
    double x = 0;
    double y = 0;
    double z = 0;
};

// @JSON_ENABLE
struct StopOffsetEstimate : dk::AsyncEvent<EventResult> {
    bool save = false;
};
#endif