#pragma once

#include "core/event_result.hpp"
#include "dk/engine.hpp"

// @JSON_ENABLE
struct StartPlandDetectEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct SetPlandTarget : dk::AsyncEvent<EventResult> {};

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
