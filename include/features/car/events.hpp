#pragma once
#include "core/event_result.hpp"
#include "dk/engine.hpp"

//@JSON_ENABLE
struct LightEvent : dk::AsyncEvent<EventResult> {
    bool far = false;
};

//@JSON_ENABLE
struct HornEvent : dk::AsyncEvent<EventResult> {
    bool horn = false;
};

//@JSON_ENABLE
struct EStopEvent : dk::AsyncEvent<EventResult> {
    bool set_estop = false;
};