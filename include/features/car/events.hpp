#pragma once
#include "core/event_result.hpp"
#include "dk/engine.hpp"

//@JSON_ENABLE
struct LightEvent : dk::AsyncEvent<EventResult> {
    int turn = 0;  // 0归位, 1左转, 2右转
    bool dipped = false;
    bool far = false;
    bool outline = false;
    bool alarm = false;
    bool horn = false;
};