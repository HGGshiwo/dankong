#pragma once
#include "core/event_result.hpp"
#include "dk/engine.hpp"

//@JSON_ENABLE
struct LightEvent : dk::AsyncEvent<EventResult> {
    bool far = false;      // 远光
    bool dipped = false;   // 近光
    bool alarm = false;    // 双闪
    bool outline = false;  // 小灯
    int turn = 0;          // 转向
};

//@JSON_ENABLE
struct HornEvent : dk::AsyncEvent<EventResult> {
    bool horn = false;
};

//@JSON_ENABLE
struct EStopEvent : dk::AsyncEvent<EventResult> {
    bool set_estop = false;
};