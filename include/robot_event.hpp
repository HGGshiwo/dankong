#pragma once
#include <variant>

#include "./utils.hpp"
#include "dk/core.hpp"
#include "mavlink/imavlink.hpp"

// @JSON_ENABLE
struct EventResult {
    std::string status = "OK";
    std::string msg;
    std::string details;
};

// @JSON_ENABLE
struct TakeoffEvent : dk::AsyncEvent<EventResult> {
    double alt = 0;
};

// @JSON_ENABLE
struct PrearmEvent : dk::AsyncEvent<EventResult> {};

struct FlightModeEvent {
    FixedString64 prev;
    FixedString64 cur;
};

struct SysStatusEvent {
    uint32_t data;
};

struct StatusTextEvent {
    std::string text;
};

struct ArmEvent {
    bool armed = false;
};

struct TakeoffDoneEvent {};

using RobotEvent = std::variant<dk::TickEvent, dk::EnterEvent, dk::ExitEvent, TakeoffEvent, FlightModeEvent,
                                SysStatusEvent, StatusTextEvent, ArmEvent, TakeoffDoneEvent, PrearmEvent>;
