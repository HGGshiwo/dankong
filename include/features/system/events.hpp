#pragma once
#include "core/event_result.hpp"
#include "dk/engine.hpp"

//@JSON_ENABLE
struct GetConfigEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct SetConfigEvent : dk::AsyncEvent<EventResult> {
    nlohmann::json config;
};

//@JSON_ENABLE
struct LogEvent : dk::AsyncEvent<EventResult> {
    std::string data;
};