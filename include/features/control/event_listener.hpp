#pragma once

#include <array>
#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "./events.hpp"
#include "context_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"

inline const int FCU_DATA_RATE = 10;

// 和控制相关的事件监听器
class ControlEventListener
    : public dk::BaseEventListener<RobotContext, ControlEventListener> {
   public:
    using AllowedEvents =
        std::tuple<PrearmEvent, TakeoffEvent, dk::StateChangeEvent,
                   SetWaypointEvent, SetModeEvent, SetPosVelEvent,
                   dk::TickEvent, RebootFcuEvent, GetWpEvent, GetGpsEvent,
                   GetParamEvent, SetParamEvent, FcuConnectedEvent, DisarmEvent,
                   RestartEvent>;

    void on_event(dk::TickEvent event, RobotContext& ctx);
    void on_event(SetPosVelEvent event, RobotContext& ctx);
    void on_event(PrearmEvent event, RobotContext& ctx);
    void on_event(TakeoffEvent event, RobotContext& ctx);
    void on_event(dk::StateChangeEvent event, RobotContext& ctx);
    void on_event(SetWaypointEvent event, RobotContext& ctx);
    void on_event(const SetModeEvent& event, RobotContext& ctx);
    void on_event(const RebootFcuEvent& event, RobotContext& ctx);
    void on_event(const GetWpEvent& event, RobotContext& ctx);
    void on_event(const GetGpsEvent& event, RobotContext& ctx);
    void on_event(const FcuConnectedEvent& event, RobotContext& ctx);
    void on_event(const GetParamEvent& event, RobotContext& ctx);
    void on_event(const SetParamEvent& event, RobotContext& ctx);
    void on_event(const DisarmEvent& event, RobotContext& ctx);
    void on_event(const RestartEvent& event, RobotContext& ctx);
};
