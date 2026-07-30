#pragma once

#include <array>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>

#include "./events.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"
#include "utils/logger/fglog.hpp"

// 和控制相关的事件监听器
class ControlEventListener
    : public dk::BaseEventListener<RobotContext, ControlEventListener> {
   public:
    using AllowedEvents = std::tuple<
        PrearmEvent, TakeoffEvent, dk::StateChangeEvent, SetWaypointEvent,
        SetModeEvent, SetPosVelEvent, RebootFcuEvent, GetWpEvent, GetGpsEvent,
        GetParamEvent, SetParamEvent, DisarmEvent, RestartEvent, JoystickEvent,
        EnableJoystickEvent, TestEvent, ReportEvent, FlightModeEvent>;

    void on_tick(double dt, RobotContext& ctx);
    void on_event(const SetPosVelEvent& event, RobotContext& ctx);
    void on_event(const PrearmEvent& event, RobotContext& ctx);
    void on_event(const TakeoffEvent& event, RobotContext& ctx);
    void on_event(const dk::StateChangeEvent& event, RobotContext& ctx);
    void on_event(const SetWaypointEvent& event, RobotContext& ctx);
    void on_event(const SetModeEvent& event, RobotContext& ctx);
    void on_event(const RebootFcuEvent& event, RobotContext& ctx);
    void on_event(const GetWpEvent& event, RobotContext& ctx);
    void on_event(const GetGpsEvent& event, RobotContext& ctx);
    void on_event(const GetParamEvent& event, RobotContext& ctx);
    void on_event(const SetParamEvent& event, RobotContext& ctx);
    void on_event(const DisarmEvent& event, RobotContext& ctx);
    void on_event(const RestartEvent& event, RobotContext& ctx);
    void on_event(const JoystickEvent& event, RobotContext& ctx);
    void on_event(const EnableJoystickEvent& event, RobotContext& ctx) {
        ctx.enable_joystick.store(event.enable);
        event.resolve({"success", "OK"});
    }

    void on_event(const TestEvent& event, RobotContext& ctx) {}

    void on_event(const ReportEvent& event, RobotContext& ctx) {
        ctx.ws_manager->publish_reliable(nlohmann::json::parse(event.data));
        spdlog::info("[WS] publish: {}", event.data);
        fglog::log("/drone/ws", event.data);
    }

    void on_event(const FlightModeEvent& event, RobotContext& ctx) {
        spdlog::info("Flight mode change: {} -> {}",
                     std::string(event.prev.mode_str),
                     std::string(event.cur.mode_str));
        fglog::publish_state("drone/flight_mode",
                             std::string(event.cur.mode_str));
    }
};
