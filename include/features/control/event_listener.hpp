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

inline std::string get_error_message(std::exception_ptr eptr) {
    if (!eptr) {
        return "No exception";  // 空指针处理
    }

    try {
        // 将指针中的异常重新抛出
        std::rethrow_exception(eptr);
    } catch (const std::exception& e) {
        // 捕获所有继承自 std::exception 的标准异常
        return e.what();
    } catch (const std::string& s) {
        // 捕获抛出类型为 std::string 的异常
        return s;
    } catch (const char* c) {
        // 捕获抛出类型为 C风格字符串 的异常
        return c;
    } catch (...) {
        // 兜底：捕获未知类型（如 throw 1;）
        return "Unknown exception type";
    }
}

// 和控制相关的事件监听器
class ControlEventListener
    : public dk::BaseEventListener<RobotContext, ControlEventListener> {
    RateLimiter rate_{1};

   public:
    using AllowedEvents =
        std::tuple<PrearmEvent, TakeoffEvent, dk::StateChangeEvent,
                   SetWaypointEvent, SetModeEvent, SetPosVelEvent,
                   dk::TickEvent, RebootFcuEvent, GetWpEvent, GetGpsEvent,
                   GetParamEvent, SetParamEvent, DisarmEvent, RestartEvent,
                   JoystickEvent, EnableJoystickEvent, TestEvent>;

    void on_event(const dk::TickEvent& event, RobotContext& ctx);
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

    void on_event(const TestEvent& event, RobotContext& ctx) {
        // ctx.engine->wait_for(1000, [event](const dk::TickEvent& e) {
        //     event.resolve({"success", "OK"});
        //     return true;
        // });
    }
};
