#pragma once
#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "./robot_context.hpp"
#include "./robot_event.hpp"
#include "./states/init_state.hpp"
#include "./states/state_utils.hpp"
#include "./states/takeoff_state.hpp"
#include "dk/adapters/web/adapter.hpp">
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "states/hover_state.hpp"
#include "states/waypoint_state.hpp"

class EventListener : public dk::BaseEventListener<RobotContext, EventListener> {
   public:
    using AllowedEvents = std::tuple<PrearmEvent, TakeoffEvent, dk::StateChangeEvent, SetWaypointEvent>;

    void on_event(PrearmEvent event, RobotContext& ctx) {
        state_utils::prearm_check(ctx)
            .then([event](bool res) {
                if (res) event.resolve({"success", "OK"});
                event.resolve({"error", "Unknown Error"});
            })
            .catch_error([event](std::exception_ptr exp) {
                try {
                    std::rethrow_exception(exp);
                } catch (const std::exception& e) {
                    event.resolve({"error", e.what()});
                }
            });
    }

    void on_event(TakeoffEvent event, RobotContext& ctx) {
        auto active_states = ctx.engine->get_active_states();
        if (active_states.empty()) {
            throw std::runtime_error("current active_states is empty!");
        }
        auto state = active_states.at(0);
        if (state_utils::is_state<InitState>(state)) {
            event.reject(state->name() + "无法起飞!");
            return;
        }
        if (!state_utils::is_state<GroundState>(state)) {
            event.reject("只允许在地面状态起飞!");
            return;
        }
        ctx.engine->step<TakeoffState::PrearmCheckState>(std::tuple(std::move(event)), std::tuple<>());
    }

    void on_event(dk::StateChangeEvent event, RobotContext& ctx) {
        spdlog::info("State change {} -> {}", event.prev, event.cur);
    }

    void on_event(SetWaypointEvent event, RobotContext& ctx) {
        auto state = ctx.engine->get_active_states().at(0);
        if (state_utils::is_state<InitState>(state)) {
            event.reject(state->name() + "无法起飞!");
            return;
        }
        if (state_utils::is_state<GroundState>(state)) {
            ctx.engine->step<TakeoffState::PrearmCheckState>(
                std::make_tuple(std::move(event), state_utils::FinishAction::HOVER), std::tuple<>());
        } else {
            event.resolve({"success", "OK"});
            ctx.engine->step<WaypointState::LiftingState>(
                std::make_tuple(std::move(event), state_utils::FinishAction::HOVER), std::tuple<>());
        }
    }
};

const double REPORT_HZ = 10.0;
const double REPORT_MISSION_HZ = 1.0;

class StateReportListener : public dk::BaseEventListener<RobotContext, StateReportListener> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent, dk::WsOpenEvent>;

    std::shared_ptr<dk::ConnectionManager> manager_;

    StateReportListener(std::shared_ptr<dk::ConnectionManager> manager) : manager_(manager) {}

    // 当有新客户端接入时的处理逻辑：
    void on_event(const dk::WsOpenEvent& event, RobotContext& ctx) {
        // 1. 无视脏位，一键提取所有变量当前的最新快照！
        nlohmann::json full_state = ctx.state_registry.get_full_state();

        // 2. 补齐静态/基础字段
        full_state["type"] = "state";
        if (ctx.engine) {
            full_state["state"] = ctx.engine->get_state_name();
        }

        // 3. 仅发给这一个新连进来的客户端
        event.conn->send(full_state);
    }

    void on_event(const dk::TickEvent& event, RobotContext& ctx) {
        nlohmann::json j;
        ctx.state_registry.report_all(j);
        // 如果没有任何数据改变或达到频率条件，j 将是空的，直接 return
        if (j.empty()) {
            return;
        }
        // 补充动态的基本状态
        j["type"] = "state";
        if (ctx.engine) {
            j["state"] = ctx.engine->get_state_name();
        }
        manager_->publish(j);
    }
};