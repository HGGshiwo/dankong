#pragma once
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "./robot_context.hpp"
#include "./robot_event.hpp"
#include "./states/init_state.hpp"
#include "./states/state_utils.hpp"
#include "./states/takeoff_state.hpp"
#include "dk/engine.hpp"
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
        if (active_states.size() == 0) {
            throw std::runtime_error("current active_states is empty!");
        }
        auto state = active_states.at(0);
        if (state_utils::is_state<InitState>(state)) {
            event.reject(state->name() + "无法起飞!");
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