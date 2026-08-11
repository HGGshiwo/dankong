#pragma once
#include "core/global_config.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"
#include "state_common.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"

class ArmState : public dk::BaseState<RobotContext, ArmState, void> {
   private:
    double start_time_{0.0};

   public:
    using AllowedEvents = std::tuple<ArmEvent, StatusTextEvent>;

    ArmState() = default;

    dk::StateAction<RobotContext> on_enter(RobotContext& ctx) override {
        start_time_ = ctx.engine->get_time_provider()->now();
        if (ctx.arm.load()) {
            return StateAction::next();
        }
        ctx.robot->arm();
        return dk::StateAction<RobotContext>::unhandled();
    }

    dk::StateAction<RobotContext> on_tick(double dt,
                                          RobotContext& ctx) override {
        double now = ctx.engine->get_time_provider()->now();
        if (state_utils::get_time_span(start_time_, now) >
            GlobalConfig.GetConfig().prearm_timeout) {
            spdlog::error("[ArmState] Arming timeout!");
            if (ctx.robot->check_hover(ctx)) {
                LOG_STATE_STEP("HoverState");
                return this->step<HoverState>();
            } else {
                LOG_STATE_STEP("GroundState");
                return this->step<GroundState>();
            }
        }
        return dk::StateAction<RobotContext>::unhandled();
    }

    dk::StateAction<RobotContext> on_event(const ArmEvent& event,
                                           RobotContext& ctx) {
        if (event.armed) {
            return StateAction::next();
        }
        return dk::StateAction<RobotContext>::unhandled();
    }

    dk::StateAction<RobotContext> on_event(const StatusTextEvent& event,
                                           RobotContext& ctx) {
        if (ctx.robot->is_prearm_msg(event.text)) {
            spdlog::warn("[ArmState] Prearm check failed: {}", event.text);
            if (ctx.robot->check_hover(ctx)) {
                LOG_STATE_STEP("HoverState");
                return this->step<HoverState>();
            } else {
                LOG_STATE_STEP("GroundState");
                return this->step<GroundState>();
            }
        }
        return dk::StateAction<RobotContext>::unhandled();
    }

    static constexpr std::string_view static_name() { return "解锁状态"; }
};