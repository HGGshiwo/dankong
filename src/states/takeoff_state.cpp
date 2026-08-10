#include "states/takeoff_state.hpp"

#include <chrono>
#include <memory>

#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "states/arm_state.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

TakeoffState::TakeoffState(TakeoffEvent e) : event_(e), alt_(e.alt) {}

TakeoffState::TakeoffState(double alt) : event_(std::nullopt), alt_(alt) {}

StateAction TakeoffState::before_enter(RobotContext& ctx,
                                       const TakeoffEvent& e) {
    if (!ctx.arm.load()) {
        StateFlags flags{.is_takeoff = true};
        if (ctx.robot->should_arm_before_enter(flags)) {
            return StateAction::plan([](auto& plan, auto& /*ctx*/) {
                plan.template push_front<ArmState>();
            });
        }
    }
    return StateAction::handled();
}

StateAction TakeoffState::before_enter(RobotContext& ctx, double alt) {
    if (!ctx.arm.load()) {
        StateFlags flags{.is_takeoff = true};
        if (ctx.robot->should_arm_before_enter(flags)) {
            return StateAction::plan([](auto& plan, auto& /*ctx*/) {
                plan.template push_front<ArmState>();
            });
        }
    }
    return StateAction::handled();
}

StateAction TakeoffState::on_enter(RobotContext& ctx) {
    auto& cfg = GlobalConfig.GetConfig();
    double now = ctx.engine->get_time_provider()->now();
    start_time_ = now;

    checker_ = std::make_shared<state_utils::StallChecker<1>>(
        std::array<double, 1>{1.0}, cfg.takeoff_timeout, now);

    if (ctx.robot->takeoff(alt_)) {
        auto pos_enu = ctx.pos_enu.load();
        ctx.takeoff_enu.emplace(
            Eigen::Vector3d{pos_enu.x(), pos_enu.y(), alt_});
        if (event_.has_value()) {
            event_->resolve({"success", "OK"});
        }
        return StateAction::unhandled();
    } else {
        if (event_.has_value() && !event_->is_settled()) {
            event_->reject("takeoff cmd error");
        }
        if (ctx.robot->check_hover(ctx)) {
            LOG_STATE_STEP("HoverState");
            return StateAction::step<HoverState>();
        } else {
            LOG_STATE_STEP("GroundState");
            return StateAction::step<GroundState>();
        }
    }
}

StateAction TakeoffState::on_tick(double dt, RobotContext& ctx) {
    bool is_settled = !event_.has_value() || event_->is_settled();

    double now = ctx.engine->get_time_provider()->now();
    if (!is_settled && state_utils::get_time_span(start_time_, now) >
                           GlobalConfig.GetConfig().prearm_timeout) {
        if (event_.has_value()) {
            event_->reject("Takeoff Timeout!");
        }
        if (ctx.robot->check_hover(ctx)) {
            LOG_STATE_STEP("HoverState");
            return step<HoverState>();
        } else {
            LOG_STATE_STEP("GroundState");
            return step<GroundState>();
        }
    }

    auto& cfg = GlobalConfig.GetConfig();
    double alt = ctx.pos_enu.load().z();
    bool is_stall = checker_->is_stall({alt}, now);
    if (is_stall) {
        if (!ctx.robot->check_hover(ctx)) {
            LOG_STATE_STEP("GroundState");
            return step<GroundState>();
        }
    }

    bool a = ctx.robot->check_hover(ctx);
    bool arrive =
        ctx.robot->is_alt_enable()
            ? std::fabs(ctx.pos_enu.load().z() - alt_) < cfg.z_tolerance
            : a;
    if (!arrive && !is_stall) {
        return StateAction::unhandled();
    }

    if (is_stall && !ctx.robot->check_hover(ctx)) {
        LOG_STATE_STEP("GroundState");
        return step<GroundState>();
    }

    report_takeoff(ctx);
    return StateAction::next();
}