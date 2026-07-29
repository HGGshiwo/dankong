#include "states/takeoff_state.hpp"

#include <chrono>
#include <memory>

#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

TakeoffState::TakeoffState(TakeoffEvent e)
    : event_(e), alt_(e.alt), step_waypoint_(false) {}

TakeoffState::TakeoffState(SetWaypointEvent e)
    : event_(e), step_waypoint_(true) {
    alt_ = e.waypoint.at(0).z();
}

StateAction TakeoffState::on_enter(RobotContext& ctx) {
    start_time_ = ctx.engine->get_time_provider()->now();
    return StateAction::unhandled();
}

StateAction TakeoffState::on_tick(double dt, RobotContext& ctx) {
    bool is_settled =
        std::visit([](const auto& obj) { return obj.is_settled(); }, event_);

    double now = ctx.engine->get_time_provider()->now();
    if (!is_settled && state_utils::get_time_span(start_time_, now) >
                           GlobalConfig.GetConfig().prearm_timeout) {
        // 超时了，直接拒绝
        std::visit(
            [](const auto& obj) { return obj.reject("Takeoff Timeout!"); },
            event_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::PrearmCheckState::on_enter(RobotContext& ctx) {
    if (state_utils::should_do_prearm_check(ctx.robot)) {
        ctx.robot->run_prearm_checks();
        return StateAction::unhandled();
    } else {
        return StateAction::step<TakeoffState::ArmState>();
    }
}

StateAction TakeoffState::PrearmCheckState::on_event(
    const SysStatusEvent& event, RobotContext& ctx) {
    if (ctx.robot->check_sensor_health(event.data)) {
        return step<TakeoffState::ArmState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::PrearmCheckState::on_event(
    const StatusTextEvent& event, RobotContext& ctx) {
    if (ctx.robot->is_prearm_msg(event.text)) {
        std::visit(
            [text = event.text](const auto& obj) { return obj.reject(text); },
            parent()->event_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::ArmState::on_enter(RobotContext& ctx) {
    if (ctx.arm.load()) {
        return StateAction::step<TakingoffState>();
    } else {
        ctx.robot->arm();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::ArmState::on_event(const ArmEvent& event,
                                             RobotContext& ctx) {
    if (event.armed) {
        return step<TakingoffState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::ArmState::on_event(const StatusTextEvent& event,
                                             RobotContext& ctx) {
    if (ctx.robot->is_prearm_msg(event.text)) {
        std::visit(
            [text = event.text](const auto& obj) { return obj.reject(text); },
            parent()->event_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::TakingoffState::on_enter(RobotContext& ctx) {
    auto& cfg = GlobalConfig.GetConfig();
    double now = ctx.engine->get_time_provider()->now();
    start_time_ = now;

    checker_ = std::make_shared<state_utils::StallChecker<1>>(
        std::array<double, 1>{1.0}, cfg.takeoff_timeout, now);

    if (ctx.robot->takeoff(parent()->alt_)) {
        // 记录下起飞时候的经纬高
        auto pos = ctx.lon_lat_alt.load();
        ctx.takeoff_lon_lat_alt.emplace(pos.x(), pos.y(), parent()->alt_);
        std::visit(
            [](const auto& obj) { return obj.resolve({"success", "OK"}); },
            parent()->event_);
        return StateAction::unhandled();
    } else {
        std::visit(
            [this](const auto& obj) {
                if (!obj.is_settled()) obj.reject("takeoff cmd error");
            },
            parent()->event_);
        return StateAction::step<GroundState>();
    }
}

StateAction TakeoffState::TakingoffState::on_tick(double dt,
                                                  RobotContext& ctx) {
    auto& cfg = GlobalConfig.GetConfig();
    double alt = ctx.pos_enu.load().z();
    double now = ctx.engine->get_time_provider()->now();
    bool is_stall = checker_->is_stall({alt}, now);
    if (is_stall) {
        if (!ctx.robot->check_hover(ctx)) {
            return step<GroundState>();
        }
    };

    // 如果支持高度的，要求等待高度达到指定值
    // 如果不支持高度，只需要在空中即可
    bool a = ctx.robot->check_hover(ctx);
    bool arrive = ctx.robot->is_alt_enable()
                      ? std::fabs(ctx.pos_enu.load().z() - parent()->alt_) <
                            cfg.z_tolerance
                      : a;
    if (!arrive && !is_stall) {
        return StateAction::unhandled();
    }
    // 卡住强制进入下一个环节
    if (is_stall && !ctx.robot->check_hover(ctx)) {
        return step<GroundState>();
    }
    // 如果到了，或者卡住+空中，则执行航线
    if (!parent()->step_waypoint_) {
        parent()->report_takeoff(ctx);
        return step<HoverState>();
    } else {
        auto event = std::get<SetWaypointEvent>(parent()->event_);
        parent()->report_takeoff(ctx);
        return step<WaypointState::LiftingState>(std::tuple(event),
                                                 std::tuple<>());
    }
}