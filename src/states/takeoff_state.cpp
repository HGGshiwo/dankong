#include "states/takeoff_state.hpp"

#include <chrono>
#include <memory>

#include "states/hover_state.hpp"
#include "states/state_utils.hpp"

TakeoffState::TakeoffState(TakeoffEvent e)
    : event_(e), start_time_(std::chrono::steady_clock::now()), step_waypoint_(false), alt_(e.alt) {}

TakeoffState::TakeoffState(SetWaypointEvent e, state_utils::FinishAction action)
    : event_(e), start_time_(std::chrono::steady_clock::now()), step_waypoint_(true), action_(action) {
    alt_ = e.waypoint.at(0).z();
}

StateAction TakeoffState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    bool is_settled = std::visit([](const auto& obj) { return obj.is_settled(); }, event_);

    if (!is_settled && state_utils::get_time_span(start_time_) > PREARM_TIMEOUT) {
        // 超时了，直接拒绝
        std::visit([](const auto& obj) { return obj.reject("Takeoff Timeout!"); }, event_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}

void TakeoffState::PrearmCheckState::on_enter(RobotContext& ctx) {
    ctx.robot->set_mode("GUIDED");
    ctx.robot->run_prearm_checks();
}

StateAction TakeoffState::PrearmCheckState::on_event(const SysStatusEvent& event, RobotContext& ctx) {
    if (state_utils::check_sensor_health(event.data)) {
        return step<TakeoffState::ArmState>();
    }
    return StateAction::unhandled();
}

StateAction TakeoffState::PrearmCheckState::on_event(const StatusTextEvent& event, RobotContext& ctx) {
    if (state_utils::is_prearm_msg(event.text)) {
        std::visit([text = event.text](const auto& obj) { return obj.reject(text); }, parent()->event_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}

void TakeoffState::ArmState::on_enter(RobotContext& ctx) {
    ctx.robot->arm();
}

const double TAKEOFF_TIMEOUT = 100.0;

StateAction TakeoffState::ArmState::on_event(const ArmEvent& event, RobotContext& ctx) {
    if (event.armed) {
        return step<TakingoffState>();
    }
    return StateAction::unhandled();
}

void TakeoffState::TakingoffState::on_enter(RobotContext& ctx) {
    start_time_ = std::chrono::steady_clock::now();
    takeoff_res_ = ctx.robot->takeoff(parent()->alt_);
    checker_ = std::make_shared<state_utils::StallChecker<1>>(std::array<double, 1>{1.0}, TAKEOFF_TIMEOUT);

    if (takeoff_res_) {
        // 记录下起飞时候的经纬高
        auto pos = ctx.lon_lat_alt.get();
        ctx.takeoff_lon_lat_alt.set({pos.x(), pos.y(), parent()->alt_});
        std::visit([](const auto& obj) { return obj.resolve({"sucess", "OK"}); }, parent()->event_);
    }
}

StateAction TakeoffState::TakingoffState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    // 由于是on_enter执行的所以这里一定能访问到!
    if (!takeoff_res_) {
        std::visit(
            [this](const auto& obj) {
                if (!obj.is_settled()) obj.reject("takeoff cmd error");
            },
            parent()->event_);
        return step<GroundState>();
    }
    double alt = ctx.pos_enu.get().z();
    double tmp[1] = {alt};
    bool is_stall = checker_->is_stall(tmp);
    if (is_stall) {
        if (!ctx.robot->check_hover(ctx.arm, alt)) {
            return step<GroundState>();
        }
    };

    // 卡住强制进入下一个环节
    if (is_stall || state_utils::check_alt(ctx, parent()->alt_)) {
        if (!parent()->step_waypoint_) {
            return step<HoverState>();
        } else {
            auto event = std::get<SetWaypointEvent>(parent()->event_);
            return step<WaypointState::LiftingState>(std::tuple(event, parent()->action_), std::tuple<>());
        }
    };
    return StateAction::unhandled();
}