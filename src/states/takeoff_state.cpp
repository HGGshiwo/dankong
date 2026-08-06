#include "states/takeoff_state.hpp"

#include <chrono>
#include <memory>

#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "robot_context.hpp"
#include "states/arm_state.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"
#include "states/waypoint_state.hpp"

TakeoffState::TakeoffState(TakeoffEvent e)
    : event_(e), alt_(e.alt), step_waypoint_(false) {}

TakeoffState::TakeoffState(SetWaypointEvent e)
    : event_(e), step_waypoint_(true) {
    alt_ = e.waypoint.at(0).z();
}

StateAction TakeoffState::on_enter(RobotContext& ctx) {
    // 1. 拦截检查：若未解锁且机器人需要解锁，强制切入 ArmState，并在成功后重回
    // TakeoffState
    if (!ctx.arm.load()) {
        auto flags = get_state_flags(ctx);
        if (ctx.robot->should_arm_before_enter(flags)) {
            auto resume_action = std::visit(
                [this](const auto& ev) {
                    return StateAction::step<TakeoffState>(std::make_tuple(ev));
                },
                event_);
            return StateAction::step<ArmState>(std::tuple(resume_action));
        }
    }

    // 2. 原 TakingoffState::on_enter 逻辑
    auto& cfg = GlobalConfig.GetConfig();
    double now = ctx.engine->get_time_provider()->now();
    start_time_ = now;

    checker_ = std::make_shared<state_utils::StallChecker<1>>(
        std::array<double, 1>{1.0}, cfg.takeoff_timeout, now);

    if (ctx.robot->takeoff(alt_)) {
        // 记录起飞时的经纬高
        auto pos = ctx.lon_lat_alt.load();
        auto pos_enu = ctx.pos_enu.load();
        ctx.takeoff_enu.emplace(
            Eigen::Vector3d{pos_enu.x(), pos_enu.y(), alt_});
        std::visit(
            [](const auto& obj) { return obj.resolve({"success", "OK"}); },
            event_);
        return StateAction::unhandled();
    } else {
        std::visit(
            [](const auto& obj) {
                if (!obj.is_settled()) obj.reject("takeoff cmd error");
            },
            event_);
        // 失败时动态决定回退
        if (ctx.robot->check_hover(ctx)) {
            return StateAction::step<HoverState>();
        } else {
            return StateAction::step<GroundState>();
        }
    }
}

StateAction TakeoffState::on_tick(double dt, RobotContext& ctx) {
    // 先检查 event 的超时状况
    bool is_settled =
        std::visit([](const auto& obj) { return obj.is_settled(); }, event_);

    double now = ctx.engine->get_time_provider()->now();
    if (!is_settled && state_utils::get_time_span(start_time_, now) >
                           GlobalConfig.GetConfig().prearm_timeout) {
        // 超时了，拒绝
        std::visit(
            [](const auto& obj) { return obj.reject("Takeoff Timeout!"); },
            event_);
        // 失败时动态决定回退
        if (ctx.robot->check_hover(ctx)) {
            return step<HoverState>();
        } else {
            return step<GroundState>();
        }
    }

    // 执行起飞监控逻辑
    auto& cfg = GlobalConfig.GetConfig();
    double alt = ctx.pos_enu.load().z();
    bool is_stall = checker_->is_stall({alt}, now);
    if (is_stall) {
        if (!ctx.robot->check_hover(ctx)) {
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

    // 卡住强制进入 Ground 或者 Hover 状态
    if (is_stall && !ctx.robot->check_hover(ctx)) {
        return step<GroundState>();
    }

    if (!step_waypoint_) {
        report_takeoff(ctx);
        return step<HoverState>();
    } else {
        auto event = std::get<SetWaypointEvent>(event_);
        report_takeoff(ctx);
        return step<WaypointState::LiftingState>(std::tuple(event),
                                                 std::tuple<>());
    }
}