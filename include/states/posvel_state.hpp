#pragma once

#include <Eigen/Dense>
#include <algorithm>  // 包含 std::clamp, std::min, std::max
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <type_traits>

#include "core/global_config.hpp"
#include "dk/state.hpp"
#include "features/control/events.hpp"
#include "features/tracker/tracker.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"
#include "states/arm_state.hpp"
#include "states/hover_state.hpp"
#include "states/posvel_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"
#include "states/waypoint_state.hpp"

template <typename ParentState>
class PosVelState : public dk::BaseState<RobotContext, PosVelState<ParentState>,
                                         ParentState> {
   public:
    static constexpr std::string_view static_name() { return "指点控制"; }
    PosVelState(SetPosVelEvent e);

    static StateAction before_enter(RobotContext& ctx,
                                    const SetPosVelEvent& e) {
        if (!ctx.arm.load()) {
            StateFlags flags{.is_posvel = true};
            if (ctx.robot->should_arm_before_enter(flags)) {
                return StateAction::plan([](auto& plan, auto& /*ctx*/) {
                    plan.template push_front<ArmState>();
                });
            }
        }
        return StateAction::handled();
    }

    StateAction on_enter(RobotContext& ctx) override {
        return StateAction::unhandled();
    }

    StateAction on_tick(double dt, RobotContext& ctx) override;

    void on_exit(RobotContext& ctx) override {
        ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                  std::nullopt, CmdFrame::ENU);
    }

    using AllowedEvents = std::tuple<SetPosVelEvent>;

    SetPosVelEvent event_;

    double start_time_;

    // [修改点 2]：将控制相关的持续性状态变量移到父状态中
    double last_tick_time_;
    Eigen::Vector2d current_vel_xy_ = Eigen::Vector2d::Zero();
    double current_z_speed_ = 0.0;  // 新增：用于 Z 轴的平滑加减速
    bool is_first_tick_ = true;

    StateAction on_event(const SetPosVelEvent& e, RobotContext& ctx);

    // 是否指定了目标朝向 (yaw 未传时不锁定朝向、不进转向子状态)
    bool has_yaw() const { return event_.yaw.has_value(); }

    // fix_yaw 的实际生效值 (不传默认 true, 同 mavproxy)
    bool fix_yaw() const { return event_.fix_yaw.value_or(true); }

    // pos 支持 [lon,lat] 或 [lon,lat,alt], 第三值 (alt) 一律忽略,
    // 目标高度始终取飞机当前高度 (cur_alt)
    static Eigen::Vector3d resolve_target(const nlohmann::json& j,
                                          double cur_alt) {
        if (!j.is_array() || (j.size() != 2 && j.size() != 3)) {
            throw std::invalid_argument(
                "pos 必须是 [lon,lat] 或 [lon,lat,alt]");
        }
        return {j[0].get<double>(), j[1].get<double>(), cur_alt};
    }

    StateAction get_next_state();

    class MoveState;
    class LaterTurnState;

    void set_target(const SetPosVelEvent& e);
};

template <typename ParentState>
class PosVelState<ParentState>::MoveState
    : public dk::BaseState<RobotContext, MoveState, PosVelState<ParentState>> {
   public:
    // 必须声明处理 SetPosVelEvent: 否则飞行中的新指令冒泡到父状态后
    // 只更新 event_ 而不重入本状态, base_move 不执行, Tracker 仍追旧目标
    using AllowedEvents = std::tuple<SetPosVelEvent>;

    // 父状态已接管初始化，这里可以保持为空，或者仅做特定子状态的日志等
    StateAction on_enter(RobotContext& ctx) override;
    StateAction on_tick(double dt, RobotContext& ctx) override;

    StateAction on_event(const SetPosVelEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点移动"; }

   private:
    void base_move(RobotContext& ctx);
};

template <typename ParentState>
class PosVelState<ParentState>::LaterTurnState
    : public dk::BaseState<RobotContext, LaterTurnState,
                           PosVelState<ParentState>> {
   public:
    using AllowedEvents = std::tuple<>;
    StateAction on_enter(RobotContext& ctx) override;
    StateAction on_tick(double dt, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "指点转向"; }
};

template <typename ParentState>
PosVelState<ParentState>::PosVelState(SetPosVelEvent e)
    : event_(e), start_time_(0.0) {
    if (event_.yaw) event_.yaw = state_utils::yaw_ned_to_enu(*event_.yaw);
}

// 工具函数，设置posvel目标
template <typename ParentState>
void PosVelState<ParentState>::set_target(const SetPosVelEvent& e) {
    event_ = e;
    if (event_.yaw) event_.yaw = state_utils::yaw_ned_to_enu(*event_.yaw);

    // 由于目标更新，重置首次 tick 标志，使得下一次重新计算 dt
    // 此时不会归零当前速度 (current_xy_speed_ /
    // current_z_speed_)，保证切换目标时的平滑性
    is_first_tick_ = true;
}

template <typename ParentState>
StateAction PosVelState<ParentState>::on_event(const SetPosVelEvent& e,
                                               RobotContext& ctx) {
    set_target(e);
    start_time_ = ctx.engine->get_time_provider()->now();
    if (!ctx.engine->is_active_state<PosVelState<ParentState>::MoveState>()) {
        return this->template step<PosVelState<ParentState>::MoveState>(
            std::tuple<>(), std::tuple<>());
    }
    return StateAction::handled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::get_next_state() {
    // follow 式退出: 到达/超时/转向完成后显式 step 回父状态对应的状态,
    // 不再依赖 plan next()+fallback (残留 fallback 会在任务完成后被误触发)。
    // LiftingState 的无参 step 与 StopFollowEvent 的处理一致:
    // WaypointState 走 try_reuse 保留任务进度, 从 wp_idx_ 断点续飞。
    if constexpr (std::is_same_v<ParentState, WaypointState>) {
        // 必须无参 (与 StopFollowEvent 处理一致); 带空元组会被当成
        // WaypointState 的构造参数导致 build_path 抛 logic_error
        return this->template step<WaypointState::LiftingState>();
    } else {
        return this->template step<HoverState>();
    }
}

template <typename ParentState>
StateAction PosVelState<ParentState>::on_tick(double dt, RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    // 超时阈值: 请求可指定, 不传用配置 (同 mavproxy 的 timeout 字段)
    double timeout =
        event_.timeout.value_or(GlobalConfig.GetConfig().posvel_timeout.get());
    if (state_utils::get_time_span(start_time_, now) > timeout) {
        spdlog::error("[PosvelState] set_posvel timeout!");
        return get_next_state();
    }
    return StateAction::handled();
}

// 工具函数，调用ctx.tracker
template <typename ParentState>
void PosVelState<ParentState>::MoveState::base_move(RobotContext& ctx) {
    auto p = this->parent();
    auto current_pos = ctx.pos_enu.load();
    auto target_gps = p->resolve_target(p->event_.pos, ctx.pos_enu.load().z());
    auto target_pos = state_utils::gps_to_enu(ctx.lon_lat_alt.load(),
                                              current_pos, target_gps);
    // [可选] 固定朝向: 仅当 fix_yaw 且指定了 yaw 时生效; 否则传 nullopt,
    // Tracker 会自动计算朝向 (机头朝运动方向)
    std::optional<double> target_yaw = std::nullopt;
    if (p->fix_yaw() && p->has_yaw()) {
        target_yaw = p->event_.yaw;
    }
    // 只在进入状态时下发一次指令 (Tracker 内部会接管 50Hz 的连续控制)
    {
        auto cur_gps = ctx.lon_lat_alt.load();
        spdlog::info(
            "[Posvel] 下发: gps=({:.7f}, {:.7f}, {:.2f}) -> enu=({:.2f}, "
            "{:.2f}, "
            "{:.2f}) cur_enu=({:.2f}, {:.2f}, {:.2f}) cur_gps=({:.7f}, {:.7f}, "
            "{:.2f}) vel={:.1f} fix_yaw={} yaw={} timeout={}",
            target_gps.x(), target_gps.y(), target_gps.z(), target_pos.x(),
            target_pos.y(), target_pos.z(), current_pos.x(), current_pos.y(),
            current_pos.z(), cur_gps.x(), cur_gps.y(), cur_gps.z(),
            p->event_.vel, p->fix_yaw(),
            p->has_yaw() ? std::to_string(*p->event_.yaw) : std::string("无"),
            p->event_.timeout.has_value() ? std::to_string(*p->event_.timeout)
                                          : std::string("配置值"));
    }
    ctx.tracker->send_pos_cmd(
        target_pos,    // 目标位置
        target_yaw,    // 目标偏航角
        std::nullopt,  // 没有前馈速度
        std::nullopt,  // 没有前馈角速度
        p->event_.vel,  // XY 巡航速度限制 (你原代码里的 target_speed_mag)
        std::nullopt,  // Z 巡航速度限制
        std::nullopt, CmdFrame::ENU);
}

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_enter(RobotContext& ctx) {
    auto p = this->parent();
    p->start_time_ = ctx.engine->get_time_provider()->now();
    p->last_tick_time_ = p->start_time_;
    p->current_vel_xy_ = Eigen::Vector2d::Zero();
    p->current_z_speed_ = 0.0;
    p->is_first_tick_ = true;
    base_move(ctx);
    return StateAction::unhandled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_event(
    const SetPosVelEvent& e, RobotContext& ctx) {
    this->parent()->set_target(e);
    // 对齐 mavproxy: 每次新请求刷新超时计时 (无新指令才算超时)
    this->parent()->start_time_ = ctx.engine->get_time_provider()->now();
    base_move(ctx);
    return StateAction::handled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::MoveState::on_tick(double dt,
                                                         RobotContext& ctx) {
    auto p = this->parent();
    auto& cfg = GlobalConfig.GetConfig();
    auto current_pos = ctx.pos_enu.load();
    double current_yaw = ctx.yaw_enu;
    auto target_gps = p->resolve_target(p->event_.pos, ctx.pos_enu.load().z());
    auto target_pos = state_utils::gps_to_enu(ctx.lon_lat_alt.load(),
                                              current_pos, target_gps);

    // 计算误差向量
    Eigen::Vector3d pos_err = target_pos - current_pos;
    Eigen::Vector2d xy_err = pos_err.head<2>();
    double dist_3d = pos_err.norm();
    double dist_xy = xy_err.norm();

    // 到达判定: 距离到位 且 水平速度已收敛。提前切出位置控制会让
    // Tracker 的减速刹车被零速指令替代, 动量导致滑过目标后折返振荡
    double speed = ctx.vel_enu.load().head<2>().norm();
    bool dist_ok = ctx.robot->is_alt_enable()
                       ? (dist_3d < cfg.posvel_arrive_tolerance.get())
                       : (dist_xy < cfg.posvel_arrive_tolerance.get());
    // 1. 检查是否到达 (分流语义同 mavproxy):
    //    fix_yaw&&有yaw: 全程已锁定, 直接完成
    //    !fix_yaw&&有yaw: 到达后进转向子状态调整朝向
    //    无yaw: 到达直接完成 (mavproxy 的 POSVEL_YAW 无 yaw 直接退回)
    if (dist_ok && speed < cfg.posvel_stop_speed.get()) {
        if (!p->fix_yaw() && p->has_yaw()) {
            return this->template step<LaterTurnState>();
        } else {
            return p->get_next_state();
        }
    }

    return StateAction::unhandled();  // 让父状态处理超时
}

template <typename ParentState>
StateAction PosVelState<ParentState>::LaterTurnState::on_enter(
    RobotContext& ctx) {
    return StateAction::unhandled();
}

template <typename ParentState>
StateAction PosVelState<ParentState>::LaterTurnState::on_tick(
    double dt, RobotContext& ctx) {
    auto p = this->parent();
    auto& cfg = GlobalConfig.GetConfig();
    // 无目标朝向时不调整, 直接完成 (同 mavproxy POSVEL_YAW 无 yaw 直接退回)
    if (!p->has_yaw()) {
        return p->get_next_state();
    }
    double current_yaw = ctx.yaw_enu;
    double target_yaw = *p->event_.yaw;

    if (state_utils::get_yaw_diff(target_yaw, current_yaw) <
        cfg.yaw_tolerance) {
        return p->get_next_state();
    }

    ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), target_yaw, std::nullopt,
                              CmdFrame::ENU);

    return StateAction::unhandled();  // 父状态处理超时
}