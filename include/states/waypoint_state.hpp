#pragma once
#include <memory>
#include <optional>
#include <tuple>

#include "state_common.hpp"
#include "state_utils.hpp"
#include "states/hover_state.hpp"
#include "states/land_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"
#include "states/waypoint_state.hpp"

class WaypointState : public dk::BaseState<RobotContext, WaypointState, void> {
   public:
    std::vector<Eigen::Vector3d> wp_list_;
    std::optional<std::vector<std::vector<std::string>>> event_list_;
    state_utils::FinishAction action_;
    int wp_idx_;  // 当前点的序号

   private:
    void waypoint_finish(RobotContext& ctx);

    void run_wp_envet(std::string e);

    bool check_arrive(RobotContext& ctx);

    void publish_wp(RobotContext& ctx);

   public:
    using AllowedEvents = std::tuple<>;

    void on_enter(RobotContext& ctx) override;

    void on_exit(RobotContext& ctx) override;

    WaypointState(SetWaypointEvent e, state_utils::FinishAction action);

    static constexpr std::string_view static_name() { return "航点模式"; };

    Eigen::Vector3d get_cur_wp();

    class LiftingState;

    class ExcuteState;
};

class WaypointState::LiftingState : public dk::BaseState<RobotContext, LiftingState, WaypointState> {
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    double target_alt_;
    double last_alt_;
    double last_yaw_;
    double target_yaw_;
    std::shared_ptr<state_utils::StallChecker<2>> checker_;

   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    static constexpr std::string_view static_name() { return "调整高度"; }

    LiftingState();

    void on_enter(RobotContext& ctx) override;

    void on_exit(RobotContext& ctx) override;

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);
};

class WaypointState::ExcuteState : public dk::BaseState<RobotContext, ExcuteState, WaypointState> {
   public:
    double wp_dist_;           // 到下一个目标的距离
    Eigen::Vector3d wp_goal_;  // 点在enu坐标系下的目标
    using AllowedEvents = std::tuple<dk::TickEvent>;

   public:
    ExcuteState() = default;

    static constexpr std::string_view static_name() { return "航点模式"; }

    bool check_arrive(RobotContext& ctx);

    StateAction on_event(const dk::TickEvent& event, RobotContext& ctx);

    void on_enter(RobotContext& ctx);
};
