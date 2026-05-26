#pragma once
#include "state_common.hpp"

class LandState : public dk::BaseState<RobotContext, LandState, void> {
   public:
    int land_target_id_;

    using AllowedEvents = std::tuple<dk::TickEvent>;
    LandState(int land_target_id) { land_target_id_ = land_target_id; };

    void on_enter(RobotContext& ctx) override;

    StateAction on_event(const dk::TickEvent& e, RobotContext& ctx);

    static constexpr std::string_view static_name() { return "降落状态"; }
};