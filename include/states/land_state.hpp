#pragma once
#include "robot_context.hpp"
#include "state_common.hpp"

class LandState : public dk::BaseState<RobotContext, LandState, void> {
   public:
    bool do_pland_ = false;
    using AllowedEvents = std::tuple<>;
    LandState(bool do_pland) : do_pland_(do_pland) {};

    StateAction on_enter(RobotContext& ctx) override;
    StateAction on_tick(double dt, RobotContext& ctx) override;
    void on_exit(RobotContext& ctx) override;

    template <typename RobotContext>
    void stop_pland(RobotContext& ctx);

    template <typename ContextType>
    bool setup_pland(ContextType& ctx);

    static constexpr std::string_view static_name() { return "降落状态"; }
};