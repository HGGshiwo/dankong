#pragma once
#include <chrono>
#include <memory>
#include <optional>

#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "features/mavlink/events.hpp"
#include "ground_state.hpp"
#include "hover_state.hpp"
#include "robot_context.hpp"
#include "state_common.hpp"
#include "state_utils.hpp"

class TakeoffState : public dk::BaseState<RobotContext, TakeoffState, void> {
   public:
    using AllowedEvents = std::tuple<>;
    double start_time_{0.0};
    std::optional<TakeoffEvent> event_;
    double alt_{0.0};

    std::shared_ptr<state_utils::StallChecker<1>> checker_;

   public:
    explicit TakeoffState(TakeoffEvent e);
    explicit TakeoffState(double alt);
    // 完全重入时旧实例被销毁, 未 settle 的事件主动 reject,
    // 避免对应的调用方 (如 HTTP 请求) 悬挂到超时
    ~TakeoffState() override {
        if (event_.has_value() && !event_->is_settled()) {
            event_->reject("被新的起飞指令取代");
        }
    }

    static StateAction before_enter(RobotContext& ctx, const TakeoffEvent& e);
    static StateAction before_enter(RobotContext& ctx, double alt);
    StateAction on_enter(RobotContext& ctx) override;
    StateAction on_tick(double dt, RobotContext& ctx) override;

    void report_takeoff(RobotContext& ctx) {
        ctx.ws_manager->publish_reliable(
            nlohmann::json{{"type", "event"}, {"event", "takeoff"}});
    }

    static constexpr std::string_view static_name() { return "起飞状态"; }
};