#include "states/land_state.hpp"

#include "robot_context.hpp"
#include "states/ground_state.hpp"

// 1. 基础模板：默认继承 false_type，表示没有该属性
template <typename T, typename = void>
struct has_land_target : std::false_type {};

// 2. 偏特化模板：尝试获取 T 的 name 属性。如果获取成功，匹配此模板并继承
// true_type
template <typename T>
struct has_land_target<T,
                       std::void_t<decltype(std::declval<T>().land_target_id)>>
    : std::true_type {};

template <typename ContextType>
void update_land_target(ContextType& ctx, int& land_target_id_) {
    // 此时 ContextType 是模板参数
    bool do_pland = false;
    if constexpr (has_land_target<ContextType>::value) {
        // 因为 ctx 依赖于模板参数，如果条件为
        // false，编译器会直接丢弃这一行，不报错
        ctx.land_target_id.store(land_target_id_);
        if (land_target_id_ >= 0) {
            do_pland = true;
            ctx.land_detector->set_target_id(land_target_id_);
            ctx.land_detector->start(50);
        }
    }
    if (!do_pland)
        ctx.robot->land();
    else
        ctx.robot->set_mode("GUIDED");  // 精准降落要求GUIDED模式
}

void LandState::on_enter(RobotContext& ctx) {
    update_land_target(ctx, land_target_id_);
    SetGimbalEvent e1;
    e1.mode = "body";
    e1.angle = 90.0;
    ctx.engine->dispatch(e1);

    StopRecordEvent e2;
    ctx.engine->dispatch(e2);
}

template <typename ContextType>
void stop_pland(ContextType& ctx, int land_target_id) {
    if constexpr (has_land_target<RobotContext>::value) {
        if (land_target_id >= 0) {
            ctx.land_detector->stop();
        }
    }
}
StateAction LandState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (ctx.robot->is_landed(ctx)) {
        stop_pland(ctx, land_target_id_);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}