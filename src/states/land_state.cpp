#include "states/land_state.hpp"

#include "core/global_config.hpp"
#include "features/pland/ilanding_controller.hpp"
#include "features/pland/ilanding_detector.hpp"
#include "robot_context.hpp"
#include "states/ground_state.hpp"

// 1. 基础模板：默认继承 false_type，表示没有该属性
template <typename T, typename = void>
struct has_land_target : std::false_type {};

// 2. 偏特化模板：尝试获取 T 的 name 属性。如果获取成功，匹配此模板并继承
// true_type
template <typename T>
struct has_land_target<T, std::void_t<decltype(std::declval<T>().tag_pos_map)>>
    : std::true_type {};

void LandState::on_exit(RobotContext& ctx) {
    StopRecordEvent e2;
    ctx.engine->dispatch(e2);
}

// 该函数是必须的，否则has_land_target仍然会检查该分支
template <typename ContextType>
bool LandState::setup_pland(ContextType& ctx) {
    if (!do_pland_) return false;
    ctx.do_pland.store(true);
    // 1. 修改：必须判断 ContextType，而不是写死 RobotContext
    if constexpr (has_land_target<decltype(GlobalConfig.GetConfig())>::value) {
        ctx.land_detector->start(30);
        ctx.land_controller->start(50);

        SetGimbalEvent e1;
        e1.angle = 90.0;

        // 2. 修改：用泛型 lambda 强行让 GlobalConfig 变成依赖类型（Dependent
        // Type） 这样只要外层 if constexpr 为
        // false，这段代码就永远不会被实例化和检查。
        [&](auto& global_cfg) {
            auto config = global_cfg.GetConfig();
            if (config.pland_gimbal_abs.get()) {
                e1.mode = "abs";
            } else {
                e1.mode = "body";
            }
        }(GlobalConfig);  // 立即调用并传入全局的 GlobalConfig

        ctx.engine->dispatch(e1);
        return true;
    }
    return false;
}

void LandState::on_enter(RobotContext& ctx) {
    bool do_pland = setup_pland(ctx);

    if (!do_pland)
        ctx.robot->land();
    else
        ctx.robot->set_mode("GUIDED");  // 精准降落要求GUIDED模式
}

template <typename ContextType>
void LandState::stop_pland(ContextType& ctx) {
    if constexpr (has_land_target<decltype(GlobalConfig.GetConfig())>::value) {
        if (do_pland_) {
            ctx.land_detector->stop();
            ctx.land_controller->stop();
            ctx.do_pland.store(false);
        }
    }
}
StateAction LandState::on_event(const dk::TickEvent& e, RobotContext& ctx) {
    if (ctx.robot->is_landed(ctx)) {
        stop_pland(ctx);
        return step<GroundState>();
    }
    return StateAction::unhandled();
}