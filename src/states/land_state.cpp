#include "states/land_state.hpp"

#include <plugins/telemetry/telemetry.h>
#include <std_msgs/Empty.h>

#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "states/ground_state.hpp"
#include "states/state_common.hpp"

// 1. 基础模板：默认继承 false_type，表示没有该属性
template <typename T, typename = void>
struct has_land_target : std::false_type {};

// 2. 偏特化模板：检查 T 是否具有 tag_pos_map 属性
template <typename T>
struct has_land_target<T, std::void_t<decltype(std::declval<T>().tag_pos_map)>>
    : std::true_type {};

void LandState::on_exit(RobotContext& ctx) {
    StopRecordEvent e2;
    ctx.engine->dispatch(e2);
    stop_pland(ctx);  // 触发取消外部精准降落模块
    spdlog::info("[LandState] Exited land state, stopped pland");
}

// 启动外部精准降落模块与参数同步
template <typename ContextType>
bool LandState::setup_pland(ContextType& ctx) {
    if (!do_pland_) return false;

    if constexpr (has_land_target<decltype(GlobalConfig.GetConfig())>::value) {
        ctx.do_pland.store(true);
        // 2. 云台转动至垂直下视 90 度
        SetGimbalEvent e1;
        e1.angle = 90.0;

        [&](auto& global_cfg) {
            auto config = global_cfg.GetConfig();
            if (config.pland_gimbal_abs.get()) {
                e1.mode = "abs";
            } else {
                e1.mode = "body";
            }
        }(GlobalConfig);

        ctx.engine->dispatch(e1);

        // 3. 发布 /pland/start 话题激活外部精准降落模块
        //    (发布器已在 PlandFeature::setup(TagInit) 时注册)
        std_msgs::Empty start_msg;
        ctx.pland_start_pub.publish(start_msg);
        spdlog::info("[LandState] Published /pland/start (subscribers={})",
                     ctx.pland_start_pub.getNumSubscribers());

        return true;
    }
    return false;
}

StateAction LandState::on_enter(RobotContext& ctx) {
    bool do_pland = setup_pland(ctx);

    if (!do_pland)
        ctx.robot->land();
    else
        ctx.robot->set_mode(
            mavsdk::Telemetry::FlightMode::Offboard);  // 精准降落要求
                                                       // GUIDED/Offboard 模式
    return StateAction::unhandled();
}

template <typename ContextType>
void LandState::stop_pland(ContextType& ctx) {
    if constexpr (has_land_target<decltype(GlobalConfig.GetConfig())>::value) {
        if (do_pland_) {
            // 发布 /pland/cancel 话题取消外部精准降落模块
            //    (发布器已在 PlandFeature::setup(TagInit) 时注册)
            std_msgs::Empty cancel_msg;
            ctx.pland_cancel_pub.publish(cancel_msg);
            spdlog::info(
                "[LandState] Published /pland/cancel to stop precision landing "
                "module");

            ctx.do_pland.store(false);
        }
    }
}
StateAction LandState::on_tick(double dt, RobotContext& ctx) {
    if (ctx.robot->is_landed(ctx) || !ctx.robot->check_hover(ctx)) {
        ctx.robot->land();  // 为了安全起见切到降落模式
        LOG_STATE_STEP("GroundState");
        return step<GroundState>();
    }
    return StateAction::unhandled();
}