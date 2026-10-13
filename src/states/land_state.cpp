#include "states/land_state.hpp"

#include <plugins/telemetry/telemetry.h>
// std_msgs 为 ROS 专有头文件, 非 ROS 独立构建下无此头文件路径;
// 下文 pland 发布逻辑均由 if constexpr 编译期裁剪, 仅 ROS 构建实例化。
#ifdef USE_ROS
#include <std_msgs/Empty.h>
#endif

#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "states/ground_state.hpp"
#include "states/state_common.hpp"

// 1. 基础模板：默认继承 false_type，表示没有该属性
template <typename T, typename = void>
struct has_land_target : std::false_type {};

// 2. 偏特化模板：检查 T 是否具有 tag_pos_map 属性
// 注: 仅在 USE_ROS 下启用。pland 精准降落依赖 ROS 话题(/pland/start 等)与
// ctx 中的 ros::Publisher, 非 ROS 构建下即使配置含 tag_pos_map 也无法执行,
// 恒按 false_type 处理, 使 pland 相关代码在编译期被整体裁剪。
#ifdef USE_ROS
template <typename T>
struct has_land_target<T, std::void_t<decltype(std::declval<T>().tag_pos_map)>>
    : std::true_type {};
#endif

void LandState::on_exit(RobotContext& ctx) {
    StopRecordEvent e2;
    ctx.engine->dispatch(e2);
    stop_pland(ctx);  // 触发取消外部精准降落模块
    spdlog::info("[LandState] Exited land state, stopped pland");
}

// 启动外部精准降落模块与参数同步
// 注: pland 依赖 ROS 话题与 ctx 中的 ros::Publisher, 整个分支用 USE_ROS 守卫;
// 不能仅依赖 if constexpr 裁剪——分支里的 std_msgs::Empty 等非依赖名在模板
// 定义阶段就会被语法检查, 非 ROS 构建下该名字不存在会直接报错。
template <typename ContextType>
bool LandState::setup_pland(ContextType& ctx) {
    if (!do_pland_) return false;
#ifdef USE_ROS
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
#endif
    return false;
}

StateAction LandState::on_enter(RobotContext& ctx) {
    bool do_pland = setup_pland(ctx);

    if (!do_pland) {
        if (!ctx.robot->land())
            spdlog::error("[LandState] 切换 LAND 模式失败!");
    } else if (!ctx.robot->set_mode(mavsdk::Telemetry::FlightMode::Offboard)) {
        // 精准降落要求 GUIDED/Offboard 模式
        spdlog::error(
            "[LandState] 切换 GUIDED 失败, pland 的设定值可能不会被执行!");
    }
    return StateAction::unhandled();
}

template <typename ContextType>
void LandState::stop_pland(ContextType& ctx) {
#ifdef USE_ROS
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
#endif
}
StateAction LandState::on_tick(double dt, RobotContext& ctx) {
    if (ctx.robot->is_landed(ctx) || !ctx.robot->check_hover(ctx)) {
        ctx.robot->land();  // 为了安全起见切到降落模式
        LOG_STATE_STEP("GroundState");
        return step<GroundState>();
    }
    return StateAction::unhandled();
}