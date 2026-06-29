#include "features/control/event_listener.hpp"

#include "Eigen/Dense"
#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/init_state.hpp"
#include "states/land_state.hpp"
#include "states/posvel_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"
#include "states/waypoint_state.hpp"
#include "utils/fixed_string64.hpp"

// ---------- 成员函数实现 ----------
void ControlEventListener::on_event(const dk::TickEvent& event,
                                    RobotContext& ctx) {
    bool on_ground = ctx.engine->is_active_state<GroundState>() ||
                     ctx.engine->is_active_state<InitState>() ||
                     ctx.engine->is_active_state<TakeoffState>() ||
                     ctx.engine->is_active_state<LandState>();
    if (!on_ground && !ctx.robot->check_hover(ctx)) {
        ctx.engine->template step<GroundState>();
    }
}

void ControlEventListener::on_event(const SetPosVelEvent& event,
                                    RobotContext& ctx) {
    if (ctx.engine->is_active_state<PosVelState>()) {
        //  不重复进入PosVelState，由自己handle
        event.resolve({"success", "OK"});
        return;
    }
    if (ctx.engine->is_active_state<WaypointState>()) {
        ctx.engine->template step<PosVelState<WaypointState>::MoveState>(
            std::tuple(event), std::tuple<>());
        event.resolve({"success", "OK"});
        return;
    } else if (ctx.engine->is_active_state<HoverState>()) {
        ctx.engine->template step<PosVelState<HoverState>::MoveState>(
            std::tuple(event), std::tuple<>());
        event.resolve({"success", "OK"});
        return;
    } else {
        event.reject("set_posvel只允许在waypoint或者hover状态调用! 当前状态: " +
                     ctx.engine->get_state_name());
        return;
    }
}

dk::Future<bool> prearm_check(RobotContext& ctx) {
    if (!state_utils::should_do_prearm_check(ctx.robot))
        return dk::Promise<bool>::resolve(ctx.engine, true);

    // 检查是否已经通过prearm check
    int bits = 0x10000000;
    if ((ctx.sensor_health & bits) == bits) {
        return dk::Promise<bool>::resolve(ctx.engine, true);
    }

    auto p = std::make_shared<dk::Promise<bool>>(ctx.engine);

    // 等待错误信息并使用 robot 进行协议无关的健康状态判定
    ctx.engine->wait_for(
        1000,
        [p, robot = ctx.robot](const StatusTextEvent& status_text) -> bool {
            if (robot->is_prearm_msg(status_text.text)) {
                p->reject(status_text.text);
                return true;
            }
            return false;
        },
        [p, robot = ctx.robot](const SysStatusEvent& sys_status) -> bool {
            if (robot->check_sensor_health(sys_status.data)) {
                p->resolve(true);
                return true;
            }
            return false;
        });

    ctx.engine->post_future_task([&ctx]() { ctx.robot->run_prearm_checks(); });
    return p->get_future();
}

void ControlEventListener::on_event(const PrearmEvent& event,
                                    RobotContext& ctx) {
    prearm_check(ctx)
        .then([event](bool res) {
            if (res) event.resolve({"success", nlohmann::json{{"arm", true}}});
            event.resolve(
                {"success",
                 nlohmann::json{{"arm", false}, {"reason", "Unknown Error"}}});
        })
        .catch_error([event](std::exception_ptr exp) {
            try {
                std::rethrow_exception(exp);
            } catch (const std::exception& e) {
                event.resolve(
                    {"success",
                     nlohmann::json{{"arm", false}, {"reason", e.what()}}});
            }
        });
}

void ControlEventListener::on_event(const TakeoffEvent& event,
                                    RobotContext& ctx) {
    if (ctx.engine->is_active_state<InitState>()) {
        event.reject("初始状态无法起飞!");
        return;
    }
    if (!ctx.engine->is_active_state<GroundState>()) {
        event.reject("只允许在地面状态起飞!");
        return;
    }
    ctx.engine->step<TakeoffState::PrearmCheckState>(
        std::tuple(std::move(event)), std::tuple<>());
}

void ControlEventListener::on_event(const dk::StateChangeEvent& event,
                                    RobotContext& ctx) {
    spdlog::info("State change {} -> {}", event.prev, event.cur);
}

void ControlEventListener::on_event(const SetWaypointEvent& event,
                                    RobotContext& ctx) {
    bool is_init = ctx.engine->is_active_state<InitState>();
    bool is_ground = ctx.engine->is_active_state<GroundState>();
    bool wp_empty = event.waypoint.empty();
    auto action = event.finish_action;

    // 1. 拦截非法状态：InitState 无法起飞或执行任务
    if (is_init) {
        event.reject("初始状态下无法执行指令!");
        return;
    }

    // 2. 规则校验：航点数为0时的限制
    if (wp_empty) {
        // 如果在地面，且要求返航或降落 -> 报错 (因为这两者只允许在空中执行)
        if (is_ground &&
            (action == FinishAction::RETURN || action == FinishAction::LAND)) {
            event.reject("航点个数为0，不具备返航或降落条件!");
            return;
        }
        // 注：如果是地面且要求悬停(HOVER)，允许放行，后续会在 is_ground
        // 分支触发起飞并悬停
    }

    // 3. 状态流转 (航点数 >= 1 的情况会直接走到这里，全都允许执行)
    if (is_ground) {
        // 地面状态：进入起飞前检查流程，并将事件和最终动作向后传递
        // （原逻辑中地面状态无需立即 resolve，通常在起飞完成或校验失败后
        // resolve）
        ctx.engine->step<TakeoffState::PrearmCheckState>(
            std::make_tuple(std::move(event)), std::tuple<>());
    } else {
        // 空中执行逻辑分流
        if (wp_empty && action == FinishAction::LAND) {
            // 空中原地降落（无航点）：直接进入 LandState
            event.resolve({"success", "OK"});
            ctx.engine->step<LandState>(std::tuple(event.do_pland));
        } else {
            // 检查是不是半路进入hover，其实没有记录起飞点
            auto takeoff_lon_lat_alt = ctx.takeoff_lon_lat_alt.load();
            if (action == FinishAction::RETURN &&
                takeoff_lon_lat_alt.norm() < 1.0) {
                event.reject("未记录起飞点，请使用降落");
            } else {
                event.resolve({"success", "OK"});
                // 其他情况（有航点，或空中原地悬停/返航）：进入
                // LiftingState 执行航点处理
                ctx.engine->step_reenter_all<WaypointState::LiftingState>(
                    std::make_tuple(std::move(event)), std::tuple<>());
            }
        }
    }
}

void ControlEventListener::on_event(const SetModeEvent& event,
                                    RobotContext& ctx) {
    using Promise = dk::Promise<bool>;
    auto p = std::make_shared<Promise>(ctx.engine);

    // 1. 提前获取 Future，避免后续 Promise 被 move 后无法获取
    auto future = p->get_future();

    if (ctx.mode.load() == event.mode) {
        // 同步完成的情况，直接 resolve，局部变量 p 会随函数结束正常销毁
        p->resolve(true);
    } else {
        ctx.robot->set_mode(event.mode);

        // 2. 将 promise 移动 (Move) 进 Lambda 中
        // 这样 p 的生命周期就交给了引擎的事件队列，执行完后自动释放

        ctx.engine->wait_for(2000,
                             [mode = FixedString64(event.mode),
                              p](const FlightModeEvent& e) mutable -> bool {
                                 if (mode != e.cur) return false;
                                 p->resolve(true);
                                 return true;
                             });
    }

    // 3. 处理 event 回调
    future
        .then([event](bool res) mutable {
            if (res)
                event.resolve({"success", "OK"});
            else
                event.reject("set mode error");
        })
        .catch_error(
            [event](std::exception_ptr e) mutable { event.reject(e); });
}

void ControlEventListener::on_event(const RebootFcuEvent& event,
                                    RobotContext& ctx) {
    auto res = ctx.robot->reboot_fcu();
    if (res)
        event.resolve({"success", "OK"});
    else
        event.reject("reboot fcu error");
}

void ControlEventListener::on_event(const GetWpEvent& event,
                                    RobotContext& ctx) {
    auto mission_data = ctx.mission_data.load();
    nlohmann::json wp_list = nlohmann::json::array();

    for (int i = 0; i < mission_data.size(); ++i) {
        auto cur_mission = mission_data[i];
        wp_list.push_back(
            {cur_mission["lon"], cur_mission["lat"], cur_mission["alt"]});
    }
    event.resolve({"success", wp_list});
}

void ControlEventListener::on_event(const GetGpsEvent& event,
                                    RobotContext& ctx) {
    auto gps = ctx.lon_lat_alt.load();
    event.resolve({"success", {gps.y(), gps.x(), gps.z()}});
}

void ControlEventListener::on_event(const GetParamEvent& event,
                                    RobotContext& ctx) {
    ctx.engine->post_background_task<bool>([robot = ctx.robot, event]() {
        nlohmann::json j;
        j["param"]["_value"] = robot->get_all_params();
        event.resolve({"success", j});
        return true;
    });
}

void ControlEventListener::on_event(const SetParamEvent& event,
                                    RobotContext& ctx) {
    for (auto& [k, v] : event.param.items()) {
        auto j = v["value"];
        ApmParam param;
        if (j.is_string()) {
            param = j.get<std::string>();
        } else if (j.is_number_integer()) {
            param = j.get<int>();  // 注意：可能溢出，下文有说明
        } else if (j.is_number_float()) {
            param = j.get<double>();
        } else {
            event.reject("Unknow data!");
            spdlog::error("Unonw data: {}", j.dump());
            return;
        }
        event.resolve({"success", "OK"});
        ctx.robot->set_param(k, param);
    }
}

void ControlEventListener::on_event(const DisarmEvent& event,
                                    RobotContext& ctx) {
    ctx.robot->disarm();
    event.resolve({"success", "OK"});
}

void ControlEventListener::on_event(const RestartEvent& event,
                                    RobotContext& ctx) {
    ctx.odom_ok = false;
    ctx.fcu_connected.store(false);
    ctx.engine->step<InitState>();
}

void ControlEventListener::on_event(const JoystickEvent& event,
                                    RobotContext& ctx) {
    ctx.robot->cmd_vel(Eigen::Vector4d{event.x, event.y, event.z, event.w});
    event.resolve({"success", "OK"});
}