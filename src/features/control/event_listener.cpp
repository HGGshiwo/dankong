#include "features/control/event_listener.hpp"

#include <exception>

#include "Eigen/Dense"
#include "core/global_config.hpp"
#include "features/control/events.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "states/arm_state.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/init_state.hpp"
#include "states/land_state.hpp"
#include "states/posvel_state.hpp"
#include "states/state_utils.hpp"
#include "states/takeoff_state.hpp"
#include "states/waypoint_state.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/logger/fglog.hpp"

// ---------- 成员函数实现 ----------

// rangefinder 仅 drone 类机型 Context 才有, 编译期探测, 缺失时记 nan
template <typename C, typename = void>
struct has_rangefinder_alt : std::false_type {};
template <typename C>
struct has_rangefinder_alt<
    C, std::void_t<decltype(std::declval<C>().rangefinder_alt.load())>>
    : std::true_type {};

template <typename C>
static double rangefinder_or_nan(C& ctx) {
    if constexpr (has_rangefinder_alt<C>::value) {
        return ctx.rangefinder_alt.load();
    } else {
        return std::nan("");
    }
}

// tracker 仅 drone 等机型 Context 才有, 编译期探测, 缺失时不记限速日志
template <typename C, typename = void>
struct has_tracker : std::false_type {};
template <typename C>
struct has_tracker<C, std::void_t<decltype(std::declval<C>().tracker)>>
    : std::true_type {};

template <typename C>
static void log_tracker_status(C& ctx) {
    if constexpr (has_tracker<C>::value) {
        if (ctx.tracker) {
            auto info = ctx.tracker->get_status_info();
            spdlog::info(
                "[Tracker] mode={} wp={}/{} target=({:.2f}, {:.2f}, {:.2f}) "
                "期望v=({:.2f}, {:.2f}, {:.2f})|{:.2f} "
                "cmd=({:.2f}, {:.2f}, {:.2f})|{:.2f} "
                "限速eff={:.2f}/{:.2f}(max {:.2f}) 来源=[{}]",
                info.mode, info.wp_idx, info.wp_total, info.target_pose.x(),
                info.target_pose.y(), info.target_pose.z(), info.target_vel.x(),
                info.target_vel.y(), info.target_vel.z(),
                info.target_vel.template head<3>().norm(), info.cmd_vel.x(),
                info.cmd_vel.y(), info.cmd_vel.z(),
                info.cmd_vel.template head<3>().norm(), info.eff_xy, info.eff_z,
                info.max_vel_xy, info.sources);
        }
    }
}

// 0.5Hz 记录关键遥测: 飞控模式 + 三个高度基准 (enu/rel_alt/测距仪) +
// 经纬度, 用于对齐多高度参考系与实机排查
void ControlEventListener::log_telemetry(RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    if (now - last_telemetry_log_ < 2.0) {
        return;
    }
    last_telemetry_log_ = now;
    auto mode = ctx.mode.load();
    auto enu = ctx.pos_enu.load();
    auto lon_lat_alt = ctx.lon_lat_alt.load();
    double rf = rangefinder_or_nan(ctx);
    spdlog::info(
        "[遥测] mode={} pos_enu=({:.2f}, {:.2f}, {:.2f}) lon_lat_alt=({:.7f}, "
        "{:.7f}, {:.2f}) rangefinder={:.2f}",
        std::string(mode.mode_str), enu.x(), enu.y(), enu.z(), lon_lat_alt.x(),
        lon_lat_alt.y(), lon_lat_alt.z(), rf);
    log_tracker_status(ctx);
}

void ControlEventListener::on_tick(double dt, RobotContext& ctx) {
    log_telemetry(ctx);

    bool on_ground = ctx.engine->is_active_state<GroundState>() ||
                     ctx.engine->is_active_state<InitState>() ||
                     ctx.engine->is_active_state<TakeoffState>() ||
                     ctx.engine->is_active_state<ArmState>() ||
                     ctx.engine->is_active_state<LandState>();
    if (!on_ground && !ctx.robot->check_hover(ctx)) {
        spdlog::info("[Contrl] 切换到GroundState");
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
        // follow 式注入: 不 clear plan、不覆盖 fallback, 航线任务自己的
        // 收尾 (悬停/降落) 原样保留, 否则任务完成后会误触发残留 fallback
        // (LiftingState 在 wp_idx_ 越界处重入)。
        // 指点完成后由 get_next_state() 显式 step 回 LiftingState 续飞。
        ctx.engine->plan()
            .push_back<PosVelState<WaypointState>::MoveState>(std::tuple(event),
                                                              std::tuple<>())
            .start();
        event.resolve({"success", "OK"});
        return;
    } else if (ctx.engine->is_active_state<HoverState>()) {
        ctx.engine->plan()
            .push_back<PosVelState<HoverState>::MoveState>(std::tuple(event),
                                                           std::tuple<>())
            .fallback<HoverState>()
            .start();
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
    // 除 InitState 外均允许 (重入) 起飞: reenter_all 强制完全重入,
    // 即使当前已处于 TakeoffState 也会销毁旧实例、重新构造并执行 on_enter
    ctx.engine->plan().clear();
    ctx.engine->plan()
        .push_back([event](RobotContext&) {
            return StateAction::step<TakeoffState>(std::tuple(event))
                .reenter_all();
        })
        .fallback<HoverState>()
        .start();
}

void ControlEventListener::on_event(const dk::StateChangeEvent& event,
                                    RobotContext& ctx) {
    ctx.dk_state.store(std::string(event.cur));
    spdlog::info("State change {} -> {}", event.prev, event.cur);
    fglog::publish_state("/drone/state", std::string(event.cur));
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
    if (wp_empty && is_ground) {
        // 地面时无法执行任何空航点指令：返航/降落只允许空中执行，
        // 起飞悬停需要明确高度，空列表没有起飞高度可用
        event.reject("航点个数为0，地面无法执行返航、降落或起飞悬停!");
        return;
    }

    // 3. 起飞高度提取 + 多点时删除首点:
    //    首点高度 = 起飞高度; 多点时首点是独立占位 (仅高度, x,y 抛弃), 删除;
    //    单点时该点就是航线点, 保留 (其 z 已提取为起飞高度)
    double takeoff_alt = 0.0;
    if (!wp_empty) {
        takeoff_alt = event.waypoint.at(0).z();
        if (event.waypoint.size() >= 2) {
            auto& ev = const_cast<SetWaypointEvent&>(event);
            ev.waypoint.erase(ev.waypoint.begin());
        }
    }

    // 4. 航点坐标转换 (对"高度 + 完整航线"统一转换, 无需跳过首点):
    //    提前到受理阶段, 失败直接 reject 返回给调用方
    //    (on_enter 里的异常发生在 resolve 之后, 调用方不可见)
    if (!event.local && !event.waypoint.empty()) {
        try {
            auto lon_lat_alt = ctx.lon_lat_alt.load();
            auto pos = ctx.pos_enu.load();
            auto& ev = const_cast<SetWaypointEvent&>(event);
            for (auto& wp : ev.waypoint) {
                wp = state_utils::gps_to_enu(lon_lat_alt, pos, wp);
            }
            ev.local = true;  // 已转为 ENU
        } catch (std::exception& e) {
            event.reject(std::string("航点坐标转换失败: ") + e.what());
            return;
        }
    } else if (event.local && !event.waypoint.empty()) {
        // local 航点 z 统一按 rel_alt 语义解释: 补偿 EKF 原点与 FCU home
        // 的固定偏差 Δ (= pos_enu.z - rel_alt), 使真实高度 = 用户下发值,
        // 与 GPS 航点 (gps_to_enu 后同为 wp.z + Δ) 及起飞段语义一致。
        // SITL 中 Δ=0, 行为不变。GPS 分支的 gps_to_enu 换算已隐含此补偿。
        double delta = ctx.pos_enu.load().z() - ctx.lon_lat_alt.load().z();
        for (auto& wp : const_cast<SetWaypointEvent&>(event).waypoint) {
            wp.z() += delta;
        }
    }

    // 5. RETURN: 直接算出返航点 (任务起点 -> 起飞点 -> 飞控 home),
    //    飞控 home 的 GPS->ENU 转换失败同样 reject 返回给调用方
    if (action == FinishAction::RETURN) {
        auto& ev = const_cast<SetWaypointEvent&>(event);
        // 返航点高度:
        //   有航点 -> 最后一个航点的高度 (已是 EKF 值)
        //   无航点 -> 空中用当前高度 (EKF), 地面用起飞高度 (rel_alt 语义,
        //            需补偿 Δ 转为 EKF, 与 local 航点处理一致)
        double return_z =
            event.waypoint.empty()
                ? (is_ground ? takeoff_alt + ctx.pos_enu.load().z() -
                                   ctx.lon_lat_alt.load().z()
                             : ctx.pos_enu.load().z())
                : event.waypoint.back().z();
        if (auto m = ctx.mission_home_enu.load()) {
            ev.return_enu = *m;
        } else if (auto t = ctx.takeoff_enu.load()) {
            ev.return_enu = *t;
        } else if (auto g = ctx.fcu_home_gps.load()) {
            try {
                ev.return_enu = state_utils::gps_to_enu(ctx.lon_lat_alt.load(),
                                                        ctx.pos_enu.load(), *g);
            } catch (std::exception& e) {
                event.reject(std::string("返航点坐标转换失败: ") + e.what());
                return;
            }
        } else {
            event.reject("无可用的返航点，请使用降落");
            return;
        }
        ev.return_enu->z() = return_z;
    }

    // 6. 状态流转 (on_enter 已不可能失败, resolve 在此之后调用是安全的)
    if (is_ground) {
        auto finish_action = event.finish_action;
        bool do_pland = event.do_pland;

        ctx.engine->plan().clear();
        ctx.engine->plan()
            .push_back<TakeoffState>(std::tuple(takeoff_alt))
            .push_back<WaypointState::LiftingState>(
                std::make_tuple(std::move(event)), std::tuple<>());

        if (finish_action == FinishAction::HOVER) {
            ctx.engine->plan().fallback<HoverState>();
        } else {
            ctx.engine->plan().fallback<LandState>(std::tuple(do_pland));
        }
        ctx.engine->plan().start();
    } else {
        // 空中执行逻辑分流
        if (wp_empty && action == FinishAction::LAND) {
            // 空中原地降落（无航点）：直接进入 LandState
            ctx.engine->step<LandState>(std::tuple(event.do_pland));
        } else {
            auto finish_action = event.finish_action;
            bool do_pland = event.do_pland;

            ctx.engine->plan().clear();
            ctx.engine->plan().push_back<WaypointState::LiftingState>(
                std::make_tuple(std::move(event)), std::tuple<>());

            if (finish_action == FinishAction::HOVER) {
                ctx.engine->plan().fallback<HoverState>();
            } else {
                ctx.engine->plan().fallback<LandState>(std::tuple(do_pland));
            }
            ctx.engine->plan().start();
        }
    }
    // 统一 resolve: reject 路径提前 return 已结算, 重复调用不会改变结果
    event.resolve({"success", "OK"});
}
void ControlEventListener::on_event(const LandEvent& event, RobotContext& ctx) {
    // 降落只允许在空中执行
    if (ctx.engine->is_active_state<InitState>() ||
        ctx.engine->is_active_state<GroundState>()) {
        event.reject("当前在地面，无法降落!");
        return;
    }
    event.resolve({"success", "OK"});
    // 无条件原地降落，不经过航点流程；reenter_all 保证即使已处于降落状态
    // (例如 pland 未生效) 也会退出重进，重新执行 on_enter 断言模式并重启 pland
    ctx.engine->step_reenter_all<LandState>(std::tuple(event.do_pland));
}

void ControlEventListener::on_event(const SetModeEvent& event,
                                    RobotContext& ctx) {
    std::optional<FlightMode> mode;
    try {
        mode = FlightMode::create(event.mode);
    } catch (const std::exception& e) {
        event.reject(e.what());
        return;
    }

    using Promise = dk::Promise<bool>;
    auto p = std::make_shared<Promise>(ctx.engine);

    // 1. 提前获取 Future，避免后续 Promise 被 move 后无法获取
    auto future = p->get_future();

    if (ctx.mode.load() == mode) {
        // 同步完成的情况，直接 resolve，局部变量 p 会随函数结束正常销毁
        p->resolve(true);
    } else {
        ctx.robot->set_mode(mode->mode_raw);

        // 2. 将 promise 移动 (Move) 进 Lambda 中
        // 这样 p 的生命周期就交给了引擎的事件队列，执行完后自动释放
        ctx.engine->wait_for(
            2000, [mode, p](const FlightModeEvent& e) mutable -> bool {
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
            {cur_mission["enu_x"], cur_mission["enu_y"], cur_mission["enu_z"]});
    }
    event.resolve({"success", wp_list});
}

void ControlEventListener::on_event(const GetGpsEvent& event,
                                    RobotContext& ctx) {
    auto gps = ctx.lon_lat_alt.load();
    event.resolve({"success", {gps.x(), gps.y(), gps.z()}});
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
    ctx.mission_home_enu.store(std::nullopt);
    ctx.engine->step<InitState>();
}

void ControlEventListener::on_event(const JoystickEvent& event,
                                    RobotContext& ctx) {
    ctx.robot->cmd_vel(Eigen::Vector4d{event.x, event.y, event.z, event.w});
    event.resolve({"success", "OK"});
}