#pragma once
#include <array>
#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <tuple>

#include "../robot_context.hpp"
#include "../robot_event.hpp"
#include "../states/hover_state.hpp"
#include "../states/init_state.hpp"
#include "../states/land_state.hpp"
#include "../states/posvel_state.hpp"
#include "../states/state_utils.hpp"
#include "../states/takeoff_state.hpp"
#include "../states/waypoint_state.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "dk/adapters/web/protocal.hpp"
#include "nlohmann/json.hpp"
#include "spdlog/spdlog.h"
#include "states/ground_state.hpp"

inline const int FCU_DATA_RATE = 10;

// 和控制相关的事件监听器
class ControlEventListener
    : public dk::BaseEventListener<RobotContext, ControlEventListener> {
   public:
    using AllowedEvents =
        std::tuple<PrearmEvent, TakeoffEvent, dk::StateChangeEvent,
                   SetWaypointEvent, SetModeEvent, SetPosVelEvent,
                   dk::TickEvent, RebootFcuEvent, GetWpEvent, GetGpsEvent,
                   StopFollowEvent, EnablePlandEvent, DisablePlandEvent,
                   EnablePlannerEvent, DisablePlannerEvent, GetParamEvent,
                   SetParamEvent, FcuConnectedEvent, DisarmEvent, RestartEvent>;

    void on_event(dk::TickEvent event, RobotContext& ctx) {
        bool on_ground = ctx.engine->is_active_state<GroundState>() ||
                         ctx.engine->is_active_state<InitState>() ||
                         ctx.engine->is_active_state<TakeoffState>() ||
                         ctx.engine->is_active_state<LandState>();
        if (!on_ground && !ctx.robot->check_hover(ctx)) {
            ctx.engine->template step<GroundState>();
        }
    }

    void on_event(SetPosVelEvent event, RobotContext& ctx) {
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
            event.reject(
                "set_posvel只允许在waypoint或者hover状态调用! 当前状态: " +
                ctx.engine->get_state_name());
            return;
        }
    }

    void on_event(PrearmEvent event, RobotContext& ctx) {
        state_utils::prearm_check(ctx)
            .then([event](bool res) {
                if (res)
                    event.resolve({"success", nlohmann::json{{"arm", true}}});
                event.resolve(
                    {"success", nlohmann::json{{"arm", false},
                                               {"reason", "Unknown Error"}}});
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

    void on_event(TakeoffEvent event, RobotContext& ctx) {
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

    void on_event(dk::StateChangeEvent event, RobotContext& ctx) {
        spdlog::info("State change {} -> {}", event.prev, event.cur);
    }

    void on_event(SetWaypointEvent event, RobotContext& ctx) {
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
            if (is_ground && (action == FinishAction::RETURN ||
                              action == FinishAction::LAND)) {
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
                ctx.engine->step<LandState>(std::tuple(event.land_target_id));
            } else {
                // 检查是不是半路进入hover，其实没有记录起飞点
                auto takeoff_lon_lat_alt = ctx.takeoff_lon_lat_alt.get();
                if (action == FinishAction::RETURN &&
                    takeoff_lon_lat_alt.norm() < 1.0) {
                    event.reject("未记录起飞点，请使用降落");
                } else {
                    event.resolve({"success", "OK"});
                    // 其他情况（有航点，或空中原地悬停/返航）：进入
                    // LiftingState 执行航点处理
                    ctx.engine->step<WaypointState::LiftingState>(
                        std::make_tuple(std::move(event)), std::tuple<>());
                }
            }
        }
    }

    void on_event(const SetModeEvent& event, RobotContext& ctx) {
        state_utils::set_mode(ctx, event.mode)
            .then([event](bool res) {  // 这里还是值的拷贝
                if (res)
                    event.resolve({"success", "OK"});
                else
                    event.reject("set mode error");
            })
            .catch_error([event](std::exception_ptr e) { event.reject(e); });
    }

    void on_event(const RebootFcuEvent& event, RobotContext& ctx) {
        auto res = ctx.robot->reboot_fcu();
        if (res)
            event.resolve({"success", "OK"});
        else
            event.reject("reboot fcu error");
    }

    void on_event(const GetWpEvent& event, RobotContext& ctx) {
        auto mission_data = ctx.mission_data.get();
        std::vector<Eigen::Vector3d> wp_list(mission_data.size());
        for (int i = 0; i < mission_data.size(); ++i) {
            auto cur_mission = mission_data[i];
            wp_list[i] = Eigen::Vector3d{cur_mission["lon"], cur_mission["lat"],
                                         cur_mission["alt"]};
        }
        event.resolve({"success", wp_list});
    }

    void on_event(const GetGpsEvent& event, RobotContext& ctx) {
        auto gps = ctx.lon_lat_alt.get();
        event.resolve({"success", {gps.y(), gps.x(), gps.z()}});
    }

    void on_event(const StopFollowEvent& event, RobotContext& ctx) {
        auto cur_name = ctx.engine->get_state_name();
        if (cur_name != "跟随模式") {
            event.reject("非跟随模式调用无效! 当前状态：" + cur_name);
            return;
        }
        event.resolve({"success", "OK"});
        ctx.stop_follow_stamp.set(std::chrono::steady_clock::now());
        if (ctx.engine->is_active_state<HoverState>()) {
            ctx.engine->step<HoverState>();
        } else if (ctx.engine->is_active_state<WaypointState>()) {
            ctx.engine->step<WaypointState::LiftingState>();
        } else {
            spdlog::warn("unknow state: {}, switch to hover", cur_name);
            ctx.engine->step<HoverState>();
        }
    }

    void on_event(const EnablePlandEvent& event, RobotContext& ctx) {
        ctx.pland_enable.set(true);
        event.resolve({"success", "OK"});
    }

    void on_event(const DisablePlandEvent& event, RobotContext& ctx) {
        ctx.pland_enable.set(false);
        event.resolve({"success", "OK"});
    }

    void on_event(const EnablePlannerEvent& event, RobotContext& ctx) {
        ctx.planner_enable.set(true);
        event.resolve({"success", "OK"});
    }

    void on_event(const DisablePlannerEvent& event, RobotContext& ctx) {
        ctx.planner_enable.set(false);
        event.resolve({"success", "OK"});
    }

    void on_event(const FcuConnectedEvent& event, RobotContext& ctx) {
        if (event.connected) {
            ctx.engine->post_background_task<bool>([robot = ctx.robot]() {
                robot->pull_params();
                return true;
            });
            ctx.engine->get_context().robot->set_stream_rate(FCU_DATA_RATE);
        }
    }

    void on_event(const GetParamEvent& event, RobotContext& ctx) {
        ctx.engine->post_background_task<bool>([robot = ctx.robot, event]() {
            nlohmann::json j;
            j["param"]["_value"] = robot->get_all_params();
            event.resolve({"success", j});
            return true;
        });
    }

    void on_event(const SetParamEvent& event, RobotContext& ctx) {
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

    // 简略实现一下，实际不提供这个接口
    void on_event(const DisarmEvent& event, RobotContext& ctx) {
        ctx.robot->disarm();
        event.resolve({"success", "OK"});
    }

    void on_event(const RestartEvent& event, RobotContext& ctx) {
        ctx.odom_ok = false;
        ctx.fcu_connected.set(false);
        ctx.engine->step<InitState>();
    }
};
