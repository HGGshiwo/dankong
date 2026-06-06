#include "states/waypoint_state.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "features/tracker/tracker.hpp"
#include "mavlink/imavlink.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

using josn = nlohmann::json;

WaypointState::WaypointState(SetWaypointEvent e)
    : wp_list_(e.waypoint),
      wp_idx_(0),
      action_(e.finish_action),
      node_event_list_(e.nodeEventList),
      land_target_id_(e.land_target_id),
      target_vel_(e.speed) {}

void WaypointState::WaypointState::on_enter(RobotContext& ctx) {
    // 现在一定是起飞中了

    // return 转为land情况
    if (action_ == FinishAction::RETURN) {
        wp_list_.push_back(ctx.takeoff_lon_lat_alt.load());
    }
    if (wp_list_.size() == 0) {
        // 直接执行剩余命令
        if (action_ == FinishAction::LAND) {
            ctx.engine->step<LandState>(std::tuple(land_target_id_));
        } else if (action_ == FinishAction::HOVER) {
            ctx.engine->step<HoverState>();
        } else {
            // RETURN至少会有一个点
            throw std::logic_error("invalid finish action!");
        }
        return;
    }
    if (wp_list_.size() == 1) {
        // 偷懒没有传第一个点
        wp_list_.insert(wp_list_.begin(), ctx.takeoff_lon_lat_alt.load());
    }

    // 计算真实的航点
    std::vector<json> wp_data;
    wp_data.reserve(wp_list_.size());

    for (uint i = 0; i < wp_list_.size(); ++i) {
        std::string command = "wp";
        if (i == 0) {
            command = "takeoff";
        } else if (i == wp_list_.size() - 1 && action_ != FinishAction::HOVER) {
            command = "land";
        }
        auto cur_wp = wp_list_[i];
        wp_data.push_back(json{{"num", i},
                               {"lon", cur_wp.x()},
                               {"lat", cur_wp.y()},
                               {"alt", cur_wp.z()},
                               {"command", command}});
    }
    ctx.mission_data.store(wp_data);
    wp_list_.erase(wp_list_.begin());
    ctx.wp_idx.store(0);
}

void WaypointState::WaypointState::on_exit(RobotContext& ctx) {
    // 清空数据
    ctx.dist_to_target.store(0.0);
}

Eigen::Vector3d WaypointState::get_cur_wp() {
    return wp_list_.at(wp_idx_);
}

inline double MAX_Z_VEL = 1.0;
double YAW_STALL_THRESH = 0.1;
double ALT_STALL_THRESH = 0.1;
inline double CLOSE_THRESH = 0.1;  // 两个点足够近视为同一个点
double YAW_TOLERANCE = 0.1;
double ALT_TOLERANCE = 2.0;
double STALL_TIMEOUT = 3.0;

WaypointState::LiftingState::LiftingState()
    : start_time_(0.0), checker_(nullptr) {};

void WaypointState::LiftingState::on_enter(RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    start_time_ = now;
    checker_ = std::make_shared<state_utils::StallChecker<2>>(
        std::array<double, 2>{YAW_STALL_THRESH, ALT_STALL_THRESH},
        STALL_TIMEOUT, now);

    auto cur_wp = parent()->get_cur_wp();
    auto diff = state_utils::get_relevant_enu(ctx.lon_lat_alt.load(), cur_wp);
    target_alt_ = cur_wp.z();
    target_yaw_ = diff.head<2>().norm() < CLOSE_THRESH
                      ? ctx.yaw_enu.load()
                      : state_utils::get_heading(diff.x(), diff.y());
}

StateAction WaypointState::LiftingState::on_event(const dk::TickEvent& e,
                                                  RobotContext& ctx) {
    // 达到目标的高度和航向退出
    double yaw_diff = state_utils::get_yaw_diff(target_yaw_, ctx.yaw_enu);
    double alt_diff = fabs(target_alt_ - ctx.lon_lat_alt.load().z());

    if (yaw_diff < YAW_TOLERANCE &&
        (!ctx.robot->is_alt_enable() || alt_diff < ALT_TOLERANCE)) {
        ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                  std::nullopt, CmdFrame::ENU);
        return step<WaypointState::ExcuteState>();
    }

    double now = ctx.engine->get_time_provider()->now();
    if (checker_->is_stall({ctx.yaw_enu, ctx.lon_lat_alt.load().z()}, now)) {
        return step<WaypointState::ExcuteState>();
    }

    double vz = std::clamp(target_alt_ - ctx.lon_lat_alt.load().z(), -MAX_Z_VEL,
                           MAX_Z_VEL);
    ctx.tracker->send_vel_cmd(Eigen::Vector3d{0.0, 0.0, vz}, target_yaw_,
                              std::nullopt, CmdFrame::ENU);
    return StateAction::unhandled();
}

void WaypointState::LiftingState::on_exit(RobotContext& ctx) {
    ctx.tracker->send_vel_cmd(Eigen::Vector3d{0.0, 0.0, 0.0}, std::nullopt,
                              std::nullopt, CmdFrame::ENU);
}

//============= excute_state =============

double TOLERANCE = 1.2;
bool WaypointState::ExcuteState::check_arrive(RobotContext& ctx) {
    if (!ctx.odom_ok) return false;
    double dist = ctx.robot->get_distance(ctx.pos_enu.load(), wp_goal_);
    ctx.dist_to_target.store(std::round(dist * 100.0) / 100.0);
    return dist < TOLERANCE;
}

// 执行当前wp_idx指向的航点
void WaypointState::ExcuteState::on_enter(RobotContext& ctx) {
    auto wp = parent()->get_cur_wp();
    wp_goal_ =
        state_utils::gps_to_enu(ctx.lon_lat_alt.load(), ctx.pos_enu.load(), wp);

    if (!ctx.planner_enable.load()) {
        ctx.tracker->send_pos_cmd(wp_goal_, std::nullopt, std::nullopt,
                                  std::nullopt, parent()->target_vel_,
                                  std::nullopt, CmdFrame::ENU);
    }

    if (parent()->node_event_list_.has_value()) {
        auto events = parent()->node_event_list_.value().at(parent()->wp_idx_);
        spdlog::info("run event: {}", events.dump());
        if (!events.is_null()) {
            for (const auto& event : events) {
                run_wp_event(ctx, event);
            }
        }
    } else {
        spdlog::info("no event for wp_idx: {}!", parent()->wp_idx_);
    }
}

StateAction WaypointState::ExcuteState::on_event(const dk::TickEvent& event,
                                                 RobotContext& ctx) {
    // 如果到达了目标点
    auto arrive = check_arrive(ctx);
    if (arrive) {
        parent()->wp_idx_ += 1;
        ctx.wp_idx.store(parent()->wp_idx_);

        // 发布数据
        json j = json{{"total", parent()->wp_list_.size()},
                      {"cur", ctx.wp_idx.load()},
                      {"type", "event"},
                      {"event", "progress"}};
        ctx.ws_manager->publish_state("progress", j);

        if (parent()->wp_idx_ >= parent()->wp_list_.size()) {
            parent()->report_task_done(ctx);
            if (parent()->action_ != FinishAction::HOVER)
                return step<LandState>(std::tuple(parent()->land_target_id_));
            else
                return step<HoverState>();
        }
        return step<WaypointState::LiftingState>();
    }
    return StateAction::unhandled();
}

void WaypointState::ExcuteState::run_wp_event(RobotContext& ctx,
                                              const nlohmann::json& j) {
    // 定义处理函数的类型
    using EventHandler =
        std::function<void(RobotContext&, const nlohmann::json&)>;

    // 静态注册表，只在第一次调用时初始化，开销极小
    static const std::unordered_map<std::string, EventHandler> event_handlers =
        {{"video:on",
          [](RobotContext& c, const nlohmann::json& j) {
              StartRecordEvent e;
              e.bag_name = j.value("eventParam", "");
              c.engine->dispatch(e);
          }},
         {"video:off",
          [](RobotContext& c, const nlohmann::json& j) {
              StopRecordEvent e;
              c.engine->dispatch(e);
          }},
         {"hat:on",
          [](RobotContext& c, const nlohmann::json& j) {
              EnableDetectEvent e;
              e.type = "hat";
              c.engine->dispatch(e);
          }},
         {"hat:off",
          [](RobotContext& c, const nlohmann::json& j) {
              DisableDetectEvent e;
              c.engine->dispatch(e);
          }},
         {"smoke:on",
          [](RobotContext& c, const nlohmann::json& j) {
              EnableDetectEvent e;
              e.type = "smoke";
              c.engine->dispatch(e);
          }},
         {"gimbal:", [](RobotContext& c, const nlohmann::json& j) {
              double angle_d = std::nanf("");
              std::string angle = j.value("eventParam", "25");
              try {
                  angle_d = std::stof(angle);
              } catch (const std::exception& e) {
                  spdlog::error("run event error, event={} error={}", j.dump(),
                                e.what());
                  return;
              }
              SetGimbalEvent e;
              e.angle = angle_d;
              e.mode = "body";
              c.engine->dispatch(e);
          }}};
    // 使用 value() 提供默认值，防止 JSON 缺少字段导致抛出异常
    std::string event_type = j.value("eventType", "");
    std::string status = j.value("eventStatus", "");

    // 组合 Key
    std::string handler_key = event_type + ":" + status;
    // 查表并执行
    auto it = event_handlers.find(handler_key);
    if (it != event_handlers.end()) {
        it->second(ctx, j);
        spdlog::info("dispatch: {}!", j.dump());
    } else {
        // 可选：记录日志，提示未知的事件类型或状态
        spdlog::error("Unknow event: {}", handler_key);
    }
}