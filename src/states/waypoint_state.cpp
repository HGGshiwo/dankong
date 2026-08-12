#include "states/waypoint_state.hpp"

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "features/report/event_listener.hpp"
#include "features/tracker/tracker.hpp"
#include "mavlink/imavlink.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"
#include "states/arm_state.hpp"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/state_common.hpp"
#include "states/state_utils.hpp"

using josn = nlohmann::json;

WaypointState::WaypointState(SetWaypointEvent e)
    : event_(e),
      wp_list_(e.waypoint),
      wp_idx_(0),
      action_(e.finish_action),
      node_event_list_(e.nodeEventList),
      do_pland_(e.do_pland),
      skip_lifting_(e.skip_lifting),
      target_vel_(e.speed) {}

StateAction WaypointState::before_enter(RobotContext& ctx,
                                        const SetWaypointEvent& e) {
    if (!ctx.arm.load()) {
        StateFlags flags{.is_waypoint = true, .local = e.local};
        if (ctx.robot->should_arm_before_enter(flags)) {
            return StateAction::plan([](auto& plan, auto& /*ctx*/) {
                plan.template push_front<ArmState>();
            });
        }
    }
    return StateAction::handled();
}

namespace {
constexpr double CLOSE_THRESH = 0.1;  // 两个点足够近视为同一个点
constexpr double TURN_ANGLE_THRESH =
    60.0 * M_PI / 180.0;  // 航向夹角大于60度停下来调整朝向
}  // namespace

StateAction WaypointState::on_enter(RobotContext& ctx) {
    // 直接全部转为局部坐标
    if (!event_.local) {
        Eigen::Vector3d lon_lat_alt = ctx.lon_lat_alt.load();
        Eigen::Vector3d pos_enu = ctx.pos_enu.load();
        for (auto& wp : wp_list_) {
            wp = state_utils::gps_to_enu(lon_lat_alt, pos_enu, wp);
        }
    }

    // 初始化暂留点标记列表
    is_dwell_point_.assign(event_.waypoint.size(), !skip_lifting_);
    if (skip_lifting_) {
        for (int idx : event_.dwell_indices) {
            if (idx >= 0 && idx < (int)is_dwell_point_.size()) {
                is_dwell_point_[idx] = true;
            }
        }
    }

    // 保证一定是有局部坐标的
    Eigen::Vector3d takeoff_pos = ctx.takeoff_enu.load().value();

    // return 转为land情况
    if (action_ == FinishAction::RETURN) {
        wp_list_.push_back(takeoff_pos);
        is_dwell_point_.push_back(true);
    }
    if (wp_list_.size() == 0) {
        // 直接执行剩余命令
        if (action_ == FinishAction::LAND) {
            return StateAction::step<LandState>(std::tuple(do_pland_));
        } else if (action_ == FinishAction::HOVER) {
            return StateAction::step<HoverState>();
        } else {
            // RETURN至少会有一个点
            throw std::logic_error("invalid finish action!");
        }
        return StateAction::unhandled();
    }
    if (wp_list_.size() == 1) {
        // 偷懒没有传第一个点
        wp_list_.insert(wp_list_.begin(), takeoff_pos);
        is_dwell_point_.insert(is_dwell_point_.begin(), true);
    }

    // 相邻航段夹角大于阈值时（>60度），自动标记为暂留点，先停下来调整朝向和高度再飞下一段
    for (size_t i = 1; i + 1 < wp_list_.size(); ++i) {
        Eigen::Vector2d v1 = (wp_list_[i] - wp_list_[i - 1]).head<2>();
        Eigen::Vector2d v2 = (wp_list_[i + 1] - wp_list_[i]).head<2>();
        if (v1.norm() > CLOSE_THRESH && v2.norm() > CLOSE_THRESH) {
            double cos_theta = v1.dot(v2) / (v1.norm() * v2.norm());
            cos_theta = std::clamp(cos_theta, -1.0, 1.0);
            double turn_angle = std::acos(cos_theta);
            if (turn_angle > TURN_ANGLE_THRESH) {
                is_dwell_point_[i] = true;
                spdlog::info(
                    "[WaypointState] Waypoint {} turn angle {:.1f} deg > "
                    "{:.1f} deg, marked as dwell point",
                    i, turn_angle * 180.0 / M_PI,
                    TURN_ANGLE_THRESH * 180.0 / M_PI);
            }
        }
    }

    // 确保最后一个点一定是暂留点
    if (!is_dwell_point_.empty()) {
        is_dwell_point_.back() = true;
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
                               {"enu_x", cur_wp.x()},
                               {"enu_y", cur_wp.y()},
                               {"enu_z", cur_wp.z()},
                               {"command", command}});
    }
    ctx.mission_data.store(wp_data);
    wp_list_.erase(wp_list_.begin());
    is_dwell_point_.erase(is_dwell_point_.begin());
    ctx.wp_idx.store(0);
    return StateAction::unhandled();
}

void WaypointState::on_exit(RobotContext& ctx) {
    // 清空数据
    ctx.dist_to_target.store(0.0);
}

Eigen::Vector3d WaypointState::get_cur_wp() {
    return wp_list_.at(wp_idx_);
}

inline double MAX_Z_VEL = 1.0;
double YAW_STALL_THRESH = 0.1;
double ALT_STALL_THRESH = 0.1;
double YAW_TOLERANCE = 0.1;
double ALT_TOLERANCE = 2.0;
double STALL_TIMEOUT = 3.0;

WaypointState::LiftingState::LiftingState()
    : start_time_(0.0), checker_(nullptr) {};

StateAction WaypointState::LiftingState::on_enter(RobotContext& ctx) {
    double now = ctx.engine->get_time_provider()->now();
    start_time_ = now;
    checker_ = std::make_shared<state_utils::StallChecker<2>>(
        std::array<double, 2>{YAW_STALL_THRESH, ALT_STALL_THRESH},
        STALL_TIMEOUT, now);

    auto cur_wp = parent()->get_cur_wp();
    auto diff = cur_wp - ctx.pos_enu.load();
    target_alt_ = cur_wp.z();
    target_yaw_ = diff.head<2>().norm() < CLOSE_THRESH
                      ? ctx.yaw_enu.load()
                      : state_utils::get_heading(diff.x(), diff.y());
    return StateAction::unhandled();
}

StateAction WaypointState::LiftingState::on_tick(double dt, RobotContext& ctx) {
    // 达到目标的高度和航向退出
    double yaw_diff = state_utils::get_yaw_diff(target_yaw_, ctx.yaw_enu);
    double alt_diff = fabs(target_alt_ - ctx.pos_enu.load().z());

    if (yaw_diff < YAW_TOLERANCE &&
        (!ctx.robot->is_alt_enable() || alt_diff < ALT_TOLERANCE)) {
        ctx.tracker->send_vel_cmd(Eigen::Vector3d::Zero(), std::nullopt,
                                  std::nullopt, CmdFrame::ENU);
        return step<WaypointState::ExcuteState>();
    }

    double now = ctx.engine->get_time_provider()->now();
    if (checker_->is_stall({ctx.yaw_enu, ctx.pos_enu.load().z()}, now)) {
        return step<WaypointState::ExcuteState>();
    }

    double vz =
        std::clamp(target_alt_ - ctx.pos_enu.load().z(), -MAX_Z_VEL, MAX_Z_VEL);
    ctx.tracker->send_vel_cmd(Eigen::Vector3d{0.0, 0.0, vz}, target_yaw_,
                              std::nullopt, CmdFrame::ENU);
    return StateAction::unhandled();
}

void WaypointState::LiftingState::on_exit(RobotContext& ctx) {
    ctx.tracker->send_vel_cmd(Eigen::Vector3d{0.0, 0.0, 0.0}, std::nullopt,
                              std::nullopt, CmdFrame::ENU);
}

//============= excute_state =============
bool WaypointState::ExcuteState::check_arrive(RobotContext& ctx) {
    if (!ctx.odom_ok) return false;
    double dist = ctx.robot->get_distance(ctx.pos_enu.load(), wp_goal_);
    ctx.dist_to_target.store(std::round(dist * 100.0) / 100.0);

    if (parent()->wp_idx_ < (int)parent()->is_dwell_point_.size() &&
        parent()->is_dwell_point_[parent()->wp_idx_]) {
        return dist < GlobalConfig.GetConfig().waypoint_tolerance.get();
    } else {
        return dist < GlobalConfig.GetConfig().non_dwell_tolerance.get();
    }
}

// 执行当前wp_idx指向的航点
StateAction WaypointState::ExcuteState::on_enter(RobotContext& ctx) {
    auto wp = parent()->get_cur_wp();
    Eigen::Vector3d pos_enu = ctx.pos_enu.load();
    wp_goal_ = wp;

    std::vector<TrackerWaypoint> path;
    for (size_t j = parent()->wp_idx_; j < parent()->wp_list_.size(); ++j) {
        TrackerWaypoint tracker_wp;
        tracker_wp.pos = parent()->wp_list_[j];
        tracker_wp.frame = CmdFrame::ENU;
        tracker_wp.fb_speed_limit_xy = parent()->target_vel_;

        path.push_back(std::move(tracker_wp));

        // 如果遇到了暂留点，说明是一段轨迹的结尾，切断并停止添加后续点
        if (j < parent()->is_dwell_point_.size() &&
            parent()->is_dwell_point_[j]) {
            break;
        }
    }
    ctx.tracker->send_pos_cmd(path);

    if (parent()->node_event_list_.has_value()) {
        auto events = parent()->node_event_list_.value().at(parent()->wp_idx_);
        spdlog::info("[WaypointEvent] run event for wp_idx: {}: {}",
                     parent()->wp_idx_, events.dump());
        if (!events.is_null()) {
            for (const auto& event : events) {
                run_wp_event(ctx, event);
            }
        }
    } else {
        spdlog::info("[WaypointEvent] no event for wp_idx: {}!",
                     parent()->wp_idx_);
    }

    return StateAction::unhandled();
}

StateAction WaypointState::ExcuteState::on_arrive(RobotContext& ctx) {
    parent()->wp_idx_ += 1;
    ctx.wp_idx.store(parent()->wp_idx_);
    ctx.engine->dispatch(TaskProgressEvent{
        ctx.wp_idx.load(), static_cast<int>(parent()->wp_list_.size())});

    if (parent()->wp_idx_ >= parent()->wp_list_.size()) {
        ctx.engine->dispatch(TaskDoneEvent{});
        return StateAction::next();
    }

    // 如果刚到达的这个点是暂留点，进入 LiftingState
    // 调整姿态，否则直接进入下一航点的 ExcuteState
    if (parent()->wp_idx_ - 1 < (int)parent()->is_dwell_point_.size() &&
        parent()->is_dwell_point_[parent()->wp_idx_ - 1]) {
        return step<WaypointState::LiftingState>();
    } else {
        return step<WaypointState::ExcuteState>()
            .reenter_from<WaypointState::ExcuteState>();
    }
}

StateAction WaypointState::ExcuteState::on_tick(double dt, RobotContext& ctx) {
    // 如果到达了目标点
    auto arrive = check_arrive(ctx);
    if (arrive) {
        return on_arrive(ctx);
    }
    return StateAction::unhandled();
}

StateAction WaypointState::ExcuteState::on_event(const WpArriveEvent& event,
                                                 RobotContext& ctx) {
    // 这个会和waypoint自带的arrive检查冲突，考虑更加优雅的实现
    // return on_arrive(ctx);
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