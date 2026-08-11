#pragma once
#include <Eigen/Dense>
#include <optional>
#include <vector>

enum class CmdFrame {
    BODY,  // 机体坐标系 (x向前, y向左, z向上)
    ENU    // 东北天全局坐标系 (x向东, y向北, z向上)
};

class ITrackerRuntime {
   public:
    virtual ~ITrackerRuntime() = default;
    virtual bool cmd_vel(Eigen::Vector4d) = 0;
};

struct TrackerWaypoint {
    Eigen::Vector3d pos;
    std::optional<double> yaw = std::nullopt;
    std::optional<double> ff_yaw_rate = std::nullopt;
    std::optional<Eigen::Vector3d> ff_vel = std::nullopt;
    std::optional<double> fb_speed_limit_xy = std::nullopt;
    std::optional<double> fb_speed_limit_z = std::nullopt;
    std::optional<double> fb_speed_limit_yaw = std::nullopt;
    CmdFrame frame = CmdFrame::ENU;
    Eigen::Vector3d gamma = {1.0, 1.0, 1.0};
    std::optional<double> max_acc_xy = std::nullopt;
    std::optional<double> max_decel_xy = std::nullopt;
    std::optional<double> max_jerk_xy = std::nullopt;
    std::optional<double> max_jerk_z = std::nullopt;
};

class ITracker {
   public:
    virtual ~ITracker() = default;

    virtual void start(int rate_hz = 50) {}
    virtual void stop() {}

    // 多航点 / 轨迹控制接口
    virtual void send_pos_cmd(const std::vector<TrackerWaypoint>& path) = 0;

    // 单航点控制接口（便利包装，默认打包转发给多航点接口）
    virtual void send_pos_cmd(
        const Eigen::Vector3d& pos, std::optional<double> yaw = std::nullopt,
        std::optional<double> ff_yaw_rate = std::nullopt,
        std::optional<Eigen::Vector3d> ff_vel = std::nullopt,
        std::optional<double> fb_speed_limit_xy = std::nullopt,
        std::optional<double> fb_speed_limit_z = std::nullopt,
        std::optional<double> fb_speed_limit_yaw = std::nullopt,
        CmdFrame frame = CmdFrame::ENU, Eigen::Vector3d gamma = {1.0, 1.0, 1.0},
        std::optional<double> max_acc_xy = std::nullopt,
        std::optional<double> max_decel_xy = std::nullopt,
        std::optional<double> max_jerk_xy = std::nullopt,
        std::optional<double> max_jerk_z = std::nullopt) {
        TrackerWaypoint wp{pos,
                           yaw,
                           ff_yaw_rate,
                           ff_vel,
                           fb_speed_limit_xy,
                           fb_speed_limit_z,
                           fb_speed_limit_yaw,
                           frame,
                           gamma,
                           max_acc_xy,
                           max_decel_xy,
                           max_jerk_xy,
                           max_jerk_z};
        send_pos_cmd(std::vector<TrackerWaypoint>{std::move(wp)});
    }

    // 速度控制
    virtual void send_vel_cmd(const Eigen::Vector3d& vel,
                              std::optional<double> yaw = std::nullopt,
                              std::optional<double> yaw_rate = std::nullopt,
                              CmdFrame frame = CmdFrame::ENU) = 0;

    // 平滑停止 / 刹车
    virtual void send_zero_vel_cmd() = 0;

    // 状态查询
    virtual bool is_position_reached() = 0;
    virtual size_t get_current_waypoint_index() { return 0; }
    virtual size_t get_total_waypoints() { return 0; }
};