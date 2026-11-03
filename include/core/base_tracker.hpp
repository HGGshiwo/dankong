#pragma once
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <string>
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

// 航点巡航限速回调的 RAII 句柄：析构时自动向 tracker 注销该回调
// (定义在 ITracker 之后, 成员函数需要完整类型)
struct SpeedLimitToken;

// tracker 状态快照, 供外部周期日志/监控查询
struct TrackerStatusInfo {
    std::string mode;  // "POSITION" / "VELOCITY" / "NONE"
    size_t wp_idx = 0;
    size_t wp_total = 0;
    Eigen::Vector4d target_pose =
        Eigen::Vector4d::Zero();  // 当前目标点 (ENU xyz + yaw)
    Eigen::Vector4d target_vel =
        Eigen::Vector4d::Zero();  // 期望速度 (前馈 xyz + yaw_rate)
    Eigen::Vector4d cmd_vel =
        Eigen::Vector4d::Zero();  // 上一拍最终下发指令 (PID+限幅后)
    double eff_xy = 0;  // 当前生效限速 (航点/配置/回调取最小)
    double eff_z = 0;
    double eff_yaw = 0;
    double max_vel_xy = 0;  // 配置的硬上限
    std::string sources;  // 生效中的限速回调名, 逗号分隔 (如 "detect")
};

class ITracker {
   public:
    virtual ~ITracker() = default;

    virtual void start(int rate_hz = 50) {}
    virtual void stop() {}

    // 注册航点巡航限速回调，返回句柄 id；回调返回 nullopt 表示不限制，
    // tracker 取所有回调限速与航线速度的最小值生效
    virtual int add_speed_limit_cb(
        std::string /*name*/, std::function<std::optional<double>()> /*cb*/) {
        return -1;
    }
    virtual void remove_speed_limit_cb(int /*id*/) {}

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
    // tracker 状态快照 (默认实现返回空对象)
    virtual TrackerStatusInfo get_status_info() const { return {}; }
};

struct SpeedLimitToken {
    int id = -1;
    ITracker* owner = nullptr;

    SpeedLimitToken() = default;
    SpeedLimitToken(const SpeedLimitToken&) = delete;
    SpeedLimitToken& operator=(const SpeedLimitToken&) = delete;
    SpeedLimitToken(SpeedLimitToken&& other) noexcept
        : id(other.id), owner(other.owner) {
        other.id = -1;
        other.owner = nullptr;
    }
    SpeedLimitToken& operator=(SpeedLimitToken&& other) noexcept {
        if (this != &other) {
            release();
            id = other.id;
            owner = other.owner;
            other.id = -1;
            other.owner = nullptr;
        }
        return *this;
    }
    ~SpeedLimitToken() { release(); }

    void release() {
        if (owner && id >= 0) {
            owner->remove_speed_limit_cb(id);
        }
        id = -1;
        owner = nullptr;
    }
};