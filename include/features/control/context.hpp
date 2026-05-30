#pragma once
#include <ros/ros.h>

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>

#include "Eigen/src/Geometry/Quaternion.h"
#include "utils/dirty_var.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/state_registry.hpp"

struct DronePoseRecord {
    ros::Time stamp;
    Eigen::Vector3d pos_enu;
    Eigen::Quaterniond q;
};

class PoseHistory {
   private:
    std::deque<DronePoseRecord> buffer_;
    std::mutex mtx_;
    const double MAX_HISTORY_SEC = 2.0;

   public:
    void push(ros::Time t, const Eigen::Vector3d& p,
              const Eigen::Quaterniond& q) {
        std::lock_guard<std::mutex> lk(mtx_);
        buffer_.push_back({t, p, q});

        // 维持最大时长
        while (buffer_.size() > 2 &&
               (t - buffer_.front().stamp).toSec() > MAX_HISTORY_SEC) {
            buffer_.pop_front();
        }
    }

    // 查找特定时间戳的位姿（简单找最近的，工业级可改为线性插值）
    bool get_pose_at(ros::Time t, Eigen::Vector3d& out_p,
                     Eigen::Quaterniond& out_q) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buffer_.empty()) return false;

        auto best_it = buffer_.begin();
        double min_diff = std::abs((best_it->stamp - t).toSec());

        for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
            double diff = std::abs((it->stamp - t).toSec());
            if (diff < min_diff) {
                min_diff = diff;
                best_it = it;
            }
        }

        // 如果时间差异超过 0.15 秒，说明历史记录丢失或不同步
        if (min_diff > 0.15) return false;

        out_p = best_it->pos_enu;
        out_q = best_it->q;
        return true;
    }
};

struct ControlContext {
    // =========================================================================
    // 纯净的数据载体 (Data Model)
    // 仅负责存储数据和线程安全，不包含任何上报逻辑、JSON 键名和频率限制
    // =========================================================================

    DirtyVar<bool> arm{false};
    DirtyVar<bool> fcu_connected{false};
    DirtyVar<int> gps_fix_type{0};
    DirtyVar<int> gps_nsats{0};
    DirtyVar<double> battery_remaining{-1.0};
    DirtyVar<double> battery_level{-1.0};

    DirtyVar<FixedString64> mode{FixedString64("未知飞控模式")};
    DirtyVar<double> dist_to_target{0.0};  // 航点到下一个目标点的距离
    DirtyVar<double> yaw_diff{0.0};        // 偏航误差
    DirtyVar<int> wp_idx{0};               // 当前执行的航点序号

    DirtyVar<Eigen::Vector3d> pos_enu{Eigen::Vector3d::Zero()};
    DirtyVar<Eigen::Vector3d> takeoff_lon_lat_alt{Eigen::Vector3d::Zero()};

    DirtyVar<double> yaw_ned{0.0};
    DirtyVar<nlohmann::json> mission_data{nlohmann::json::array()};

    DirtyVar<Eigen::Vector3d> lon_lat_alt{Eigen::Vector3d::Zero()};
    DirtyVar<Eigen::Vector3d> vel_enu{Eigen::Vector3d::Zero()};
    DirtyVar<Eigen::Vector3d> vel_body{Eigen::Vector3d::Zero()};

    // 统一替换为你新实现的 DirtyVar (原代码中似乎叫 dirty)
    DirtyVar<std::chrono::steady_clock::time_point> stop_follow_stamp;

    DirtyVar<Eigen::Quaterniond> orientation;

    std::atomic<bool> odom_ok{false};
    std::atomic<double> current_battery{-1.0};
    std::atomic<double> voltage_battery{-1.0};

    std::atomic<int> sensor_health{0};
    std::atomic<double> yaw_enu{0.0};
    DirtyVar<double> roll;
    DirtyVar<double> pitch;

    PoseHistory pose_history;

   public:
    // =========================================================================
    // 上报注册 (Telemetry Binding)
    // 构造时由外部传入 Registry，一次性完成绑定映射。结构体本身不再持有 reg
    // 的引用
    // =========================================================================
    explicit ControlContext(StateRegistry& reg) {
        // --- 1. 标准 JSON 键值对绑定 ---
        reg.bind("arm", arm, 5.0);
        reg.bind("connected", fcu_connected, 2.0);
        reg.bind("gps_fix_type", gps_fix_type, 1.0);
        reg.bind("gps_nsats", gps_nsats, 1.0);
        reg.bind("battery_remaining", battery_remaining, 1.0);
        reg.bind("battery_level", battery_level, 1.0);
        reg.bind("mode", mode, 5.0);
        reg.bind("dist", dist_to_target, 5.0);
        reg.bind("yaw_diff", yaw_diff, 5.0);
        reg.bind("wp_idx", wp_idx, 5.0);

        reg.bind("pos_enu", pos_enu, 0.0);
        reg.bind("takeoff_lon_lat_alt", takeoff_lon_lat_alt, 0.0);
        reg.bind("yaw", yaw_ned, 10.0);
        reg.bind("pitch", pitch, 10.0);
        reg.bind("roll", roll, 10.0);
        reg.bind("mission_data", mission_data, 1.0);

        // --- 2. 复合类型的自定义展开绑定 ---
        reg.bind_custom(lon_lat_alt, 10.0,
                        [](nlohmann::json& j, const Eigen::Vector3d& data) {
                            j["lon"] = data.x();
                            j["lat"] = data.y();
                            j["rel_alt"] = data.z();
                        });

        reg.bind_custom(vel_enu, 10.0,
                        [](nlohmann::json& j, const Eigen::Vector3d& data) {
                            j["x_vel"] = data.x();
                            j["y_vel"] = data.y();
                        });

        reg.bind_custom(vel_body, 10.0,
                        [](nlohmann::json& j, const Eigen::Vector3d& data) {
                            j["x_vel_body"] = data.x();
                            j["y_vel_body"] = data.y();
                        });
    }
};