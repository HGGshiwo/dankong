#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>

#include "Eigen/src/Geometry/Quaternion.h"
#include "utils/datum_synchronizer.hpp"
#include "utils/dirty_var.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/state_registry.hpp"

struct DronePosRecord {
    double stamp;
    Eigen::Vector3d pos_enu;
};

struct DroneQuatRecord {
    double stamp;
    Eigen::Quaterniond q;
};

struct ScalarRecord {
    double stamp;
    double value;
};

class PoseHistory {
   private:
    std::deque<DronePosRecord> pos_buffer_;
    std::deque<DroneQuatRecord> quat_buffer_;
    std::deque<ScalarRecord> roll_buf_;
    std::deque<ScalarRecord> pitch_buf_;
    std::deque<ScalarRecord> yaw_buf_;
    std::mutex mtx_;
    const double MAX_HISTORY_SEC = 2.0;
    const double MAX_TOLERANCE_SEC = 0.15;  // 允许的最大外推/丢失时间差

    void push_scalar(std::deque<ScalarRecord>& buf, double t, double val) {
        if (!buf.empty() && t <= buf.back().stamp) {
            return;
        }
        buf.push_back({t, val});
        while (buf.size() > 2 && (t - buf.front().stamp) > MAX_HISTORY_SEC) {
            buf.pop_front();
        }
    }

    bool get_scalar_at(const std::deque<ScalarRecord>& buf, double t,
                       double& out) {
        if (buf.empty()) return false;
        if (t <= buf.front().stamp) {
            if (buf.front().stamp - t > MAX_TOLERANCE_SEC) return false;
            out = buf.front().value;
            return true;
        }
        if (t >= buf.back().stamp) {
            if (t - buf.back().stamp > MAX_TOLERANCE_SEC) return false;
            out = buf.back().value;
            return true;
        }
        auto it_after = std::lower_bound(
            buf.begin(), buf.end(), t,
            [](const ScalarRecord& record, double target_time) {
                return record.stamp < target_time;
            });
        if (it_after == buf.begin() || it_after == buf.end()) return false;
        auto it_before = it_after - 1;
        double alpha =
            (t - it_before->stamp) / (it_after->stamp - it_before->stamp);
        out = it_before->value + alpha * (it_after->value - it_before->value);
        return true;
    }

   public:
    void push_pos(double t, const Eigen::Vector3d& p) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!pos_buffer_.empty() && t <= pos_buffer_.back().stamp) return;
        pos_buffer_.push_back({t, p});
        while (pos_buffer_.size() > 2 &&
               (t - pos_buffer_.front().stamp) > MAX_HISTORY_SEC) {
            pos_buffer_.pop_front();
        }
    }

    void push_quat(double t, const Eigen::Quaterniond& q) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!quat_buffer_.empty() && t <= quat_buffer_.back().stamp) return;
        quat_buffer_.push_back({t, q});
        while (quat_buffer_.size() > 2 &&
               (t - quat_buffer_.front().stamp) > MAX_HISTORY_SEC) {
            quat_buffer_.pop_front();
        }
    }

    void push_gimbal_roll(double t, double val) {
        std::lock_guard<std::mutex> lk(mtx_);
        push_scalar(roll_buf_, t, val);
    }
    void push_gimbal_pitch(double t, double val) {
        std::lock_guard<std::mutex> lk(mtx_);
        push_scalar(pitch_buf_, t, val);
    }
    void push_gimbal_yaw(double t, double val) {
        std::lock_guard<std::mutex> lk(mtx_);
        push_scalar(yaw_buf_, t, val);
    }

    bool get_gimbal_at(double t, std::optional<double>& roll,
                       std::optional<double>& pitch,
                       std::optional<double>& yaw) {
        std::lock_guard<std::mutex> lk(mtx_);
        double r, p, y;
        bool ok_r = get_scalar_at(roll_buf_, t, r);
        bool ok_p = get_scalar_at(pitch_buf_, t, p);
        bool ok_y = get_scalar_at(yaw_buf_, t, y);
        if (ok_r)
            roll = r;
        else
            roll = std::nullopt;
        if (ok_p)
            pitch = p;
        else
            pitch = std::nullopt;
        if (ok_y)
            yaw = y;
        else
            yaw = std::nullopt;
        return ok_r || ok_p || ok_y;
    }

    // 工业级插值查找
    bool get_pose_at(double t, Eigen::Vector3d& out_p,
                     Eigen::Quaterniond& out_q, double& min_diff) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (pos_buffer_.empty() || quat_buffer_.empty()) return false;

        // --- 查找位置 ---
        double min_diff_p = 0.0;
        if (t <= pos_buffer_.front().stamp) {
            min_diff_p = (pos_buffer_.front().stamp - t);
            if (min_diff_p > MAX_TOLERANCE_SEC) return false;
            out_p = pos_buffer_.front().pos_enu;
        } else if (t >= pos_buffer_.back().stamp) {
            min_diff_p = (t - pos_buffer_.back().stamp);
            if (min_diff_p > MAX_TOLERANCE_SEC) return false;
            out_p = pos_buffer_.back().pos_enu;
        } else {
            auto it_after_p = std::lower_bound(
                pos_buffer_.begin(), pos_buffer_.end(), t,
                [](const DronePosRecord& record, double target_time) {
                    return record.stamp < target_time;
                });
            if (it_after_p == pos_buffer_.begin() ||
                it_after_p == pos_buffer_.end())
                return false;
            auto it_before_p = it_after_p - 1;
            double dt_p = it_after_p->stamp - it_before_p->stamp;
            if (dt_p < 1e-6) {
                out_p = it_before_p->pos_enu;
            } else {
                double alpha_p = (t - it_before_p->stamp) / dt_p;
                out_p = it_before_p->pos_enu +
                        alpha_p * (it_after_p->pos_enu - it_before_p->pos_enu);
            }
            min_diff_p =
                std::min(it_after_p->stamp - t, t - it_before_p->stamp);
        }

        // --- 查找姿态 ---
        double min_diff_q = 0.0;
        if (t <= quat_buffer_.front().stamp) {
            min_diff_q = (quat_buffer_.front().stamp - t);
            if (min_diff_q > MAX_TOLERANCE_SEC) return false;
            out_q = quat_buffer_.front().q;
        } else if (t >= quat_buffer_.back().stamp) {
            min_diff_q = (t - quat_buffer_.back().stamp);
            if (min_diff_q > MAX_TOLERANCE_SEC) return false;
            out_q = quat_buffer_.back().q;
        } else {
            auto it_after_q = std::lower_bound(
                quat_buffer_.begin(), quat_buffer_.end(), t,
                [](const DroneQuatRecord& record, double target_time) {
                    return record.stamp < target_time;
                });
            if (it_after_q == quat_buffer_.begin() ||
                it_after_q == quat_buffer_.end())
                return false;
            auto it_before_q = it_after_q - 1;
            double dt_q = it_after_q->stamp - it_before_q->stamp;
            if (dt_q < 1e-6) {
                out_q = it_before_q->q;
            } else {
                double alpha_q = (t - it_before_q->stamp) / dt_q;
                out_q = it_before_q->q.slerp(alpha_q, it_after_q->q);
            }
            min_diff_q =
                std::min(it_after_q->stamp - t, t - it_before_q->stamp);
        }

        min_diff = std::max(min_diff_p, min_diff_q);
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
    DirtyVar<Eigen::Vector3d> vel_angular_body{Eigen::Vector3d::Zero()};

    DirtyVar<double> stop_follow_stamp;

    DirtyVar<Eigen::Quaterniond> orientation;

    std::atomic<bool> odom_ok{false};
    std::atomic<double> current_battery{-1.0};
    std::atomic<double> voltage_battery{-1.0};
    std::atomic<bool> enable_joystick{false};  // 是否开启摇杆

    std::atomic<int> sensor_health{0};
    std::atomic<double> yaw_enu{0.0};
    DirtyVar<double> roll;
    DirtyVar<double> pitch;

    std::optional<std::function<void(Eigen::Vector3d, std::optional<double>)>>
        set_waypoint_goal = std::nullopt;

    PoseHistory pose_history;

    DatumSynchronizer datum;

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
        reg.bind("mission_data", mission_data, 10.0);

        // --- 2. 复合类型的自定义展开绑定 ---
        reg.bind_custom(lon_lat_alt, 10.0,
                        [](nlohmann::json& j, const Eigen::Vector3d& data) {
                            j["lon"] = data.x();
                            j["lat"] = data.y();
                            j["rel_alt"] = data.z();
                            j["gps"] =
                                Eigen::Vector3d{data.x(), data.y(), data.z()};
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