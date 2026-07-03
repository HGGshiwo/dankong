#include <Eigen/Dense>
#include <cmath>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>

// 数据载体定义
struct StampedENU {
    double time;
    Eigen::Vector3d position;  // x, y, z
};

struct StampedGPS {
    double time;
    Eigen::Vector3d lon_lat_alt;
};

// 经过时间同步后的一对数据
struct SyncedPair {
    double time;
    Eigen::Vector3d enu;
    Eigen::Vector3d gps;
};

class DatumSynchronizer {
   public:
    DatumSynchronizer(double max_time_diff = 0.1, size_t window_size = 10)
        : max_time_diff_(max_time_diff), window_size_(window_size) {}

    // ---------------------------------------------------------
    // 回调入口 1：推入 ENU 数据 (高频)
    // ---------------------------------------------------------
    void pushENU(const Eigen::Vector3d& enu, double time) {
        std::lock_guard<std::mutex> lock(mutex_);
        enu_buffer_.push_back({time, enu});

        // 维持 Buffer 大小，防止内存溢出 (保留最近 2 秒的数据，假设 100Hz)
        if (enu_buffer_.size() > 200) {
            enu_buffer_.pop_front();
        }

        // 每次有新的 ENU 数据到来，尝试唤醒并处理积压的 GPS 数据
        trySync();
    }

    // ---------------------------------------------------------
    // 回调入口 2：推入 GPS 数据 (高/低频均可)
    // ---------------------------------------------------------
    void pushGPS(const Eigen::Vector3d& gps, double time) {
        std::lock_guard<std::mutex> lock(mutex_);
        gps_buffer_.push_back({time, gps});

        // 维持 GPS Buffer 大小
        if (gps_buffer_.size() > 50) {
            gps_buffer_.pop_front();
        }

        // 尝试同步
        trySync();
    }

    // ---------------------------------------------------------
    // 外部调用：获取一对可靠的基准坐标
    // ---------------------------------------------------------
    std::optional<SyncedPair> getReliableDatum() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (synced_window_.size() < window_size_) {
            return std::nullopt;  // 数据量不足以进行稳态评估
        }

        if (isWindowReliable()) {
            // 如果窗口内数据统计学稳定，返回最新的一对作为基准
            return synced_window_.back();
        }

        return std::nullopt;
    }

   private:
    // 核心同步逻辑 (调用前必须已加锁)
    void trySync() {
        while (!gps_buffer_.empty()) {
            double target_gps_time = gps_buffer_.front().time;

            if (enu_buffer_.size() < 2) {
                break;  // ENU 数据不足，继续等待
            }

            // 情况 1: GPS 数据太老了，已经被最老的 ENU
            // 甩在后面，无法插值，直接丢弃
            if (target_gps_time < enu_buffer_.front().time) {
                gps_buffer_.pop_front();
                continue;
            }

            // 情况 2: GPS 数据比目前最新的 ENU 还要新。
            // 解决你的 Bug 的关键：不要丢弃，直接 break 等待未来的 ENU 到来！
            if (target_gps_time > enu_buffer_.back().time) {
                break;
            }

            // 情况 3: GPS 时间戳正好落在 ENU
            // 队列的时间范围内，完美包夹，进行插值
            std::optional<Eigen::Vector3d> synced_enu =
                interpolateENU(target_gps_time);

            if (synced_enu.has_value()) {
                synced_window_.push_back({target_gps_time, synced_enu.value(),
                                          gps_buffer_.front().lon_lat_alt});
                if (synced_window_.size() > window_size_) {
                    synced_window_.pop_front();
                }
            }

            // 无论插值是否成功（可能因为 max_time_diff_ 限制失败），这个 GPS
            // 已经处理完毕
            gps_buffer_.pop_front();
        }
    }

    // 基于时间戳的线性插值 (调用前已确保 target_time 在 enu_buffer_ 范围内)
    std::optional<Eigen::Vector3d> interpolateENU(double target_time) {
        // 寻找目标时间前后的两个 ENU 帧
        for (size_t i = 0; i < enu_buffer_.size() - 1; ++i) {
            const auto& enu_1 = enu_buffer_[i];
            const auto& enu_2 = enu_buffer_[i + 1];

            if (target_time >= enu_1.time && target_time <= enu_2.time) {
                // 如果相邻两个 ENU 帧的时间跨度过大
                // (比如传感器掉线)，拒绝插值以防飞车
                if ((enu_2.time - enu_1.time) > max_time_diff_) {
                    return std::nullopt;
                }

                // 计算插值权重
                double dt = enu_2.time - enu_1.time;
                if (dt < 1e-6) return enu_1.position;  // 避免除以 0

                double ratio = (target_time - enu_1.time) / dt;

                // 线性插值计算对齐后的 ENU
                Eigen::Vector3d interpolated =
                    enu_1.position + ratio * (enu_2.position - enu_1.position);
                return interpolated;
            }
        }
        return std::nullopt;
    }

    // 可靠性校验逻辑 (保持原样即可)
    bool isWindowReliable() {
        Eigen::Vector3d enu_mean = Eigen::Vector3d::Zero();
        for (const auto& pair : synced_window_) {
            enu_mean += pair.enu;
        }
        enu_mean /= synced_window_.size();

        double enu_variance = 0.0;
        for (const auto& pair : synced_window_) {
            enu_variance += (pair.enu - enu_mean).squaredNorm();
        }
        enu_variance /= synced_window_.size();

        double max_allowed_variance = 5.0;
        return enu_variance < max_allowed_variance;
    }

    std::mutex mutex_;
    double max_time_diff_;
    size_t window_size_;

    std::deque<StampedENU> enu_buffer_;
    std::deque<StampedGPS> gps_buffer_;  // 新增 GPS 缓存队列
    std::deque<SyncedPair> synced_window_;
};