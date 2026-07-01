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
    }

    // ---------------------------------------------------------
    // 回调入口 2：推入 GPS 数据并触发同步 (低频)
    // ---------------------------------------------------------
    void pushGPS(const Eigen::Vector3d& gps, double time) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 1. 尝试将高频 ENU 插值对齐到当前 GPS 的时间戳
        std::optional<Eigen::Vector3d> synced_enu = interpolateENU(time);

        if (synced_enu.has_value()) {
            // 2. 如果插值成功，压入滑动窗口
            synced_window_.push_back({time, synced_enu.value(), gps});
            if (synced_window_.size() > window_size_) {
                synced_window_.pop_front();
            }
        }
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
    // 基于时间戳的线性插值
    std::optional<Eigen::Vector3d> interpolateENU(double target_time) {
        if (enu_buffer_.size() < 2) return std::nullopt;

        // 如果目标时间比我们拥有的最老数据还老，或者比最新数据还新很多，抛弃
        if (target_time < enu_buffer_.front().time ||
            target_time > enu_buffer_.back().time + max_time_diff_) {
            return std::nullopt;
        }

        // 寻找目标时间前后的两个 ENU 帧
        for (size_t i = 0; i < enu_buffer_.size() - 1; ++i) {
            const auto& enu_1 = enu_buffer_[i];
            const auto& enu_2 = enu_buffer_[i + 1];

            if (target_time >= enu_1.time && target_time <= enu_2.time) {
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

    // 可靠性校验逻辑
    bool isWindowReliable() {
        // 核心思想：在短时间窗口内，机器人 ENU 的相对位移，应该与 GPS
        // 的相对位移在物理学上是一致的。 如果 GPS
        // 发生了室内多径漂移，其局部方差会急剧上升。

        // 注意：这里为了保持代码不依赖第三方库，采用了一个简化的启发式稳态校验。
        // 严谨的做法是：在这里调用你已有的 GeographicLib，将 synced_window_
        // 中的 gps 转换为局部 XYZ， 然后计算 (ENU_Delta - GPS_XYZ_Delta)
        // 的均方根误差 (RMSE)。如果 RMSE 小于阈值，则认为可靠。

        // 简化版稳态检测（仅检测高程和水平震荡）：
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

        // 阈值需要根据你的建图算法输出尺度和 GPS 噪声级别进行整定 (Tuning)
        double max_allowed_variance = 5.0;

        return enu_variance < max_allowed_variance;
    }

    std::mutex mutex_;
    double max_time_diff_;
    size_t window_size_;

    std::deque<StampedENU> enu_buffer_;
    std::deque<SyncedPair> synced_window_;
};