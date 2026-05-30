#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>

// 定义目标运动状态枚举
enum class TargetState { STATIONARY, MOVING };

class TargetTracker {
   public:
    /**
     * @brief 构造函数
     * @param window_size 观测窗口的帧数，例如 30 帧 (1秒@30Hz)
     * @param base_threshold 判定运动的基础距离阈值，例如 0.2 米
     */
    TargetTracker(int window_size = 30, double base_threshold = 0.2)
        : window_size_(window_size),
          threshold_(base_threshold),
          state_(TargetState::STATIONARY) {}

    /**
     * @brief 重置接口：在开始追踪、或者丢失目标重新找回时调用
     */
    void reset() {
        history_.clear();
        state_ = TargetState::STATIONARY;
    }

    /**
     * @brief 核心更新接口：每次视觉出新位置时调用
     * @param pos 当前目标在物理世界（或 Odom）下的 2D 坐标 (X, Y)
     * @return 当前目标的运动状态
     */
    TargetState update(const Eigen::Vector2d& pos) {
        // 1. 更新滑动窗口
        history_.push_back(pos);
        if (history_.size() > window_size_) {
            history_.pop_front();
        }

        // 数据太少时，不足以做判断，默认保持原状态（通常是静止）
        // 至少需要窗口填满一半才开始计算
        if (history_.size() < window_size_ / 2) {
            return state_;
        }

        // 2. 计算滑动窗口内的包围盒 (Bounding Box)
        double min_x = history_[0].x(), max_x = history_[0].x();
        double min_y = history_[0].y(), max_y = history_[0].y();

        for (const auto& p : history_) {
            min_x = std::min(min_x, p.x());
            max_x = std::max(max_x, p.x());
            min_y = std::min(min_y, p.y());
            max_y = std::max(max_y, p.y());
        }

        // 3. 计算对角线分布距离，代表这段时间内的活动范围
        double spread_dist =
            std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_y - min_y, 2));

        // 4. 带迟滞的状态转移逻辑 (防止在阈值边缘反复横跳)
        if (state_ == TargetState::STATIONARY) {
            // 静止 -> 运动：要求更严格，必须超过阈值的 1.2 倍
            if (spread_dist > threshold_ * 1.2) {
                state_ = TargetState::MOVING;
            }
        } else {
            // 运动 -> 静止：要求必须真正停下来，活动范围小于阈值的 0.8 倍
            if (spread_dist < threshold_ * 0.8) {
                state_ = TargetState::STATIONARY;
            }
        }

        return state_;
    }

    /**
     * @brief 便捷查询接口
     */
    bool isMoving() const { return state_ == TargetState::MOVING; }

   private:
    int window_size_;
    double threshold_;
    TargetState state_;
    std::deque<Eigen::Vector2d> history_;  // 双端队列作为滑动窗口
};