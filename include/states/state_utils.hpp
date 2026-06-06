#pragma once
#include <Eigen/Dense>
#include <GeographicLib/UTMUPS.hpp>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <optional>
#include <vector>

#include "dk/future.hpp"
#include "dk/logger.hpp"
#include "features/control/events.hpp"
#include "robot/irobot.hpp"
#include "spdlog/spdlog.h"
#include "utils/fixed_string64.hpp"

namespace state_utils {

Eigen::Vector3d get_relevant_enu(const Eigen::Vector3d& drone_lon_lat_alt,
                                 const Eigen::Vector3d& target_lon_lat_alt);

Eigen::Vector3d gps_to_enu(Eigen::Vector3d cur_lon_lat_alt,
                           Eigen::Vector3d cur_pos_enu,
                           Eigen::Vector3d lon_lat_alt);

Eigen::Vector3d enu_to_gps(Eigen::Vector3d cur_lon_lat_alt,
                           Eigen::Vector3d cur_pos_enu,
                           const Eigen::Vector3d& enu);

double yaw_enu_to_ned(double yaw_enu);

double yaw_ned_to_enu(double yaw_ned);

Eigen::Quaterniond euler_to_orientation(double r, double p, double y);

Eigen::Vector3d orientation_to_euler(double x, double y, double z, double w);

Eigen::Vector3d orientation_to_euler(Eigen::Quaterniond q);

double norm_yaw(double yaw);

Eigen::Vector4d body_to_enu(const Eigen::Vector4d& body_target,
                            const Eigen::Vector4d& current_odom);

double get_heading(double enu_x, double enu_y);

double get_yaw_diff(double a, double b);

double get_time_span(double start_time, double current_now);

bool should_do_prearm_check(std::shared_ptr<IRobot> robot);

bool check_sensor_health(uint32_t sensor_health);

bool is_prearm_msg(const std::string& text);

// 对double数组进行检查，是否所有值都被卡住
template <int N>
class StallChecker {
    double start_time_;
    double check_time_s_;

    // 强烈建议使用 std::array，它支持直接赋值 (=) 和拷贝
    std::array<double, N> last_data_;
    std::array<double, N> threshold_;
    bool is_initialized_ = false;  //
   public:
    // 构造函数：直接接收 std::array 作为阈值，保证元素个数绝对等于 N
    StallChecker(std::array<double, N> threshold, double check_time_s,
                 double now)
        : threshold_(threshold),
          check_time_s_(check_time_s),
          start_time_(now) {}

    // 参数也改为接收 std::array 或者 double 数组指针
    bool is_stall(std::array<double, N> data, double now) {
        if (!is_initialized_) {
            // 第一次初始化
            for (int i = 0; i < N; ++i) {
                last_data_[i] = data[i];
            }
            is_initialized_ = true;
            start_time_ = now;  // 初始化时重置时间
            return false;
        }
        bool res = false;  // 默认没有卡住 (或者根据你的业务逻辑改为 true)

        double diff = now - start_time_;
        // 到达检测频率，进行检测
        if (diff > check_time_s_) {
            res = true;
            for (int i = 0; i < N; ++i) {
                // 注意这里比较的是 threshold_[i] 而不是 threshold_
                if (std::fabs(last_data_[i] - data[i]) > threshold_[i]) {
                    res = false;  // 数据有明显变化，没有卡住
                    break;
                }
            }

            // 更新 last_data_ 为当前数据
            for (int i = 0; i < N; ++i) {
                last_data_[i] = data[i];
            }
            // 必须重置时间，否则下次调用会直接跳过时间检查！
            start_time_ = now;
        }
        return res;
    }
};

class AngleController {
   private:
    bool was_turning_cw = false;  // 记录上一次是否在顺时针旋转
   public:
    /**
     * @brief 带有状态迟滞的最短角度计算
     * @param current 当前角度 (弧度)
     * @param target 目标角度 (弧度)
     * @param deadband 迟滞死区 (弧度)，例如 0.17 弧度 (约10度)
     */
    double get_distance(double current, double target, double deadband = 0.17) {
        double diff = target - current;
        double wrapped_diff = std::remainder(diff, 2.0 * M_PI);
        // 如果误差在 180度 (PI) 附近（进入死区）
        if (std::abs(wrapped_diff) > (M_PI - deadband)) {
            // 如果我们之前已经在顺时针转了，强制继续顺时针（保证输出为负）
            // 只要 wrapped_diff 是正的，就减去 2PI
            if (was_turning_cw && wrapped_diff > 0) {
                wrapped_diff -= 2.0 * M_PI;
            }
            // 如果题目要求：只要在这个范围内，*无条件* 开始顺时针转（覆盖历史）
            // 则取消上面的判断，直接写：
            // if (wrapped_diff > 0) wrapped_diff -= 2.0 * M_PI;
            // was_turning_cw = true;
        }
        // 更新历史状态：如果输出小于0，说明正在顺时针旋转
        was_turning_cw = (wrapped_diff < 0);
        return wrapped_diff;
    }
};

}  // namespace state_utils