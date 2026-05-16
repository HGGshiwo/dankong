#pragma once
#include <GeographicLib/UTMUPS.hpp>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <memory>
#include <optional>
#include <vector>

#include "../robot_context.hpp"
#include "./utils.hpp"
#include "dk/future.hpp"
#include "dk/logger.hpp"
#include "spdlog/spdlog.h"

namespace state_utils {

bool check_alt(RobotContext& ctx, double target);

Eigen::Vector3d get_relevant_enu(const Eigen::Vector3d& drone_lon_lat_alt,
                                 const Eigen::Vector3d& target_lon_lat_alt);

Eigen::Vector3d gps_to_enu(RobotContext& ctx, Eigen::Vector3d lon_lat_alt);

Eigen::Vector3d enu_to_gps(RobotContext& ctx, const Eigen::Vector3d& enu);

dk::Future<bool> prearm_check(RobotContext& ctx);

double yaw_enu_to_ned(double yaw_enu);

double yaw_ned_to_enu(double yaw_ned);

Eigen::Quaterniond euler_to_orientation(double r, double p, double y);

Eigen::Vector3d orientation_to_euler(double x, double y, double z, double w);

double norm_yaw(double yaw);

Eigen::Vector4d body_to_enu(const Eigen::Vector4d& body_target,
                            const Eigen::Vector4d& current_odom);

double get_heading(double enu_x, double enu_y);

double get_yaw_diff(double a, double b);

double get_time_span(std::chrono::steady_clock::time_point start);

bool should_do_prearm_check(RobotContext& ctx);

bool check_sensor_health(uint32_t sensor_health);

bool is_prearm_msg(const std::string& text);

dk::Future<bool> set_mode(RobotContext& ctx, FixedString64 mode);

// 对double数组进行检查，是否所有值都被卡住
template <int N>
class StallChecker {
    std::chrono::steady_clock::time_point start_time_;
    double check_time_s_;

    // 强烈建议使用 std::array，它支持直接赋值 (=) 和拷贝
    std::array<double, N> last_data_;
    std::array<double, N> threshold_;
    bool is_initialized_ = false;  //
   public:
    // 构造函数：直接接收 std::array 作为阈值，保证元素个数绝对等于 N
    StallChecker(std::array<double, N> threshold, double check_time_s)
        : threshold_(threshold),
          check_time_s_(check_time_s),
          start_time_(std::chrono::steady_clock::now()) {}

    // 参数也改为接收 std::array 或者 double 数组指针
    bool is_stall(std::array<double, N> data) {
        if (!is_initialized_) {
            // 第一次初始化
            for (int i = 0; i < N; ++i) {
                last_data_[i] = data[i];
            }
            is_initialized_ = true;
            start_time_ = std::chrono::steady_clock::now();  // 初始化时重置时间
            return false;
        }
        bool res = false;  // 默认没有卡住 (或者根据你的业务逻辑改为 true)
        // 计算时间差
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> diff = now - start_time_;
        // 到达检测频率，进行检测
        if (diff.count() > check_time_s_) {
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

}  // namespace state_utils