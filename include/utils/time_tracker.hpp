#pragma once

#include <chrono>
#include <limits>

// =====================================================================
// 环境切换开关：
// 迁移到非 ROS 环境时，注释掉此宏即可剥离所有 ROS 依赖
#define ENVIRONMENT_HAS_ROS
// =====================================================================

#ifdef ENVIRONMENT_HAS_ROS
#include <ros/ros.h>  // 如果使用 ROS 2，改为 #include <rclcpp/rclcpp.hpp>
#endif

class TimeTracker {
   public:
    // 定义时间来源类型
    enum class TimeSource {
        UNINITIALIZED,  // 未初始化（默认极大值状态）
        CHRONO,         // 纯 C++ 系统时间
        ROS             // ROS 系统时间
    };

   private:
    double record_time_;
    TimeSource source_;

    // 内部私有方法：获取纯 C++ 系统时间的秒数
    static double get_chrono_now() {
        auto current_time = std::chrono::system_clock::now();
        auto duration = current_time.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

   public:
    // ==========================================
    // 构造函数
    // ==========================================

    // 1. 无参构造：默认赋极大值，状态标记为未初始化
    TimeTracker()
        : record_time_(std::numeric_limits<double>::max()),
          source_(TimeSource::UNINITIALIZED) {}

    // 2. 非 ROS 方式：传入纯 C++ 的 double 时间戳
    explicit TimeTracker(double chrono_sec)
        : record_time_(chrono_sec), source_(TimeSource::CHRONO) {}

#ifdef ENVIRONMENT_HAS_ROS
    // 3. ROS 方式：传入 ROS 时间戳 (ROS 1 为例)
    explicit TimeTracker(const ros::Time& ros_stamp)
        : record_time_(ros_stamp.toSec()), source_(TimeSource::ROS) {}
#endif

    // ==========================================
    // 核心方法
    // ==========================================

    // 获取到目前为止流逝的秒数
    double elapsed_seconds() const {
        // 如果是初始的极大值状态，不进行实际计算，直接返回 0.0
        // （你可以根据业务逻辑，将其改为返回 -1.0 以表示异常或未触发）
        if (source_ == TimeSource::UNINITIALIZED) {
            return 0.0;
        }

        double current_now = 0.0;

        if (source_ == TimeSource::ROS) {
#ifdef ENVIRONMENT_HAS_ROS
            // 如果是 ROS 来源，使用 ROS 的 now()
            current_now = ros::Time::now().toSec();
#else
            // 安全回退：如果关闭了 ROS 宏但误入了此分支
            current_now = get_chrono_now();
#endif
        } else {
            // 如果是 CHRONO 来源，使用 Chrono 的 now()
            current_now = get_chrono_now();
        }

        return current_now - record_time_;
    }

    double to_double() const { return record_time_; }

#ifdef ENVIRONMENT_HAS_ROS
    ros::Time to_ros_time() const { return ros::Time(record_time_); }
#endif

    // ==========================================
    // 更新值的方法 (因为有无参构造，通常需要后续赋值)
    // ==========================================

    // 赋实际值：非 ROS 时间戳
    void update(double chrono_sec) {
        record_time_ = chrono_sec;
        source_ = TimeSource::CHRONO;
    }

#ifdef ENVIRONMENT_HAS_ROS
    // 赋实际值：ROS 时间戳
    void update(const ros::Time& ros_stamp) {
        record_time_ = ros_stamp.toSec();
        source_ = TimeSource::ROS;
    }
#endif
};