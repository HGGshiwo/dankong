#pragma once

#include <spdlog/fmt/ranges.h>  // 确保包含这个头文件

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>

#include "core/global_config.hpp"
#include "dk/adapters/can/can_client.hpp"
#include "features/car/events.hpp"
#include "ipc_vcu_zrd.h"
#include "irobot.hpp"
#include "utils/thread_runner.hpp"
#include "utils/throttle.hpp"

inline uint16_t to_uint16(double raw, double scale) {
    return (uint16_t)(raw / scale);
}

inline int16_t to_int16(double raw, double scale) {
    return (int16_t)(raw / scale);
}

class Car : public IRobot, public IThreadRunner {
   private:
    // 车辆物理参数常量 (需根据实际底盘尺寸修改)
    double WHEELBASE = GlobalConfig.GetConfig().wheelbase.get();  // 轴距 (米)
    double MAX_ANGLE = 25;  // 轮胎最大角度(度)

    static constexpr double TANK_TURN_SPEED = 5.0;  // 原地掉头时的默认旋转车速
    std::shared_ptr<CanClient> can_client_;
    Throttle t_{50};
    struct ipc_vcu_zrd_ipc_210_t cmd_210_;
    std::mutex cmd_mutex_;

   public:
    Car(std::shared_ptr<IMavlink> mavlink,
        std::shared_ptr<CanClient> can_client,
        std::shared_ptr<dk::ITimeProvider> time_provider)
        : IRobot(mavlink),
          IThreadRunner(time_provider),
          can_client_(can_client) {
        memset(&cmd_210_, 0, sizeof(cmd_210_));
        cmd_210_.ipc_en = 1;

        this->start(50);
    }
    ~Car() override = default;

    void on_step(double dt) override {
        uint8_t payload[8] = {0};
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            ipc_vcu_zrd_ipc_210_pack(payload, &cmd_210_, sizeof(payload));
            if (t_.shouldLog()) {
                spdlog::info("[Car] data: {:02X}", fmt::join(payload, " "));
            }
        }
        can_client_->send_frame(0x210, payload, 8);
    }

    // 计算目标距离：忽略 Z 轴（高度），只算 2D 平面欧氏距离
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return std::hypot(pos.x() - goal.x(), pos.y() - goal.y());
    }

    // 降落 -> 映射为：驻车/急停
    bool land() override {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.ipc_en = 1;       // 保持使能
        cmd_210_.brake_en = 1;     // 踩死刹车
        cmd_210_.target_gear = 0;  // 挂 P 挡
        cmd_210_.target_speed = 0.0;
        cmd_210_.target_angle = 0.0;
        cmd_210_.steering_mode = 0;
        return true;
    }

    // 悬停 -> 映射为：临时停车（N 挡待命）
    bool loiter() override {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.ipc_en = 1;
        cmd_210_.brake_en = 1;     // 踩下刹车
        cmd_210_.target_gear = 2;  // 挂 N 挡
        cmd_210_.target_speed = 0.0;
        cmd_210_.target_angle = 0.0;
        cmd_210_.steering_mode = 0;
        return true;
    }

    // 起飞 -> 映射为：解锁车辆，挂 D 挡准备行驶
    bool takeoff(double alt) override {
        // 地面车辆忽略 alt(高度) 参数
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.ipc_en = 1;       // 开启线控
        cmd_210_.brake_en = 0;     // 松开刹车
        cmd_210_.target_gear = 3;  // 挂 D 挡
        cmd_210_.target_speed = 0.0;
        cmd_210_.target_angle = 0.0;
        cmd_210_.steering_mode = 0;
        return true;
    }

    // --- 核心：运动学控制映射 ---
    // 输入: vel = [vx, vy, vz, yaw_rate]
    // vx/vy 单位 m/s, yaw_rate 单位 rad/s
    bool cmd_vel(Eigen::Vector4d vel) override {
        double vx = vel[0];
        double vy = vel[1];
        double yaw_rate = vel[3];

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        HoverArgs args;
        args.gear = cmd_210_.target_gear;
        if (!inner_check_hover(args)) {
            return false;
        }

        cmd_210_.ipc_en = 1;
        cmd_210_.brake_en = 0;

        double speed_ms = std::hypot(vx, vy);
        cmd_210_.target_gear = (vx < -0.05) ? 1 : 3;

        // 1. 纯原地旋转判定 (线速度极小，只有角速度)
        if (speed_ms < 0.05 && std::abs(yaw_rate) > 0.05) {
            cmd_210_.steering_mode = 1;
            cmd_210_.target_speed = 0;
            const double MAX_YAW_RATE_RAD_S = 1.0;
            double turn_cmd_val = (yaw_rate / MAX_YAW_RATE_RAD_S) * 300.0;
            cmd_210_.target_angle =
                static_cast<int16_t>(std::clamp(turn_cmd_val, -300.0, 300.0));
            return true;
        }

        // 2. 正常行驶状态 (存在线速度)
        cmd_210_.steering_mode = 0;

        if (speed_ms >= 0.05) {
            // 计算理论需要的纯前轮偏角 (弧度)
            double tire_angle_rad =
                std::atan((WHEELBASE * yaw_rate) / speed_ms);
            double wheel_angle_deg = tire_angle_rad * 180.0 / M_PI;

            // 检查是否超过了物理最大转角
            if (std::abs(wheel_angle_deg) > MAX_ANGLE) {
                // 【优雅处理】：超过极限转角时，限制在最大物理角度，并等比例降低线速度
                wheel_angle_deg =
                    (wheel_angle_deg > 0) ? MAX_ANGLE : -MAX_ANGLE;

                // 计算当前最大打角下的最小物理转弯半径
                double min_radius =
                    WHEELBASE / std::tan(MAX_ANGLE * M_PI / 180.0);
                // 重新计算能够匹配期望 yaw_rate 的安全线速度
                double safe_speed_ms = std::abs(yaw_rate) * min_radius;
                // 取较小值，避免加速
                speed_ms = std::min(speed_ms, safe_speed_ms);
            }

            // 将物理角度 wheel_angle_deg 线性映射到底盘指令区间 [-300, 300]
            double target_angle_cmd = (wheel_angle_deg / MAX_ANGLE) * 300.0;

            // 限幅防越界并强转
            cmd_210_.target_angle = static_cast<int16_t>(
                std::clamp(target_angle_cmd, -300.0, 300.0));

        } else {
            cmd_210_.target_angle = 0;
        }

        // 统一处理速度下发（经过上述可能的降速削减后）
        cmd_210_.target_speed =
            to_uint16(std::min(speed_ms * 3.6,
                               GlobalConfig.GetConfig().max_speed_kmh.get()),
                      0.1);

        return true;
    }

    void set_light(const LightEvent& event) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.ipc_en = 1;
        cmd_210_.turn_lamp = event.turn;
        cmd_210_.dipped_lamp = event.dipped ? 1 : 0;
        cmd_210_.far_lamp = event.far ? 1 : 0;
        cmd_210_.out_line_lamp = event.outline ? 1 : 0;
        cmd_210_.alarm_lamp = event.alarm ? 1 : 0;  // 双闪
    }

    void set_horn(const HornEvent& event) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.ipc_en = 1;
        // cmd_210_.turn_lamp = event.turn;
        // cmd_210_.dipped_lamp = event.dipped ? 1 : 0;
        // cmd_210_.far_lamp = event.far ? 1 : 0;
        // cmd_210_.out_line_lamp = event.outline ? 1 : 0;
        // cmd_210_.alarm_lamp = event.alarm ? 1 : 0;  // 双闪
        cmd_210_.horn = event.horn ? 1 : 0;
    }

    void set_estop(bool estop) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_210_.domain_ctrl_epo = estop ? 1 : 0;
    }

    // --- 状态检查 ---
    bool is_prearm_enable() override { return false; }
    bool is_alt_enable() override { return false; }  // 车不需要气压计高度

    // 复用基类的 unsigned int state 重载，你可以将底层采集的(车速+挡位)打包成
    // state 传进来
    bool inner_check_hover(HoverArgs args) override {
        // 这里只是示例，实际业务中可位操作提取：低8位是挡位，高16位是车速
        // return (speed < 0.1 && gear == 2);
        return args.gear.value() != 0 && args.gear.value() != 2;  // 不是P或者N
    }

    bool inner_is_landed(HoverArgs args) override {
        // 比如 state = EPOSts (急停状态)
        // return state != 0;
        return !inner_check_hover(args);
    }
};