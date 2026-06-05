#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "dk/adapters/can/can_client.hpp"
#include "irobot.hpp"

// 假设这是底层 CAN 适配器或打包库提供的接口
// 如果你使用 cantools，它会自动生成这个结构体和 pack 函数
struct ipc_210_cmd_t {
    uint8_t ipc_en;         // 控制使能
    uint8_t brake_en;       // 制动使能
    uint8_t target_gear;    // 目标挡位: 0=P, 1=R, 2=N, 3=D
    uint8_t steering_mode;  // 转向模式: 0=阿克曼, 1=原地掉头
    double target_speed;    // 目标车速 (km/h)
    double target_angle;    // 目标角度 (°)
};
extern void ipc_210_pack(uint8_t* dst, const ipc_210_cmd_t* src, size_t len);

class Car : public IRobot {
   private:
    // 车辆物理参数常量 (需根据实际底盘尺寸修改)
    static constexpr double WHEELBASE = 2.5;  // 轴距 (米)
    static constexpr double STEERING_RATIO =
        16.0;  // 转向比 (方向盘转角 / 轮胎实际偏角)
    static constexpr double MAX_SPEED_KMH = 2.0;  // 限制最高车速
    static constexpr double TANK_TURN_SPEED = 5.0;  // 原地掉头时的默认旋转车速
    std::shared_ptr<CanClient> can_client_;

   public:
    Car(std::shared_ptr<IMavlink> mavlink,
        std::shared_ptr<CanClient> can_client)
        : IRobot(mavlink), can_client_(can_client) {}
    ~Car() override = default;

    // 计算目标距离：忽略 Z 轴（高度），只算 2D 平面欧氏距离
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return std::hypot(pos.x() - goal.x(), pos.y() - goal.y());
    }

    // 降落 -> 映射为：驻车/急停
    bool land() override {
        ipc_210_cmd_t cmd = {0};
        cmd.ipc_en = 1;       // 保持使能
        cmd.brake_en = 1;     // 踩死刹车
        cmd.target_gear = 0;  // 挂 P 挡
        cmd.target_speed = 0.0;
        cmd.target_angle = 0.0;
        cmd.steering_mode = 0;

        uint8_t payload[8] = {0};
        ipc_210_pack(payload, &cmd, sizeof(payload));
        return can_client_->send_frame(0x210, payload,
                                       8);  // 0x210 即十进制 528
    }

    // 悬停 -> 映射为：临时停车（N 挡待命）
    bool loiter() override {
        ipc_210_cmd_t cmd = {0};
        cmd.ipc_en = 1;
        cmd.brake_en = 1;     // 踩下刹车
        cmd.target_gear = 2;  // 挂 N 挡
        cmd.target_speed = 0.0;
        cmd.target_angle = 0.0;
        cmd.steering_mode = 0;

        uint8_t payload[8] = {0};
        ipc_210_pack(payload, &cmd, sizeof(payload));
        return can_client_->send_frame(0x210, payload, 8);
    }

    // 起飞 -> 映射为：解锁车辆，挂 D 挡准备行驶
    bool takeoff(double alt) override {
        // 地面车辆忽略 alt(高度) 参数
        ipc_210_cmd_t cmd = {0};
        cmd.ipc_en = 1;       // 开启线控
        cmd.brake_en = 0;     // 松开刹车
        cmd.target_gear = 3;  // 挂 D 挡
        cmd.target_speed = 0.0;
        cmd.target_angle = 0.0;
        cmd.steering_mode = 0;

        uint8_t payload[8] = {0};
        ipc_210_pack(payload, &cmd, sizeof(payload));
        return can_client_->send_frame(0x210, payload, 8);
    }

    // --- 核心：运动学控制映射 ---
    // 输入: vel = [vx, vy, vz, yaw_rate]
    // vx/vy 单位 m/s, yaw_rate 单位 rad/s
    bool cmd_vel(Eigen::Vector4d vel) override {
        double vx = vel[0];
        double vy = vel[1];
        // double vz = vel[2]; // 车辆忽略垂直速度
        double yaw_rate = vel[3];

        ipc_210_cmd_t cmd = {0};
        cmd.ipc_en = 1;
        cmd.brake_en = 0;

        // 1. 计算合速度 (m/s 转 km/h)
        double speed_ms = std::hypot(vx, vy);
        cmd.target_speed = std::min(speed_ms * 3.6, MAX_SPEED_KMH);

        // 2. 挡位判断：依据前向速度 vx 的正负号
        if (vx < -0.05) {
            cmd.target_gear = 1;  // 1 = R 挡 (倒车)
        } else {
            cmd.target_gear = 3;  // 3 = D 挡 (前进)
        }

        // 3. 转向模式与角度计算
        if (speed_ms < 0.05 && std::abs(yaw_rate) > 0.05) {
            // 【状态 A：原地掉头】车速趋于0，但要求偏航角速度不为0
            cmd.steering_mode = 1;  // 1 = 原地掉头

            // 原地掉头时，Target_Speed 往往用作旋转的驱动力度/速度
            cmd.target_speed = TANK_TURN_SPEED;

            // 角度根据方向给极值，或者根据厂家协议要求给特定值
            cmd.target_angle = (yaw_rate > 0) ? 90.0 : -90.0;

        } else {
            // 【状态 B：阿克曼正常转向】
            cmd.steering_mode = 0;  // 0 = 阿克曼转向

            // 车辆运动学模型 (自行车模型): δ = arctan(L * ω / v)
            if (speed_ms >= 0.05) {
                double tire_angle_rad =
                    std::atan((WHEELBASE * yaw_rate) / speed_ms);

                // 将轮胎弧度转换为 方向盘角度 (乘以转向比并转为角度)
                double steering_wheel_angle =
                    (tire_angle_rad * 180.0 / M_PI) * STEERING_RATIO;

                // 限制在 DBC 规定的范围内 (DBC定义的是 -32768 到
                // 32767，但实际车会有物理极值)
                cmd.target_angle =
                    std::clamp(steering_wheel_angle, -540.0, 540.0);
            } else {
                cmd.target_angle = 0.0;
            }
        }

        uint8_t payload[8] = {0};
        ipc_210_pack(payload, &cmd, sizeof(payload));
        return can_client_->send_frame(0x210, payload, 8);
    }

    // --- 状态检查 ---
    bool is_prearm_enable() override { return true; }
    bool is_alt_enable() override { return true; }  // 车不需要气压计高度

    // 复用基类的 unsigned int state 重载，你可以将底层采集的(车速+挡位)打包成
    // state 传进来
    bool inner_check_hover(unsigned int state) override {
        // 这里只是示例，实际业务中可位操作提取：低8位是挡位，高16位是车速
        // return (speed < 0.1 && gear == 2);
        return true;
    }

    bool inner_is_landed(unsigned int state) override {
        // 比如 state = EPOSts (急停状态)
        // return state != 0;
        return true;
    }
};