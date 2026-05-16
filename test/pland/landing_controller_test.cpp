#include "pland/landing_controller.hpp"

#include <gtest/gtest.h>

#include <random>

#include "utils/easyboard.hpp"

// 1. 测试初始化和重置逻辑
TEST(LandingControllerTest, ResetLogic) {
    LandingConfig config;
    LandingController ctrl(config);

    // 没使能时，update 应该返回 false
    Eigen::Vector4d cmd;
    EXPECT_FALSE(ctrl.update(Eigen::Vector4d::Zero(), 1.0, 1.0, cmd));

    ctrl.set_land_enable(true);
    EXPECT_TRUE(ctrl.update(Eigen::Vector4d::Zero(), 1.0, 1.0, cmd));
}
// 2. 测试目标丢失（超时保护）逻辑
TEST(LandingControllerTest, TargetLostTimeout) {
    LandingConfig config;
    config.lost_timeout = 2.0;  // 设置 2 秒超时
    LandingController ctrl(config);
    ctrl.set_land_enable(true);
    Eigen::Vector4d pos = Eigen::Vector4d(1.0, 0.0, 5.0, 0.0);
    Eigen::Vector4d cmd;
    // 正常传入，视觉时间戳和控制时间戳一致
    EXPECT_TRUE(ctrl.update(pos, 10.0, 10.0, cmd));
    // 假设过了 2.1 秒，视觉时间戳没有更新（停在 10.0），但控制时间戳到了 12.1
    EXPECT_FALSE(ctrl.update(pos, 10.0, 12.1, cmd));  // 应该触发超时断开
}
// 3. 测试 Z 轴的分段降落逻辑
TEST(LandingControllerTest, ZAxisDescentCurve) {
    LandingConfig config;
    LandingController ctrl(config);
    ctrl.set_land_enable(true);
    Eigen::Vector4d cmd;
    // 模拟无人机在正上方 5 米处 (XY完美对齐)
    Eigen::Vector4d pos(0.0, 0.0, 5.0, 0.0);
    ctrl.update(pos, 1.0, 1.0, cmd);  // 第一次初始化时间戳
    ctrl.update(pos, 1.1, 1.1, cmd);  // 产生 dt=0.1

    // Z 轴应该输出一个负向的速度 (下降)
    EXPECT_LT(cmd.z(), 0.0);
    // 模拟无人机偏离中心 2 米 (没有对齐)
    pos.x() = 2.0;
    // 最大加速度是config.max_acc.z()
    double t = cmd.z() / config.max_acc.z();
    ctrl.update(pos, t + 0.1, t + 0.1, cmd);
    // 没对齐时，Z 轴速度应该为 0 (悬停等待)
    EXPECT_NEAR(cmd.z(), 0.0, 1e-5);
}

TEST(LandingControllerTest, ClosedLoopConvergence) {
    LandingConfig config;
    config.kp = Eigen::Vector3d(1.0, 1.0, 1.0);
    // ... 配置好你的参数
    easyboard::SummaryWriter writer(
        "/home/hgg/catkin_ws/src/mavproxy_ros/test_log/landing_controller",
        {"landing_controller"});
    LandingController ctrl(config);
    ctrl.set_land_enable(true);

    // 物理世界模拟变量
    double dt = 0.01;  // 100Hz 模拟
    double time = 0.0;

    // 假设无人机初始位置在 [X: 5.0m, Y: -3.0m, Z: 10.0m, Yaw: 0.5rad]
    Eigen::Vector4d drone_pos(5.0, -3.0, 10.0, 0.5);
    // 假设降落板(目标)永远在原点 [0, 0, 0, 0]
    std::normal_distribution<double> dist(0, 0.1);
    // 初始化随机数发生器
    std::random_device rd;   // 随机数种子
    std::mt19937 gen(rd());  // 均值标准发生器

    for (int i = 0; i < 3000; ++i) {  // 模拟飞行 30 秒
        time += dt;

        // 1. 计算传感器看到的相对误差
        Eigen::Vector4d relative_pos = Eigen::Vector4d::Zero() - drone_pos;
        writer.add_scalar("pos/x", relative_pos.x(), i);
        writer.add_scalar("pos/y", relative_pos.y(), i);
        writer.add_scalar("pos/z", relative_pos.z(), i);
        writer.add_scalar("pos/w", relative_pos.w(), i);
        // 【可选：加入传感器噪声】可以给 relative_pos
        // 加上一点高斯随机噪声，测试 KF 的鲁棒性

        // 生成噪声向量并相加
        Eigen::Vector4d noise(dist(gen), dist(gen), dist(gen), dist(gen));
        relative_pos += noise;
        // 2. 调用控制器计算速度指令
        Eigen::Vector4d cmd_vel;
        ctrl.update(relative_pos, time, time, cmd_vel);

        // 3. 模拟无人机的物理运动 (积分：位置 = 位置 + 速度 * dt)
        // 假设飞控底层能完美追踪你下发的速度
        drone_pos.x() += cmd_vel.x() * dt;
        drone_pos.y() += cmd_vel.y() * dt;
        drone_pos.z() += cmd_vel.z() * dt;
        drone_pos.w() += cmd_vel.w() * dt;

        writer.add_scalar("cmd_vel/x", cmd_vel.x(), i);
        writer.add_scalar("cmd_vel/y", cmd_vel.y(), i);
        writer.add_scalar("cmd_vel/z", cmd_vel.z(), i);
        writer.add_scalar("cmd_vel/w", cmd_vel.w(), i);
    }

    double z = std::fmax(drone_pos.z(), 0.0);
    // 断言：20秒后，无人机应该无限接近降落板
    EXPECT_NEAR(drone_pos.x(), 0.0, 0.1);
    EXPECT_NEAR(drone_pos.y(), 0.0, 0.1);
    EXPECT_NEAR(z, 0.0, 0.35);  // 比如最后停留在 0.35m
}

TEST(LandingControllerTest, LandingHard) {
    LandingConfig config;
    // config.kp = Eigen::Vector3d(1.0, 1.0, 1.0);
    config.max_output = {3.0, 3.0, 2.0};
    // 假设你配置了其他前馈和滤波参数...

    easyboard::SummaryWriter writer_drone(
        "/home/hgg/catkin_ws/src/mavproxy_ros/test_log/ultimate_landing",
        {"landing_controller_hard", "drone"});
    easyboard::SummaryWriter writer_target(
        "/home/hgg/catkin_ws/src/mavproxy_ros/test_log/ultimate_landing2",
        {"landing_controller_hard", "target"});
    LandingController ctrl(config);
    ctrl.set_land_enable(true);
    double dt = 0.01;  // 100Hz 模拟
    double time = 0.0;
    // 无人机初始位置 (高空，并且有较大偏差)
    Eigen::Vector4d drone_pos(5.0, -5.0, 15.0, 0.0);

    // 目标(船只)初始位置
    Eigen::Vector4d target_pos(0.0, 0.0, 0.0, 0.0);
    // 随机数与噪声生成器 (放在循环外)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> pos_dist(0, 0.05);  // 5cm 位置视觉噪声
    std::normal_distribution<double> yaw_dist(0, 0.02);  // 角度视觉噪声
    for (int i = 0; i < 4000; ++i)  // 模拟飞行 40 秒 (足够长，看全过程)
    {
        time += dt;
        // ==========================================
        // 😈 难度 1：目标的变加速蛇形走位 (模拟船只摇摆/避障)
        // ==========================================
        Eigen::Vector4d target_vel;
        // X轴：忽快忽慢的变速运动 (在 0.5 到 2.5 m/s 之间波动)
        target_vel.x() = 1.5 + 1.0 * std::sin(0.2 * time);
        // Y轴：S型左右摇摆走位
        target_vel.y() = 1.2 * std::cos(0.3 * time);
        // Z轴：海浪导致的甲板上下颠簸 (上下 20 厘米浮动)
        target_vel.z() = 0.2 * std::sin(0.5 * time);
        target_vel.w() = 0.0;  // 假设目标不自转
        target_pos += target_vel * dt;
        // 计算真实相对误差并加入噪声
        Eigen::Vector4d true_relative_pos = target_pos - drone_pos;
        Eigen::Vector4d noise(pos_dist(gen), pos_dist(gen), pos_dist(gen),
                              yaw_dist(gen));
        Eigen::Vector4d measured_relative_pos = true_relative_pos + noise;
        // ==========================================
        // 😈 难度 2：视觉短暂丢失 (模拟云台被遮挡或背光)
        // ==========================================
        double pos_timestamp = time;
        // 在 15秒 到 16秒 的这 1 秒钟内，视觉突然卡死不更新！
        if (time > 15.0 && time < 16.0) {
            // 维持旧的测量值和旧的时间戳，看控制器会不会乱飞或超时
            // measured_relative_pos =
            // [使用上一帧的旧数据，这里省略复杂的状态机，仅用时间戳滞后模拟]
            pos_timestamp = 15.0;
        }
        // 调用控制器计算
        Eigen::Vector4d cmd_vel;
        ctrl.update(measured_relative_pos, pos_timestamp, time, cmd_vel);
        // ==========================================
        // 😈 难度 3：突发高空切变风扰动 (考验 I 项和 D 项)
        // ==========================================
        Eigen::Vector4d wind_disturbance(0, 0, 0, 0);
        // 在第 5 秒到第 8 秒，突然刮起一阵把飞机向 Y 轴负方向吹的强风
        if (time > 5.0 && time < 8.0) {
            wind_disturbance.y() = -1.5;  // 风的等效漂移速度
        }
        // 模拟无人机的物理运动
        // 物理位置 = 飞控执行的速度 + 外界强风的吹拂
        drone_pos += (cmd_vel + wind_disturbance) * dt;
        // 记录数据到 TensorBoard 进行复盘
        writer_target.add_scalar("Vel_X", target_vel.x(), i);
        writer_target.add_scalar("Vel_Y", target_vel.y(), i);
        writer_target.add_scalar("Error/True_X", true_relative_pos.x(), i);
        writer_target.add_scalar("Error/True_Y", true_relative_pos.y(), i);
        writer_target.add_scalar("Error/True_Z", true_relative_pos.z(), i);
        writer_drone.add_scalar("Vel_X", cmd_vel.x(), i);
        writer_drone.add_scalar("Vel_Y", cmd_vel.y(), i);
        writer_drone.add_scalar("Vel_Z", cmd_vel.z(), i);
        if (std::fabs(true_relative_pos.z()) < 0.2) {
            break;  // 提前结束
        }
    }
    // 验证逻辑：在这种极其变态的环境下，系统一定会有动态滞后误差。
    Eigen::Vector4d final_true_error = target_pos - drone_pos;
    // 对于 S 型变加速运动，卡尔曼滤波(恒速模型)天然存在滞后。
    // 如果最后误差能压在 0.35 米以内，说明这套控制器的跟踪能力已经是顶级的了！
    EXPECT_NEAR(final_true_error.x(), 0.0, 0.35);
    EXPECT_NEAR(final_true_error.y(), 0.0, 0.35);

    // 由于甲板在上下颠簸，无人机最后的高度应该在 0.15m ~ 0.55m 之间随甲板起伏
    double z = std::fmax(drone_pos.z(), 0.0);
    EXPECT_NEAR(z - target_pos.z(), 0.35, 0.2);
}

int main(int argc, char **argv) {
    // 初始化 Google Test
    testing::InitGoogleTest(&argc, argv);

    // 如果你以后需要在测试中用到 ROS 的时间 (ros::Time::now()) 等功能，
    // 可以把下面这行取消注释：
    // ros::init(argc, argv, "test_landing_controller_node");
    // 运行所有的 TEST() 用例
    return RUN_ALL_TESTS();
}