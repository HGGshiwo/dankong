#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>  // 用于清理输入流
#include <thread>

#include "features/tracker/tracker.hpp"

// 1. 模拟底层硬件接口
class SimRuntime : public ITrackerRuntime {
   private:
    std::mutex mtx_;
    Eigen::Vector4d current_cmd_ = Eigen::Vector4d::Zero();

   public:
    bool cmd_vel(Eigen::Vector4d cmd) override {
        std::lock_guard<std::mutex> lock(mtx_);
        current_cmd_ = cmd;
        return true;
    }
    Eigen::Vector4d get_latest_cmd() {
        std::lock_guard<std::mutex> lock(mtx_);
        return current_cmd_;
    }
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Starting C++ Tracker Simulation...    " << std::endl;
    std::cout << "========================================\n" << std::endl;

    TrackerConfig config;
    config.loop_rate_hz.set(50);  // 频率通常固定， 无需每次输入

    // ==========================================
    // 新增：从标准输入读取关键配置
    // ==========================================
    std::cout << "--- Please enter configuration ---\n";

    int omni_input;
    std::cout
        << "1. Is omnidirectional? (0 for false, 1 for true) [default 0]: ";
    if (std::cin >> omni_input) {
        config.is_omnidirectional = (omni_input != 0);
    }

    std::cout << "2. Auto heading enable distance in meters (e.g., 1.0): ";
    double dist_input;
    if (std::cin >> dist_input) {
        config.auto_heading_enable_dist_m = dist_input;
    }

    double target_x = 0.0, target_y = 0.0;
    std::cout << "3. Target Position X (e.g., -1.5): ";
    std::cin >> target_x;

    std::cout << "4. Target Position Y (e.g., 1.5): ";
    std::cin >> target_y;

    std::cout << "----------------------------------\n";
    std::cout << "Configuration Loaded:" << std::endl;
    std::cout << " - Omnidirectional: "
              << (config.is_omnidirectional ? "True" : "False") << std::endl;
    std::cout << " - Auto-heading Dist: " << config.auto_heading_enable_dist_m
              << " m" << std::endl;
    std::cout << " - Target Pos: (" << target_x << ", " << target_y << ")\n"
              << std::endl;
    // ==========================================

    // 2. 初始化环境与状态变量
    SimRuntime sim_runtime;
    DirtyVar<Eigen::Vector3d> pos_var{
        Eigen::Vector3d::Zero(),
    };
    std::atomic<double> yaw_var{0.0};

    // 3. 实例化你的控制器并启动线程
    ThreadedTracker tracker(config, &sim_runtime, pos_var, yaw_var, false);
    tracker.start(50);

    // 4. 下发目标点 (使用刚刚输入的参数)
    Eigen::Vector3d target_pos(target_x, target_y, 0.0);
    tracker.send_pos_cmd(target_pos, std::nullopt, std::nullopt, std::nullopt,
                         std::nullopt, std::nullopt, CmdFrame::ENU);

    // 5. 准备记录 CSV 数据
    std::ofstream csv_file("trajectory.csv");
    csv_file << "t,x,y,yaw,vx,vy,vw\n";

    // 6. 开启物理引擎仿真循环 (运行 15 秒)
    double t = 0.0;
    double dt = 0.02;  // 50Hz 物理仿真
    Eigen::Vector3d sim_pos(0.0, 0.0, 0.0);
    double sim_yaw = 0.0;

    auto start_time = std::chrono::steady_clock::now();

    while (t <= 15.0) {
        // a. 获取控制器的最新指令
        Eigen::Vector4d cmd = sim_runtime.get_latest_cmd();

        // b. 物理学更新 (欧拉积分) -> 模拟无人机的真实运动
        double dx_world =
            cmd.x() * std::cos(sim_yaw) - cmd.y() * std::sin(sim_yaw);
        double dy_world =
            cmd.x() * std::sin(sim_yaw) + cmd.y() * std::cos(sim_yaw);

        sim_pos.x() += dx_world * dt;
        sim_pos.y() += dy_world * dt;
        sim_pos.z() += cmd.z() * dt;
        sim_yaw += cmd.w() * dt;

        // 限制 yaw 在 -PI 到 PI 之间
        while (sim_yaw > M_PI) sim_yaw -= 2.0 * M_PI;
        while (sim_yaw < -M_PI) sim_yaw += 2.0 * M_PI;

        // c. 将传感器读数更新回 Tracker
        pos_var.store(sim_pos);
        yaw_var.store(sim_yaw);

        // d. 记录到 CSV
        csv_file << t << "," << sim_pos.x() << "," << sim_pos.y() << ","
                 << sim_yaw << "," << cmd.x() << "," << cmd.y() << ","
                 << cmd.w() << "\n";

        // e. 检查是否完全刹停
        double dist = std::hypot(target_pos.x() - sim_pos.x(),
                                 target_pos.y() - sim_pos.y());
        if (dist < 0.05 && cmd.norm() < 0.001 && t > 2.0) {
            std::cout << "Target Reached and Stopped at t = " << t << "s"
                      << std::endl;
            break;
        }

        t += dt;

        tracker.on_step(dt);

        // 让物理循环与真实时间同步
        // std::this_thread::sleep_until(
        //     start_time + std::chrono::milliseconds(static_cast<int>(t *
        //     1000)));
    }

    tracker.stop();
    csv_file.close();
    std::cout << "Simulation finished. Data saved to trajectory.csv"
              << std::endl;
    return 0;
}