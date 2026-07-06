#pragma once
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "./event_listener.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "features/control/events.hpp"
#include "robot/go2.hpp"
#include "robot_context.hpp"

class Go2Feature {
   public:
    static void setup(TagInit, RobotContext& ctx) {
        auto mavlink = std::make_shared<MavsdkDrone>(
            ctx.engine->get_context().mavsdk_system);

        ctx.robot =
            std::make_shared<Go2>(std::make_shared<MavsdkDrone>(
                                      ctx.engine->get_context().mavsdk_system),
                                  ctx);
        ctx.use_fcu_enu.store(false);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        engine->add_listener(std::make_shared<Go2EventListener>());
    }

    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        ros->bind_context(
            "/loc_base",
            [](const nav_msgs::Odometry::ConstPtr& msg,
               RobotContext& ctx) -> void {
                // 0. 统一获取当前时间戳
                auto now = ctx.engine->get_time_provider()->now();

                // ==========================================
                // 1. 姿态更新 (Orientation) - 全部来自 msg
                // ==========================================
                // 提取 odom 消息中的姿态四元数 (ROS 标准默认即为 ENU-FLU)
                // 注意 Eigen::Quaterniond 的构造顺序是 (w, x, y, z)
                Eigen::Quaterniond q_enu_flu(
                    msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

                // 归一化以防止传感器数据的浮点误差在后续乘法中放大
                q_enu_flu.normalize();

                // 从标准的四元数中提取出 ENU_FLU 欧拉角
                auto rpy_enu = state_utils::orientation_to_euler(
                    q_enu_flu.x(), q_enu_flu.y(), q_enu_flu.z(), q_enu_flu.w());

                // 存储 ENU/FLU 标准欧拉角
                ctx.yaw_enu = rpy_enu.z();
                ctx.pitch.store(rpy_enu.y());
                ctx.roll.store(rpy_enu.x());

                // 【兼容旧逻辑】如果你仍然需要 NED 系的 Yaw 角：
                // 纯数学转换：NED 的北(0度) 对应 ENU 的
                // Y轴(90度)，且旋转方向相反 Yaw_NED = 90度(pi/2) - Yaw_ENU
                double yaw_ned = M_PI_2 - rpy_enu.z();
                // 将角度限制在 [-pi, pi] 之间
                if (yaw_ned > M_PI) yaw_ned -= 2.0 * M_PI;
                if (yaw_ned < -M_PI) yaw_ned += 2.0 * M_PI;
                ctx.yaw_ned.store(yaw_ned);

                // 存储四元数状态
                ctx.orientation.store(q_enu_flu);
                ctx.pose_history.push_quat(now, q_enu_flu);

                // ==========================================
                // 2. 位置更新 (Position) - 全部来自 msg
                // ==========================================
                double enu_x = msg->pose.pose.position.x;
                double enu_y = msg->pose.pose.position.y;
                double enu_z = msg->pose.pose.position.z;

                ctx.pos_enu.emplace(enu_x, enu_y, enu_z);
                ctx.datum.pushENU({enu_x, enu_y, enu_z}, now);
                ctx.pose_history.push_pos(now,
                                          Eigen::Vector3d(enu_x, enu_y, enu_z));

                // ==========================================
                // 3. 速度更新 (Velocity) - 全部来自 msg
                // ==========================================
                // Odom 的 Twist.linear 通常是在世界坐标系(ENU)下的线速度
                double vel_enu_x = msg->twist.twist.linear.x;
                double vel_enu_y = msg->twist.twist.linear.y;
                double vel_enu_z = msg->twist.twist.linear.z;

                Eigen::Vector3d vel_enu{vel_enu_x, vel_enu_y, vel_enu_z};
                ctx.vel_enu.store(vel_enu);

                // 核心计算：使用 odom 原生的 ENU
                // 速度，配合当前的姿态，求出机体系(Body/FLU)速度 q_enu_flu 是从
                // Body(FLU) 到 World(ENU) 的旋转 它的共轭 (conjugate) 就是从
                // World(ENU) 回到 Body(FLU)
                ctx.vel_body.store(q_enu_flu.conjugate() * vel_enu);
            });

        ros->bind_event(
            "/task/feedback",
            [](const std_msgs::String::ConstPtr& msg) -> WpArriveEvent {
                return WpArriveEvent{};
            });
    };
};