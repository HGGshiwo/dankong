#pragma once

#ifdef USE_ROS1
#include <mavsdk/plugins/gimbal/gimbal.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <ros/ros.h>
#include <rsos_msgs/SetGimbalAngle.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include "core/global_config.hpp"
#include "robot_context.hpp"
#include "spdlog/spdlog.h"

// rsos 云台服务桥: 对外提供与串口云台机型一致的
// /UAV0/sensor/serial_gimbal/set_gimbal_angle (rsos_msgs/SetGimbalAngle)。
//
// 纯转发节点: 云台语义 (YAW_FOLLOW、pitch/roll 世界系、一次性指令)
// 全部由 MavDrone App 侧实现, 本节点只做符号约定转换和协议转发,
// 不包含任何机型相关的知识。
//   mode 约定: "body"/"abs" 均接受, 行为一致 (与其他机型的接口兼容) —
//   yaw 随机体, pitch/roll 世界系; angle 正值向下 (90 = 俯视),
//   转换为 DJI 负值向下后透传
class GimbalBridge {
   public:
    GimbalBridge(RobotContext& ctx, std::shared_ptr<mavsdk::System> system)
        : ctx_(ctx),
          system_(system),
          gimbal_(std::make_shared<mavsdk::Gimbal>(system)),
          passthrough_(std::make_shared<mavsdk::MavlinkPassthrough>(system)) {
        gimbal_service_topic_ =
            GlobalConfig.GetConfig().gimbal_service_topic.get();

        // 防冲突: 其他机型上该 service 由串口云台驱动节点提供,
        // 若已存在同名 service 则不重复注册, 避免调用随机命中两个 provider
        if (ros::service::exists(gimbal_service_topic_, false)) {
            spdlog::warn(
                "[GimbalBridge] service {} already provided by another node, "
                "gimbal bridge disabled",
                gimbal_service_topic_);
            return;
        }

        // 云台姿态回传诊断 (GIMBAL_DEVICE_ATTITUDE_STATUS, App 10Hz):
        // 确认实际云台模式/角度是否与指令一致
        passthrough_->subscribe_message(
            282, [this](const mavlink_message_t& msg) {
                if (!first_282_logged_.exchange(true)) {
                    spdlog::info(
                        "[GimbalBridge] first GIMBAL_DEVICE_ATTITUDE_STATUS "
                        "received");
                }
                msg282_count_++;
                mavlink_gimbal_device_attitude_status_t st;
                mavlink_msg_gimbal_device_attitude_status_decode(&msg, &st);
                double roll = std::atan2(
                    2.0 * (st.q[0] * st.q[1] + st.q[2] * st.q[3]),
                    1.0 - 2.0 * (st.q[1] * st.q[1] + st.q[2] * st.q[2]));
                double pitch =
                    std::asin(2.0 * (st.q[0] * st.q[2] - st.q[3] * st.q[1]));
                double yaw = std::atan2(
                    2.0 * (st.q[0] * st.q[3] + st.q[1] * st.q[2]),
                    1.0 - 2.0 * (st.q[2] * st.q[2] + st.q[3] * st.q[3]));

                auto now = ros::Time::now();
                if ((now - last_gimbal_log_).toSec() >= 2.0) {
                    spdlog::info(
                        "[GimbalBridge] gimbal rpy=({:.1f}, {:.1f}, {:.1f}) "
                        "mode={} msgs2s={}",
                        roll * 180.0 / M_PI, pitch * 180.0 / M_PI,
                        yaw * 180.0 / M_PI,
                        last_dji_mode_.empty() ? "-" : last_dji_mode_,
                        msg282_count_.exchange(0));
                    last_gimbal_log_ = now;
                }
            });

        srv_ = nh_.advertiseService(gimbal_service_topic_,
                                    &GimbalBridge::set_gimbal_angle_cb, this);
        spdlog::info("[GimbalBridge] serving {}", gimbal_service_topic_);
    }

    ~GimbalBridge() { srv_.shutdown(); }

   private:
    bool set_gimbal_angle_cb(rsos_msgs::SetGimbalAngle::Request& req,
                             rsos_msgs::SetGimbalAngle::Response& res) {
        if (req.mode != "body" && req.mode != "abs") {
            res.success = false;
            res.message = "unsupported mode: " + req.mode + " (body/abs)";
            return true;
        }

        std::lock_guard<std::mutex> lock(mutex_);

        if (!ensure_control()) {
            res.success = false;
            res.message = "failed to take gimbal control";
            return true;
        }

        if (!apply_dji_mode(req.mode)) {
            res.success = false;
            res.message = "failed to set gimbal mode";
            return true;
        }

        // 符号约定转换: rsos 侧 angle 为正值向下 (90 = 俯视),
        // DJI pitch 为负值向下; 越界按机型可控范围 clamp
        const float pitch_dji = std::clamp(
            static_cast<float>(-req.angle),
            static_cast<float>(GlobalConfig.GetConfig().gimbal_pitch_min.get()),
            static_cast<float>(
                GlobalConfig.GetConfig().gimbal_pitch_max.get()));

        auto result = send_pitch_only(pitch_dji);
        release_control();

        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            spdlog::warn("[GimbalBridge] set_pitch({:.1f}, {}) failed: {}",
                         req.angle, req.mode, static_cast<int>(result));
            res.success = false;
            res.message = "set gimbal angle failed";
            return true;
        }

        // 回写参数, 供 GetGimbalEvent 查询
        nh_.setParam("/UAV0/sensor/serial_gimbal/angle_mode", req.mode);
        nh_.setParam("/UAV0/sensor/serial_gimbal/gimbal_angle", req.angle);

        res.success = true;
        res.message = "OK";
        return true;
    }

    // 语义定稿: body/abs 不再区分, 一律 DJI YAW_FOLLOW (yaw 随机体;
    // pitch/roll 世界系稳定, pitch 为一次性世界角, 语义由 MavDrone 实现)。
    // mount mode RC_TARGETING(3) → DJI YAW_FOLLOW
    bool apply_dji_mode(const std::string& mode) {
        if (mode == last_dji_mode_) return true;

        mavsdk::MavlinkPassthrough::CommandLong cmd{};
        cmd.target_sysid = passthrough_->get_target_sysid();
        cmd.target_compid = 154;  // MAV_COMP_ID_GIMBAL
        cmd.command = 204;        // MAV_CMD_DO_MOUNT_CONFIGURE
        cmd.param4 = 3.0f;        // RC_TARGETING -> DJI YAW_FOLLOW
        cmd.param7 = 0;           // any gimbal device

        auto result = passthrough_->send_command_long(cmd);
        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            spdlog::warn("[GimbalBridge] set dji mode '{}' failed: {}", mode,
                         static_cast<int>(result));
            return false;
        }
        last_dji_mode_ = mode;
        spdlog::info("[GimbalBridge] dji gimbal mode -> {}", mode);
        return true;
    }

    static const char* result_str(mavsdk::Gimbal::Result r) {
        switch (r) {
            case mavsdk::Gimbal::Result::Unknown:
                return "Unknown";
            case mavsdk::Gimbal::Result::Success:
                return "Success";
            case mavsdk::Gimbal::Result::Error:
                return "Error";
            case mavsdk::Gimbal::Result::Timeout:
                return "Timeout";
            case mavsdk::Gimbal::Result::Unsupported:
                return "Unsupported";
            case mavsdk::Gimbal::Result::NoSystem:
                return "NoSystem";
            case mavsdk::Gimbal::Result::InvalidArgument:
                return "InvalidArgument";
            default:
                return "?";
        }
    }

    bool ensure_control() {
        if (control_taken_) return true;

        // 广播寻址 (gimbal_id=0) 为首选, 配合 MavDrone 的 ACK 竞争修复
        auto r0 =
            gimbal_->take_control(0, mavsdk::Gimbal::ControlMode::Primary);
        if (r0 == mavsdk::Gimbal::Result::Success) {
            control_taken_ = true;
            return true;
        }
        spdlog::warn(
            "[GimbalBridge] take_control(0) failed: {} ({}), "
            "falling back to targeted",
            static_cast<int>(r0), result_str(r0));

        auto r1 =
            gimbal_->take_control(1, mavsdk::Gimbal::ControlMode::Primary);
        if (r1 != mavsdk::Gimbal::Result::Success) {
            spdlog::warn("[GimbalBridge] take_control(1) failed: {} ({})",
                         static_cast<int>(r1), result_str(r1));
            return false;
        }
        control_taken_ = true;
        return true;
    }

    void release_control() {
        if (!control_taken_) return;
        auto result = gimbal_->release_control(0);
        if (result != mavsdk::Gimbal::Result::Success) {
            spdlog::warn("[GimbalBridge] release_control failed: {}",
                         static_cast<int>(result));
        }
        control_taken_ = false;
    }

    // pitch-only 透传: DO_GIMBAL_MANAGER_PITCHYAW (指令号 1000) 没有
    // roll 参数, 不会干扰 FPV 模式的 roll 随机体行为。
    // body/abs 的参考系语义由 MavDrone 按 mount mode 解释
    mavsdk::MavlinkPassthrough::Result send_pitch_only(float pitch_dji) {
        mavsdk::MavlinkPassthrough::CommandLong cmd{};
        cmd.target_sysid = passthrough_->get_target_sysid();
        cmd.target_compid = 154;  // MAV_COMP_ID_GIMBAL
        cmd.command = 1000;       // MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        cmd.param1 = pitch_dji;
        cmd.param2 = NAN;  // yaw 不动
        cmd.param3 = NAN;  // pitch rate 不用
        cmd.param4 = NAN;  // yaw rate 不用
        // 枚举值: ROLL_LOCK=4, PITCH_LOCK=8 (误写会命中 MavDrone 的
        // NEUTRAL/RETRACT 分支)
        cmd.param5 = 4 | 8;
        cmd.param7 = 0;  // any gimbal device
        return passthrough_->send_command_long(cmd);
    }

    RobotContext& ctx_;
    std::shared_ptr<mavsdk::System> system_;
    std::shared_ptr<mavsdk::Gimbal> gimbal_;
    std::shared_ptr<mavsdk::MavlinkPassthrough> passthrough_;
    std::mutex mutex_;  // 序列化 MAVSDK 同步调用
    bool control_taken_ = false;
    std::string last_dji_mode_;

    // 云台姿态回读诊断
    std::atomic<bool> first_282_logged_{false};
    std::atomic<int> msg282_count_{0};
    ros::Time last_gimbal_log_;

    ros::NodeHandle nh_;
    std::string gimbal_service_topic_;
    ros::ServiceServer srv_;
};
#endif  // USE_ROS1
