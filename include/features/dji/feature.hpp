#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/src/Geometry/Quaternion.h"
#include "command.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "features/control/events.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"

class DjiFeature {
   public:
    // 将常见的 DJI 模式枚举映射为系统内部状态字符串
    static std::string parse_flight_mode(uint8_t mode_id) {
        switch (mode_id) {
            case 1:
                return "GUIDED";  // 约定 1 为 VirtualStick 模式
            case 2:
                return "AUTO";  // 航点任务
            case 3:
                return "RTL";  // 返航
            case 4:
                return "LOITER";  // 悬停
            case 5:
                return "MANUAL";  // 手动
            default:
                return "UNKNOWN";
        }
    }

    static void setup(
        TagUdp,
        std::shared_ptr<dk::UdpAdapter<RobotContext, Engine, MsgId>>& udp) {
        // ====================================================================
        // 1. 绑定 Context 更新 (高频遥测，更新状态和运动学数据)
        // ====================================================================
        udp->bind_context(
            MsgId::TELEMETRY_HIGH_HZ,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                // 校验长度
                if (data.size() <
                    sizeof(PacketHeader) + sizeof(TelemetryHighHz))
                    return;

                const auto* tlm = reinterpret_cast<const TelemetryHighHz*>(
                    data.data() + sizeof(PacketHeader));

                // --- 状态与模式更新 ---
                bool is_connected = (tlm->is_connected != 0);
                if (ctx.fcu_connected.load() != is_connected) {
                    ctx.fcu_connected.store(is_connected);
                    ctx.engine->dispatch_internal(
                        FcuConnectedEvent{is_connected});
                    spdlog::info("FCU Connection Status Changed: {}",
                                 is_connected);
                }

                bool is_armed = (tlm->is_armed != 0);
                if (ctx.arm.load() != is_armed) {
                    ctx.arm.store(is_armed);
                    ctx.engine->dispatch_internal(ArmEvent{is_armed});
                }

                std::string mode_str = parse_flight_mode(tlm->flight_mode);
                if (ctx.mode.load() != mode_str) {
                    ctx.engine->dispatch_internal(
                        FlightModeEvent{ctx.mode.load(), mode_str});
                    ctx.mode.store(mode_str);
                }

                // --- 运动学数据更新 (极致精简) ---
                ctx.pos_enu.emplace(tlm->pos_enu_x, tlm->pos_enu_y,
                                    tlm->pos_enu_z);
                ctx.datum.pushENU(
                    {tlm->pos_enu_x, tlm->pos_enu_y, tlm->pos_enu_z},
                    ctx.engine->get_time_provider()->now());

                Eigen::Vector3d vel_enu(tlm->vel_enu_x, tlm->vel_enu_y,
                                        tlm->vel_enu_z);
                ctx.vel_enu.store(vel_enu);

                ctx.vel_angular_body.store(Eigen::Vector3d{
                    tlm->ang_vel_x, tlm->ang_vel_y, tlm->ang_vel_z});

                Eigen::Quaterniond q_enu_flu(tlm->q_w, tlm->q_x, tlm->q_y,
                                             tlm->q_z);
                ctx.orientation.store(q_enu_flu);

                auto rpy_enu = state_utils::orientation_to_euler(
                    q_enu_flu.x(), q_enu_flu.y(), q_enu_flu.z(), q_enu_flu.w());
                ctx.yaw_enu = rpy_enu.z();
                ctx.pitch.store(rpy_enu.y());
                ctx.roll.store(rpy_enu.x());

                ctx.vel_body.store(q_enu_flu.conjugate() * vel_enu);

                auto now = ctx.engine->get_time_provider()->now();
                ctx.pose_history.push_pos(
                    now, Eigen::Vector3d(tlm->pos_enu_x, tlm->pos_enu_y,
                                         tlm->pos_enu_z));
                ctx.pose_history.push_quat(now, q_enu_flu);

                ctx.lon_lat_alt.write([tlm](Eigen::Vector3d& d) {
                    d.x() = tlm->longitude;
                    d.y() = tlm->latitude;
                    d.z() = tlm->relative_alt;
                });

                ctx.datum.pushGPS(
                    {tlm->longitude, tlm->latitude, tlm->relative_alt},
                    ctx.engine->get_time_provider()->now());
                ctx.odom_ok = true;
            });

        // ====================================================================
        // 2. 绑定 Context 更新 (低频遥测，更新电量和GPS等)
        // ====================================================================
        udp->bind_context(
            MsgId::TELEMETRY_LOW_HZ,
            [](const std::vector<uint8_t>& data, RobotContext& ctx) {
                if (data.size() < sizeof(PacketHeader) + sizeof(TelemetryLowHz))
                    return;

                const auto* tlm = reinterpret_cast<const TelemetryLowHz*>(
                    data.data() + sizeof(PacketHeader));

                ctx.battery_remaining.store(tlm->battery_percent);
                ctx.gps_fix_type.store(tlm->gps_fix_type);
                ctx.gps_nsats.store(tlm->gps_nsats);

                // 健康状态虽然属于 Context，但也触发事件以便响应
                ctx.engine->dispatch_internal(
                    SysStatusEvent{tlm->sensor_health_bitmask});
            });

        // ====================================================================
        // 3. 绑定 Event (报警文本，仅作为纯事件派发)
        // ====================================================================
        udp->bind_event(
            MsgId::STATUS_TEXT,
            [](const std::vector<uint8_t>& data) -> StatusTextEvent {
                if (data.size() <
                    sizeof(PacketHeader) + sizeof(StatusTextPayload)) {
                    return StatusTextEvent{"", false};
                }

                const auto* tlm = reinterpret_cast<const StatusTextPayload*>(
                    data.data() + sizeof(PacketHeader));

                std::string text_safe(tlm->text,
                                      strnlen(tlm->text, sizeof(tlm->text)));
                bool should_report = false;

                switch (tlm->severity) {
                    case 2:  // Error
                    case 3:  // Critical
                        spdlog::error("[DJI MSDK ERROR] {}", text_safe);
                        should_report = true;
                        break;
                    case 1:  // Warning
                        spdlog::warn("[DJI MSDK WARN] {}", text_safe);
                        should_report = true;
                        break;
                    default:  // Info
                        spdlog::info("[DJI MSDK INFO] {}", text_safe);
                        break;
                }

                // Adapter 会自动将返回的 Event 派发到 Engine 总线中
                return StatusTextEvent{text_safe, should_report};
            });
    }

    // 挂载特定的 Listener (如果后续有需要的话)
    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        // engine->add_listener(std::make_shared<DjiUdpEventListener>(engine->get_context()));
    }
};