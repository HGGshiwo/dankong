#pragma once

#ifdef USE_ROS
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <rsos_msgs/PointObj.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include "dk/adapters/ros.hpp"
#endif

#include <Eigen/Dense>
#include <memory>

#include "./context.hpp"
#include "Eigen/src/Geometry/Quaternion.h"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/control/events.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"

class ControlFeature {
   public:
#ifdef USE_ROS
    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        ros->bind_event(
            "/restart",
            [](std_msgs::String::ConstPtr data) -> RestartEvent { return {}; });

        ros->bind_event("/UAV0/perception/object_location/location_vel",
                        [](rsos_msgs::PointObj::ConstPtr msg) -> DetectEvent {
                            DetectEvent e;
                            e.score = msg->score;
                            e.target_pos.x() = msg->pos.x;
                            e.target_pos.y() = msg->pos.y;
                            e.target_pos.z() = msg->pos.z;
                            e.cmd_vel.x() = msg->velocity.x;
                            e.cmd_vel.y() = msg->velocity.y;
                            e.cmd_vel.z() = msg->velocity.z;
                            return e;
                        });
    }
#endif

    static void setup(
        TagMavsdk,
        std::shared_ptr<dk::MavsdkAdapter<RobotContext, Engine>>& adapter) {
        adapter->bind_system_context(
            &mavsdk::System::subscribe_is_connected,
            [](bool is_connected, RobotContext& ctx) -> void {
                if (ctx.fcu_connected.load() != is_connected) {
                    ctx.fcu_connected.store(is_connected);
                    ctx.engine->dispatch_internal(
                        FcuConnectedEvent{is_connected});
                    spdlog::info("FCU Connection Status Changed: {}",
                                 is_connected);
                }
            });

        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_armed,
            [](bool is_armed, RobotContext& ctx) -> void {
                auto armed = ctx.arm.load();
                if (armed != is_armed) {
                    ctx.arm.store(is_armed);
                    ctx.engine->dispatch_internal(ArmEvent{is_armed});
                }
            });

        // 角速度
        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_attitude_angular_velocity_body,
            [](mavsdk::Telemetry::AngularVelocityBody ang_vel,
               RobotContext& ctx) -> void {
                ctx.vel_angular_body.store(Eigen::Vector3d{ang_vel.roll_rad_s,
                                                           ang_vel.pitch_rad_s,
                                                           ang_vel.yaw_rad_s});
            });

        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_flight_mode,
            [](mavsdk::Telemetry::FlightMode mode, RobotContext& ctx) -> void {
                std::string mode_str = "";
                // 核心修改：将 MAVSDK 映射的通用模式转换为 APM 专属名称
                switch (mode) {
                    case mavsdk::Telemetry::FlightMode::Unknown:
                        mode_str = "UNKNOWN";
                        break;
                    case mavsdk::Telemetry::FlightMode::Ready:
                        mode_str = "READY";
                        break;
                    case mavsdk::Telemetry::FlightMode::Takeoff:
                        mode_str = "GUIDED";
                        break;  // APM在外部控制起飞时也是 GUIDED
                    case mavsdk::Telemetry::FlightMode::Hold:
                        mode_str = "LOITER";
                        break;  // MAVSDK 会把 APM 的 LOITER 翻译成 Hold
                    case mavsdk::Telemetry::FlightMode::Mission:
                        mode_str = "AUTO";
                        break;  // 航线模式在 APM 叫 AUTO
                    case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                        mode_str = "RTL";
                        break;
                    case mavsdk::Telemetry::FlightMode::Land:
                        mode_str = "LAND";
                        break;
                    case mavsdk::Telemetry::FlightMode::Offboard:
                        mode_str = "GUIDED";
                        break;  // Offboard 等价于 APM 的 GUIDED 模式
                    case mavsdk::Telemetry::FlightMode::FollowMe:
                        mode_str = "FOLLOW";
                        break;
                    case mavsdk::Telemetry::FlightMode::Manual:
                        mode_str = "MANUAL";
                        break;
                    case mavsdk::Telemetry::FlightMode::Altctl:
                        mode_str = "ALT_HOLD";
                        break;
                    case mavsdk::Telemetry::FlightMode::Posctl:
                        mode_str = "POSHOLD";
                        break;
                    case mavsdk::Telemetry::FlightMode::Acro:
                        mode_str = "ACRO";
                        break;
                    case mavsdk::Telemetry::FlightMode::Stabilized:
                        mode_str = "STABILIZE";
                        break;
                }

                if (ctx.mode.load() != mode_str) {
                    ctx.engine->dispatch_internal(
                        FlightModeEvent{ctx.mode.load(), mode_str});
                    ctx.mode.store(mode_str);
                }
            });

        // 1. 订阅 Local Position NED 并转换为 ENU 坐标系 (替代 Odometry
        // 的位移和速度)
        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_position_velocity_ned,
            [](mavsdk::Telemetry::PositionVelocityNed pos_vel,
               RobotContext& ctx) -> void {
                // NED 转 ENU 坐标系映射规则:
                // ENU X (East)  = NED Y (East)
                // ENU Y (North) = NED X (North)
                // ENU Z (Up)    = -NED Z (Down)
                double enu_x = pos_vel.position.east_m;
                double enu_y = pos_vel.position.north_m;
                double enu_z = -pos_vel.position.down_m;

                ctx.pos_enu.emplace(enu_x, enu_y, enu_z);

                double vel_enu_x = pos_vel.velocity.east_m_s;
                double vel_enu_y = pos_vel.velocity.north_m_s;
                double vel_enu_z = -pos_vel.velocity.down_m_s;

                ctx.vel_enu.store(
                    Eigen::Vector3d{vel_enu_x, vel_enu_y, vel_enu_z});

                Eigen::Vector3d vel_ned(pos_vel.velocity.north_m_s,
                                        pos_vel.velocity.east_m_s,
                                        pos_vel.velocity.down_m_s);
                Eigen::Quaterniond current_q = ctx.orientation.load();
                ctx.vel_body.store(current_q.conjugate() * vel_ned);

                ctx.pose_history.push_pos(
                    ctx.engine->get_time_provider()->now(),
                    Eigen::Vector3d(enu_x, enu_y, enu_z));
            });

        // 2. 订阅 Attitude Quaternion 获取姿态 (替代 Odometry 中的四元数)
        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_attitude_euler,  // 订阅欧拉角
            [](mavsdk::Telemetry::EulerAngle euler, RobotContext& ctx) -> void {
                // 1. MAVSDK 返回的是角度 (deg)，先转换为弧度 (rad)
                // 注意：此时这三个角属于 NED 导航系 和 FRD 机体系
                double roll_ned_frd = euler.roll_deg * (M_PI / 180.0);
                double pitch_ned_frd = euler.pitch_deg * (M_PI / 180.0);
                double yaw_ned_frd = euler.yaw_deg * (M_PI / 180.0);

                // 2. 将原生的 NED_FRD 欧拉角转换为标准的乘积四元数
                // (使用固件默认的 Z-Y-X 顺序)
                Eigen::AngleAxisd rollAngle(roll_ned_frd,
                                            Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch_ned_frd,
                                             Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw_ned_frd,
                                           Eigen::Vector3d::UnitZ());
                Eigen::Quaterniond q_ned_frd =
                    yawAngle * pitchAngle * rollAngle;

                // 3. 构造世界坐标系转换矩阵：ENU 到 NED
                // NED: X=北, Y=东, Z=下 | ENU: X=东, Y=北, Z=上
                Eigen::Matrix3d R_enu_ned;
                R_enu_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
                Eigen::Quaterniond q_enu_ned(R_enu_ned);

                // 4. 构造机体坐标系转换：FRD 到 FLU (绕 X 轴旋转 180 度)
                Eigen::Quaterniond q_frd_flu(0, 1, 0, 0);  // w=0, x=1, y=0, z=0

                // 5. 核心复合转换：计算出纯粹的 ENU_FLU 状态下的四元数
                Eigen::Quaterniond q_enu_flu =
                    q_enu_ned * q_ned_frd * q_frd_flu;

                // 6. 从统一后的四元数中，提取出纯正的 ENU_FLU 欧拉角
                auto rpy_enu = state_utils::orientation_to_euler(
                    q_enu_flu.x(), q_enu_flu.y(), q_enu_flu.z(), q_enu_flu.w());

                // 7. 统一存储到 Context 中 (此时内部数据 100% 符合 ENU + FLU
                // 标准)
                ctx.yaw_enu = rpy_enu.z();     // 真正的 ENU 偏航角
                ctx.pitch.store(rpy_enu.y());  // 真正的 FLU 俯仰角
                ctx.roll.store(rpy_enu.x());   // 真正的 FLU 横滚角

                ctx.orientation.store(q_enu_flu);  // 存储 ENU_FLU 四元数
                ctx.pose_history.push_quat(
                    ctx.engine->get_time_provider()->now(), q_enu_flu);

                // 8. 如果你的旧模块依然需要原始的 NED
                // 偏航角，直接存入最开始转好的弧度即可
                ctx.yaw_ned.store(yaw_ned_frd);
            });

        // // 注意：APM 默认可能不发送 ODOMETRY 消息（MAVLink MSG 331）
        // // 请确保在 APM 参数中设置了适当的消息回传频率，或者 APM 正在发布
        // LOCAL_POSITION_NED
        // adapter->bind_telemetry_context(&mavsdk::Telemetry::subscribe_odometry,
        //     [](mavsdk::Telemetry::Odometry odom, RobotContext& ctx) -> void {
        //         auto q = odom.q;
        //         auto p = odom.position_body;

        //         ctx.pos_enu.emplace(p.x_m, p.y_m, p.z_m);

        //         auto rpy = state_utils::orientation_to_euler(
        //             q.x, q.y, q.z, q.w);

        //         ctx.yaw_enu = rpy.z();
        //         ctx.pitch.store(rpy.y());
        //         ctx.roll.store(rpy.x());
        //         ctx.orientation.store(
        //             Eigen::Quaterniond(q.w, q.x, q.y, q.z));
        //         ctx.yaw_ned.store(state_utils::yaw_enu_to_ned(rpy.z()));

        //         // 将当前位姿存入历史记录
        //         ctx.pose_history.push(
        //             std::chrono::duration_cast<std::chrono::duration<double>>(
        //                 std::chrono::system_clock::now().time_since_epoch()).count(),
        //             Eigen::Vector3d(p.x_m, p.y_m, p.z_m),
        //             Eigen::Quaterniond(q.w, q.x, q.y, q.z));
        //     });

        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_position,
            [](mavsdk::Telemetry::Position pos, RobotContext& ctx) -> void {
                ctx.lon_lat_alt.write([&pos](Eigen::Vector3d& data) {
                    data.x() = pos.longitude_deg;
                    data.y() = pos.latitude_deg;
                    data.z() =
                        pos.relative_altitude_m;  // 注意：APM 的 relative_alt
                                                  // 通常是相对于 home 点
                });
                ctx.odom_ok = true;
            });

        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_gps_info,
            [](mavsdk::Telemetry::GpsInfo msg, RobotContext& ctx) -> void {
                ctx.gps_fix_type.store(static_cast<int>(msg.fix_type));
                ctx.gps_nsats.store(msg.num_satellites);
            });

        adapter->bind_telemetry_context(
            &mavsdk::Telemetry::subscribe_battery,
            [](mavsdk::Telemetry::Battery battery, RobotContext& ctx) -> void {
                ctx.battery_remaining.store(battery.remaining_percent);
            });

        adapter->bind_passthrough_event(
            1, [](const mavlink_message_t& msg) -> SysStatusEvent {
                mavlink_sys_status_t sys_status;
                mavlink_msg_sys_status_decode(&msg, &sys_status);

                // onboard_control_sensors_health 就是 MAVROS 里的
                // status->sensors_health
                return SysStatusEvent{
                    sys_status.onboard_control_sensors_health};
            });

        adapter->bind_telemetry_event(
            &mavsdk::Telemetry::subscribe_status_text,
            [](mavsdk::Telemetry::StatusText status_text) -> StatusTextEvent {
                bool should_report = false;
                switch (status_text.type) {
                    case mavsdk::Telemetry::StatusTextType::Error:
                        spdlog::error("[ERROR] FCU: {}", status_text.text);
                        should_report = true;
                        break;

                    case mavsdk::Telemetry::StatusTextType::Warning:
                        spdlog::error("[WARN] FCU: {}", status_text.text);
                        should_report = true;
                        break;

                    case mavsdk::Telemetry::StatusTextType::Info:
                        spdlog::info("[INFO] FCU: {}", status_text.text);
                        break;

                    case mavsdk::Telemetry::StatusTextType::Critical:
                    case mavsdk::Telemetry::StatusTextType::Emergency:
                    case mavsdk::Telemetry::StatusTextType::Alert:
                        spdlog::error("[EMERGENCY] FCU: {}", status_text.text);
                        should_report = true;
                        break;
                    default:
                        // 覆盖 Notice, Debug 等级别
                        spdlog::info("[OTHER] FCU: {}", status_text.text);
                        break;
                }

                return StatusTextEvent{status_text.text, should_report};
            });
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        web->template register_route<PrearmEvent, EventResult>(
            boost::beast::http::verb::get, "/prearms", 5000);

        web->template register_route<TakeoffEvent, EventResult>(
            boost::beast::http::verb::post, "/takeoff", 5000);

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/set_waypoint", 5000);

        web->template register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/set_mode", 5000);

        // Web 端发送 LOITER 模式，APM 完全兼容这个名称
        web->template register_route<SetModeEvent, EventResult>(
            boost::beast::http::verb::post, "/loiter", 5000,
            [](SetModeEvent& event) { event.mode = "LOITER"; });

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/return", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::RETURN;
            });

        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/land", 5000,
            [](SetWaypointEvent& event) {
                event.finish_action = FinishAction::LAND;
            });

        web->template register_route<RebootFcuEvent, EventResult>(
            boost::beast::http::verb::post, "/reboot_fcu");

        web->template register_route<GetWpEvent, EventResult>(
            boost::beast::http::verb::get, "/get_waypoint");

        web->template register_route<GetGpsEvent, EventResult>(
            boost::beast::http::verb::get, "/get_gps");

        web->template register_route<GetParamEvent, EventResult>(
            boost::beast::http::verb::get, "/params");

        web->template register_route<SetParamEvent, EventResult>(
            boost::beast::http::verb::post, "/set_param");

        web->template register_route<DisarmEvent, EventResult>(
            boost::beast::http::verb::post, "/disarm");

        // 该接口只是调试时候使用
        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/pland", 5000,
            [](SetWaypointEvent& event) {
                event.land_target_id = 0;
                event.finish_action = FinishAction::LAND;
            });

        web->template register_route<SetPosVelEvent, EventResult>(
            boost::beast::http::verb::post, "/set_posvel", 5000);
    }

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine);
};

#include "./event_listener.hpp"
inline void ControlFeature::setup(TagListeners,
                                  const std::shared_ptr<Engine>& engine) {
    auto listener = std::make_shared<ControlEventListener>();
    engine->add_listener(listener);
}