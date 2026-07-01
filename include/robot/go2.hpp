#pragma once
#include <bitset>
#include <boost/beast/http/verb.hpp>
#include <memory>
#include <optional>

#include "dk/adapters/udp/udp_client.hpp"
#include "dk/future.hpp"
#include "features/dog/command.hpp"
#include "features/tracker/tracker.hpp"
#include "robot/irobot.hpp"
#include "robot_context.hpp"
#include "spdlog/fmt/bundled/format.h"
#include "utils/fixed_string64.hpp"
#include "utils/request.hpp"

// 机器狗硬件逻辑实现
class Go2 : public IRobot {
   private:
    RobotContext& ctx_;
    std::atomic<bool> standing_{false};

   public:
    Go2(RobotContext& ctx) : IRobot(nullptr), ctx_(ctx) {
        ctx.set_waypoint_goal = [engine = ctx.engine](
                                    Eigen::Vector3d target_enu,
                                    std::optional<double> vel) {
            std::string url = fmt::format("/push_message?x={}&y={}",
                                          target_enu.x(), target_enu.y());
            send_request(engine, http::verb::get, "127.0.0.1", url, 8444)
                .then([](HttpResponse res) {
                    spdlog::info("success={}, data={}, error={}", res.success(),
                                 res.body, res.error_msg);
                });
        };
    }

    // 计算目标距离（无人机与机器狗在 Z 轴上的处理不同）
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
        return (pos - goal).head<2>().norm();
    };

    // 降落（无人机切换 LAND 模式，机器狗发送蹲下指令）
    bool land() {
        // 可能也是一个http
        return true;
    };

    bool is_prearm_enable() { return false; };

    bool is_alt_enable() { return false; };

    bool loiter() { return true; };

    bool takeoff(double alt) {
        // 可能也是一个http
        return true;
    };

    bool cmd_vel(Eigen::Vector4d) {
        return true;  // 不支持
    };

    bool inner_check_hover() { return standing_; };

    bool inner_is_landed() { return !inner_check_hover(); };

    bool set_mode(const FixedString64& mode) { return true; }
    void set_target_type(VehicleType type) {}

    bool set_stream_rate(int stream_id, int rate) { return true; }

    bool set_msg_interval(int stream_id, int rate) { return true; }

    bool arm() { return true; }

    bool disarm() { return true; }

    bool reboot_fcu() { return true; }

    bool run_prearm_checks() { return true; }

    bool pull_params() { return true; }

    void send_rtcm_data(const uint8_t* data, size_t size) {}

    nlohmann::json get_all_params() { return {}; }

    bool set_param(std::string name, ApmParam value) { return true; }

    bool is_prearm_msg(const std::string& text) { return false; }

    bool check_sensor_health(uint32_t sensor_health) { return false; }
};
