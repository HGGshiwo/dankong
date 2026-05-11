#include "states/state_utils.hpp"

namespace state_utils {
bool is_prearm_msg(const std::string& text) {
    return text.rfind("PreArm: ", 0) == 0 || text.rfind("Arm: ", 0) == 0;
}

dk::Future<bool> set_mode(RobotContext& ctx, FixedString64 mode) {
    using Promise = dk::Promise<bool>;
    if (ctx.mode.get() == mode) return Promise::resolve(ctx.engine, true);
    ctx.robot->set_mode(mode);

    return ctx.engine->wait_for(1000, [mode](const FlightModeEvent& e) -> bool {
        if (mode != e.cur) return false;
        return true;
    });
}

// 判断是否真的需要进行prearm_check
bool should_do_prearm_check(RobotContext& ctx) {
    if (!ctx.robot->is_prearm_enable()) {
        return false;
    }
    ApmParam param = ctx.robot->get_param("ARMING_CHECK", 0);
    int value = IMavlink::unpack<int>(param);
    spdlog::info("prearm_check: ARMING_CHECK={}", value);
    if (value == 0) return false;
    return true;
}

bool check_sensor_health(uint32_t sensor_health) {
    int bits = 0x10000000;
    if ((sensor_health & bits) == bits) {
        spdlog::info("prearm_check: sensor_health check pass!");
        return true;
    }
    return false;
};

// TODO: 带上一个停止token，如果重复调用就返回或者做别的事情
dk::Future<bool> prearm_check(RobotContext& ctx) {
    if (!should_do_prearm_check(ctx)) return dk::Promise<bool>::resolve(ctx.engine, true);

    // 检查是否已经通过prearm check
    int bits = 0x10000000;
    if ((ctx.sensor_health & bits) == bits) {
        return dk::Promise<bool>::resolve(ctx.engine, true);
    }

    auto p = std::make_shared<dk::Promise<bool>>(ctx.engine);

    // 等待错误信息
    ctx.engine->wait_for(
        1000,
        [p](const StatusTextEvent& status_text) -> bool {
            if (is_prearm_msg(status_text.text)) {
                p->reject(status_text.text);
                return true;
            }
            return false;
        },
        [p](const SysStatusEvent& sys_status) -> bool {
            if (check_sensor_health(sys_status.data)) {
                p->resolve(true);
                return true;
            }
            return false;
        });

    ctx.engine->post_future_task([&ctx]() { ctx.robot->run_prearm_checks(); });
    return p->get_future();
}

bool check_alt(RobotContext& ctx, double target) {
    double TAKEOFF_RATE = 0.1;
    double TAKEOFF_DIFF_THRSH = 0.6;
    auto pos = ctx.pos_enu.get();
    return std::fabs(pos.z() - target) < std::fmax(target * TAKEOFF_RATE, TAKEOFF_DIFF_THRSH);
}

Eigen::Vector3d gps_to_enu(RobotContext& ctx, Eigen::Vector3d lon_lat_alt) {
    Eigen::Vector3d diff = get_relevant_enu(ctx.lon_lat_alt.get(), lon_lat_alt);
    auto pos = ctx.pos_enu.get();
    return diff + pos;
}

// 计算目标 GPS 点相对于无人机当前 GPS 点的 ENU 偏差向量
Eigen::Vector3d get_relevant_enu(const Eigen::Vector3d& drone_lon_lat_alt, const Eigen::Vector3d& target_lon_lat_alt) {
    // 1. 获取无人机当前的 UTM 投影参数
    int drone_zone;
    bool is_north;
    double drone_x, drone_y;

    // drone_gps[1] 是纬度， drone_gps[0] 是经度
    GeographicLib::UTMUPS::Forward(drone_lon_lat_alt[1], drone_lon_lat_alt[0], drone_zone, is_north, drone_x, drone_y);
    // 2. 获取目标点的 UTM 投影参数
    int target_zone;
    bool target_northp;
    double target_x, target_y;
    // 关键点：强制目标点使用无人机当前的投影带 (setzone = drone_zone)
    // 这样算出来的欧式距离才是准确的
    GeographicLib::UTMUPS::Forward(target_lon_lat_alt[1], target_lon_lat_alt[0], target_zone, target_northp, target_x,
                                   target_y, drone_zone);
    // 3. 返回相对偏差 (Target - Drone)
    return {
        target_x - drone_x,                           // 偏差 East (X)
        target_y - drone_y,                           // 偏差 North (Y)
        target_lon_lat_alt[2] - drone_lon_lat_alt[2]  // 偏差 Up (Z)
    };
}

dk::Future<bool> do_land(RobotContext& ctx) {
    return ctx.robot->land(ctx);
}

// enu坐标系的yaw转为ned
double yaw_enu_to_ned(double yaw_enu) {
    double heading = M_PI * 0.5 - yaw_enu;
    heading = norm_yaw(heading);
    return heading;
}

// 规范化到0~pi之间
double norm_yaw(double yaw) {
    while (yaw < 0) yaw += M_PI * 2;
    while (yaw >= 2 * M_PI) yaw -= M_PI * 2;
    return yaw;
}

// 给定两个坐标的差值，计算enu下的yaw
double get_heading(double enu_x, double enu_y) {
    double enu_yaw = atan2(enu_y, enu_x);
    return norm_yaw(enu_yaw);
}

// 计算两个角度之间的差值
double get_yaw_diff(double a, double b) {
    double diff = a - b;
    double theta = atan2(sin(diff), cos(diff));
    return fabs(theta);
}

Eigen::Quaterniond euler_to_orientation(double r, double p, double y) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());

    return q;
}

Eigen::Vector3d orientation_to_euler(double x, double y, double z, double w) {
    Eigen::Quaterniond q(w, x, y, z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    // Extract yaw using atan2 to keep range in [-pi, pi]
    double yaw = std::atan2(R(1, 0), R(0, 0));
    // Extract pitch using asin, clamp to [-1.0, 1.0] to prevent NaN
    double sin_pitch = -R(2, 0);
    sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
    double pitch = std::asin(sin_pitch);
    // Extract roll using atan2
    double roll = std::atan2(R(2, 1), R(2, 2));
    Eigen::Vector3d rpy = {roll, pitch, yaw};
    return rpy;
}

// 获取经过的时长
double get_time_span(std::chrono::steady_clock::time_point start) {
    auto diff = std::chrono::steady_clock::now() - start;
    return std::chrono::duration<double>(diff).count();
}

}  // namespace state_utils