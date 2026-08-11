#include "states/state_utils.hpp"

#include "core/engine.hpp"
#include "robot_context.hpp"
#include "states/follow_state.hpp"
#include "states/hover_state.hpp"
#include "states/land_state.hpp"
#include "states/posvel_state.hpp"
#include "states/takeoff_state.hpp"
#include "states/waypoint_state.hpp"

namespace state_utils {
bool is_prearm_msg(const std::string& text) {
    return text.rfind("PreArm: ", 0) == 0 || text.rfind("Arm: ", 0) == 0;
}

// 判断是否真的需要进行prearm_check
bool should_do_prearm_check(std::shared_ptr<IRobot> robot) {
    if (!robot->is_prearm_enable()) {
        return false;
    }
    int value = robot->get_param<int>("ARMING_CHECK", 0);
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

Eigen::Vector3d gps_to_enu(Eigen::Vector3d cur_lon_lat_alt,
                           Eigen::Vector3d cur_pos_enu,
                           Eigen::Vector3d lon_lat_alt) {
    Eigen::Vector3d diff = get_relevant_enu(cur_lon_lat_alt, lon_lat_alt);
    auto pos = cur_pos_enu;
    return diff + pos;
}

// Helper: Convert relative BODY pose to absolute ENU pose
// Vector4d: body_x, body_y, body_z, body_yaw -> enu_x, enu_y, enu_z, enu_yaw
Eigen::Vector4d body_to_enu(const Eigen::Vector4d& body_target,
                            const Eigen::Vector4d& current_odom) {
    Eigen::Vector4d enu_target;
    // 2D Rotation matrix transformation
    double cos_yaw = std::cos(current_odom.w());
    double sin_yaw = std::sin(current_odom.w());

    enu_target.x() = current_odom.x() +
                     (body_target.x() * cos_yaw - body_target.y() * sin_yaw);
    enu_target.y() = current_odom.y() +
                     (body_target.x() * sin_yaw + body_target.y() * cos_yaw);
    enu_target.z() = current_odom.z() + body_target.z();
    enu_target.w() = state_utils::norm_yaw(current_odom.w() + body_target.w());

    return enu_target;
}

// 计算目标 GPS 点相对于无人机当前 GPS 点的 ENU 偏差向量
Eigen::Vector3d get_relevant_enu(const Eigen::Vector3d& drone_lon_lat_alt,
                                 const Eigen::Vector3d& target_lon_lat_alt) {
    // 1. 获取无人机当前的 UTM 投影参数
    int drone_zone;
    bool is_north;
    double drone_x, drone_y;

    // drone_gps[1] 是纬度， drone_gps[0] 是经度
    GeographicLib::UTMUPS::Forward(drone_lon_lat_alt[1], drone_lon_lat_alt[0],
                                   drone_zone, is_north, drone_x, drone_y);
    // 2. 获取目标点的 UTM 投影参数
    int target_zone;
    bool target_northp;
    double target_x, target_y;
    // 关键点：强制目标点使用无人机当前的投影带 (setzone = drone_zone)
    // 这样算出来的欧式距离才是准确的
    GeographicLib::UTMUPS::Forward(target_lon_lat_alt[1], target_lon_lat_alt[0],
                                   target_zone, target_northp, target_x,
                                   target_y, drone_zone);
    // 3. 返回相对偏差 (Target - Drone)
    return {
        target_x - drone_x,                           // 偏差 East (X)
        target_y - drone_y,                           // 偏差 North (Y)
        target_lon_lat_alt[2] - drone_lon_lat_alt[2]  // 偏差 Up (Z)
    };
}

// Calculate the target GPS point based on the drone's current GPS and the ENU
// offset vector
Eigen::Vector3d enu_to_gps(Eigen::Vector3d cur_lon_lat_alt,
                           Eigen::Vector3d cur_pos_enu,
                           const Eigen::Vector3d& enu) {
    // 1. Get the UTM projection parameters and coordinates of the drone
    int drone_zone;
    bool is_north;
    double drone_x, drone_y;
    auto drone_lon_lat_alt = cur_lon_lat_alt;
    // drone_lon_lat_alt[1] is Latitude, drone_lon_lat_alt[0] is Longitude
    GeographicLib::UTMUPS::Forward(drone_lon_lat_alt[1], drone_lon_lat_alt[0],
                                   drone_zone, is_north, drone_x, drone_y);
    // 2. Add the ENU offset to the drone's UTM coordinates to get the target's
    // UTM coordinates enu_vector[0] is East offset, enu_vector[1] is North
    // offset
    auto drone_enu = cur_pos_enu;
    double target_x = drone_x + enu[0] - drone_enu.x();
    double target_y = drone_y + enu[1] - drone_enu.y();
    // 3. Reverse project the target's UTM coordinates back to Latitude and
    // Longitude Must use the drone's UTM zone and hemisphere (is_north)
    double target_lat, target_lon;
    GeographicLib::UTMUPS::Reverse(drone_zone, is_north, target_x, target_y,
                                   target_lat, target_lon);
    // 4. Return the target's {Longitude, Latitude, Altitude}
    return {
        target_lon,  // Target Longitude
        target_lat,  // Target Latitude
        drone_lon_lat_alt[2] + enu[2] -
            drone_enu.z()  // Target Altitude (Drone Alt + Up offset)
    };
}

// enu坐标系的yaw转为ned
double yaw_enu_to_ned(double yaw_enu) {
    double heading = M_PI * 0.5 - yaw_enu;
    heading = norm_yaw(heading);
    return heading;
}

// ned坐标系的yaw转为enu
double yaw_ned_to_enu(double yaw_ned) {
    double heading = M_PI * 0.5 - yaw_ned;
    heading = norm_yaw(heading);
    return heading;
}

// 规范化到0~2pi之间
double norm_yaw(double yaw) {
    while (yaw < 0) yaw += M_PI * 2;
    while (yaw >= 2 * M_PI) yaw -= M_PI * 2;
    return yaw;
}

// 给定两个坐标的差值，计算enu下的yaw
// 注意要判断两点是否足够的近
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

Eigen::Vector3d orientation_to_euler(Eigen::Quaterniond q) {
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

Eigen::Vector3d orientation_to_euler(double x, double y, double z, double w) {
    Eigen::Quaterniond q(w, x, y, z);
    return orientation_to_euler(q);
}

// 获取经过的时长
double get_time_span(double start_time, double current_now) {
    return current_now - start_time;
}

}  // namespace state_utils