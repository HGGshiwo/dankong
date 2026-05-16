#pragma once
#include <string.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <variant>
#include <vector>

#include "./utils.hpp"
#include "dk/engine.hpp"
#include "mavlink/imavlink.hpp"

using json = nlohmann::json;
namespace Eigen {
// JSON 转 Eigen::Vector3d
inline void from_json(const json& j, Vector3d& v) {
    // 假设 JSON 是一个包含 3 个数字的数组，例如 [1.0, 2.0, 3.0]
    v.x() = j.at(0).get<double>();
    v.y() = j.at(1).get<double>();
    v.z() = j.at(2).get<double>();
}
// Eigen::Vector3d 转 JSON
inline void to_json(json& j, const Vector3d& v) {
    j = json{v.x(), v.y(), v.z()};
}
}  // namespace Eigen

// @JSON_ENABLE
struct EventResult {
    std::string status = "OK";
    nlohmann::json msg;
};

// @JSON_ENABLE
enum FinishAction {
    HOVER,
    RETURN,
    LAND,
};

//@JSON_ENABLE
struct SetWaypointEvent : dk::AsyncEvent<EventResult> {
    std::vector<Eigen::Vector3d> waypoint;
    std::optional<std::vector<nlohmann::json>> nodeEventList;
    int land_target_id = -1;
    FinishAction finish_action = FinishAction::HOVER;
};

//@JSON_ENABLE
struct RebootFcuEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct GetWpEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct GetGpsEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct StopFollowEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct EnablePlandEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct DisablePlandEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct EnablePlannerEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct DisablePlannerEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct GetParamEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct SetParamEvent : dk::AsyncEvent<EventResult> {
    nlohmann::json param;
};

// @JSON_ENABLE
struct TakeoffEvent : dk::AsyncEvent<EventResult> {
    double alt = 10.0;
};

// @JSON_ENABLE
struct SetModeEvent : dk::AsyncEvent<EventResult> {
    std::string mode;
};

// @JSON_ENABLE
struct PrearmEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct SetPosVelEvent : dk::AsyncEvent<EventResult> {
    Eigen::Vector3d pos;
    double vel;
    double yaw;  // enu坐标系的yaw
    bool fix_yaw;
};

// @JSON_ENABLE
struct EnableDetectEvent : dk::AsyncEvent<EventResult> {
    std::string type;
};  // 开始检测

// @JSON_ENABLE
struct DisableDetectEvent : dk::AsyncEvent<EventResult> {};  // 结束检测

//@JSON_ENABLE
struct GetDetectEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct StartRecordEvent : dk::AsyncEvent<EventResult> {
    std::string bag_name;
};  // 开始录制

// @JSON_ENABLE
struct StopRecordEvent : dk::AsyncEvent<EventResult> {};  // 结束录制

// @JSON_ENABLE
struct GetGimbalEvent : dk::AsyncEvent<EventResult> {};  // 获取云台数据

// @JSON_ENABLE
struct SetGimbalEvent : dk::AsyncEvent<EventResult> {
    std::string mode;
    double angle;
};  // 设置云台

//@JSON_ENABLE
struct SetExposureEvent : dk::AsyncEvent<EventResult> {
    double shutter;
    double sensitivity;
};  // 设置曝光

//@JSON_ENABLE
struct GetExposureEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct DisarmEvent : dk::AsyncEvent<EventResult> {};

struct RestartEvent {};

struct FcuConnectedEvent {
    bool connected;
};

struct DetectEvent {
    double score;
    Eigen::Vector3d target_pos;
    Eigen::Vector3d cmd_vel;
};

struct DetectTargetEvent {
    Eigen::Vector3d pos;
};

struct ExcutePlandEvent {
    int target_tag_id;
};

struct FlightModeEvent {
    FixedString64 prev;
    FixedString64 cur;
};

struct SysStatusEvent {
    uint32_t data;
};

struct StatusTextEvent {
    std::string text;
};

struct ArmEvent {
    bool armed = false;
};

struct TakeoffDoneEvent {};

struct GlobalPositionEvent {
    double lat;
    double lon;
};

struct GlobalPositionAltEvent {
    double alt;
};
