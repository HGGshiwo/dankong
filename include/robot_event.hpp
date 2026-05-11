#pragma once
#include <string.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <variant>

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
    std::string msg;
};

//@JSON_ENABLE
struct SetWaypointEvent : dk::AsyncEvent<EventResult> {
    std::vector<Eigen::Vector3d> waypoint;
};

// @JSON_ENABLE
struct TakeoffEvent : dk::AsyncEvent<EventResult> {
    double alt = 10.0;
};

// @JSON_ENABLE
struct PrearmEvent : dk::AsyncEvent<EventResult> {};

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
