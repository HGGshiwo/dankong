#pragma once
#include <Eigen/Dense>
#include <optional>

#include "core/event_result.hpp"
#include "core/flight_mode.hpp"
#include "dk/engine.hpp"
#include "nlohmann/json.hpp"
#include "utils/fixed_string64.hpp"

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
    std::optional<double> speed;
    bool do_pland = false;
    FinishAction finish_action = FinishAction::HOVER;
};

//@JSON_ENABLE
struct RebootFcuEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct GetWpEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct GetGpsEvent : dk::AsyncEvent<EventResult> {};

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
struct LoiterEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct TestEvent : dk::AsyncEvent<EventResult> {};

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
struct DisarmEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct JoystickEvent : dk::AsyncEvent<EventResult> {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

// @JSON_ENABLE
struct EnableJoystickEvent : dk::AsyncEvent<EventResult> {
    bool enable = false;
};

struct RestartEvent {};

struct FlightModeEvent {
    FlightMode prev;
    FlightMode cur;
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

struct WpArriveEvent {};

//@JSON_ENABLE
struct ReportEvent {
    std::string data;
};  // 用于ros->ws上报