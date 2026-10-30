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
    bool local = false;
    bool skip_lifting = true;
    std::vector<int> dwell_indices;
};

//@JSON_ENABLE
struct LandEvent : dk::AsyncEvent<EventResult> {
    bool do_pland = false;
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
    // 目标位置 GPS, 支持 [lon,lat,alt] 或 [lon,lat]; 第三值 (alt) 忽略,
    // 目标高度始终为飞机当前高度
    nlohmann::json pos;
    double vel = 2;  // XY 巡航速度限制 (m/s), 不传默认 2 (同 mavproxy)
    std::optional<double>
        yaw;  // 到达后的期望偏航 (NED 系弧度); 不传=不指定朝向
    std::optional<bool> fix_yaw =
        true;  // true: 运动全程锁定目标yaw; false: 机头朝运动方向
    std::optional<double>
        timeout;  // 指令失效时间 (s); 不传用配置 posvel_timeout
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
struct OldJoystickEvent : dk::AsyncEvent<EventResult> {
    double right_x = 0;
    double right_y = 0;
    double left_x = 0;
    double left_y = 0;
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