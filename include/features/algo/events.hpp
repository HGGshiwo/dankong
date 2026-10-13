#pragma once
// 注: 原有 #ifdef USE_ROS 守卫已注释。本文件仅为纯数据结构(事件)定义, 其依赖
// (Eigen/nlohmann/dk 框架)在非 ROS 构建下同样可用; 若保留守卫, dk/scripts/gen_json.py
// 按文本扫描 @JSON_ENABLE 标记时会在非 ROS 构建中生成引用不存在类型的序列化代码,
// 导致编译失败。功能实现层(feature.hpp/event_listener.hpp)仍保留 USE_ROS 守卫。
// #ifdef USE_ROS
#include <Eigen/Dense>
#include <nlohmann/json.hpp>  // AbnormalEvent 依赖, 原先靠传递包含, 显式补齐

#include "core/event_result.hpp"
#include "dk/engine.hpp"

struct AbnormalEvent {
    nlohmann::json data;
};

//@JSON_ENABLE
struct StopFollowEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct EnablePlannerEvent : dk::AsyncEvent<EventResult> {};

//@JSON_ENABLE
struct DisablePlannerEvent : dk::AsyncEvent<EventResult> {};

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
// #endif