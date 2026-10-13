#pragma once
#include <optional>
// 注: 原有 #ifdef USE_ROS 守卫已注释。本文件仅为纯数据结构(事件)定义, 其依赖
// (Eigen/dk 框架)在非 ROS 构建下同样可用; 若保留守卫, dk/scripts/gen_json.py
// 按文本扫描 @JSON_ENABLE 标记时会在非 ROS 构建中生成引用不存在类型的序列化代码,
// 导致编译失败。功能实现层(pland/feature.hpp 等)仍保留 USE_ROS 守卫。
// #ifdef USE_ROS
#include "Eigen/Dense"
#include "core/event_result.hpp"
#include "dk/engine.hpp"

// @JSON_ENABLE
struct StartPlandDetectEvent : dk::AsyncEvent<EventResult> {};

// @JSON_ENABLE
struct SetPlandTarget : dk::AsyncEvent<EventResult> {
    std::optional<Eigen::Vector3d> position = std::nullopt;
    std::optional<Eigen::Vector3d> velocity = std::nullopt;
};

// @JSON_ENABLE
struct StartOffsetEstimate : dk::AsyncEvent<EventResult> {
    double x = 0;
    double y = 0;
    double z = 0;
};

// @JSON_ENABLE
struct StopOffsetEstimate : dk::AsyncEvent<EventResult> {
    bool save = false;
};
// #endif