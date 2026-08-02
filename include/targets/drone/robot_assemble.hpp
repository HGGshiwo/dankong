#pragma once
#include "core/base_assembler.hpp"

// 引入你需要的 Feature
#include "features/algo/feature.hpp"
#include "features/control/feature.hpp"
#include "features/drone/feature.hpp"
#include "features/mavlink/feature.hpp"
#include "features/ntrip/feature.hpp"
#include "features/pland/feature.hpp"
#include "features/report/feature.hpp"
#include "features/system/feature.hpp"
#include "features/tracker/feature.hpp"

// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
using DroneAssembler = BaseAssembler<
#ifdef USE_ROS
    AlgoFeature, PlandFeature,
#endif
    ControlFeature, MavlinkFeature, ReportFeature, DroneFeature, TrackerFeature,
    SystemFeature, NtripFeature>;

// 全局唯一的上下文类型生成
using RobotAssembler = DroneAssembler;