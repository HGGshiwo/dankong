#pragma once
#include "core/base_assembler.hpp"

// 引入你需要的 Feature
#include "features/control/config.hpp"
#include "features/drone/config.hpp"
#include "features/system/config.hpp"
#include "features/tracker/config.hpp"

#ifdef USE_ROS
#include "features/algo/config.hpp"
#include "features/pland/config.hpp"
#endif

// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
using RobotConfig = ConfigGenerator<
#ifdef USE_ROS
    AlgoConfig, PlandConfig,
#endif
    SystemConfig, ControlConfig, DroneConfig, TrackerConfig>;
