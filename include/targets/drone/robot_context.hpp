#pragma once
#include "core/base_assembler.hpp"

// 引入你需要的 Feature
#include "features/control/context.hpp"
#include "features/drone/context.hpp"
#include "features/mavlink/context.hpp"
#include "features/ntrip/context.hpp"
#include "features/tracker/context.hpp"
#ifdef USE_ROS
#include "features/algo/context.hpp"
#include "features/pland/context.hpp"
#endif

// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
using RobotContext = ContextGenerator<
#ifdef USE_ROS
    AlgoContext, PlandContext,
#endif
    ControlContext, DroneContext, TrackerContext, NtripContext, MavlinkContext>;
