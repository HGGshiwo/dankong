#pragma once
#include "core/base_assembler.hpp"

// 引入你需要的 Feature
#include "features/algo/context.hpp"
#include "features/control/context.hpp"
#include "features/dog/context.hpp"
#include "features/tracker/context.hpp"
// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
using RobotContext =
    ContextGenerator<AlgoContext, ControlContext, DogContext, TrackerContext>;
