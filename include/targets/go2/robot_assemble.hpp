#pragma once
#include "core/base_assembler.hpp"
#include "features/control/feature.hpp"
#include "features/go2/feature.hpp"
#include "features/mavlink/feature.hpp"
#include "features/report/feature.hpp"
#include "features/system/feature.hpp"
#include "features/tracker/feature.hpp"
// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
// 注意顺序，否则Tracker拿不到robot
using Go2Assembler =
    BaseAssembler<SystemFeature, ControlFeature, MavlinkFeature, ReportFeature,
                  Go2Feature, TrackerFeature>;

// 全局唯一的上下文类型生成
using RobotAssembler = Go2Assembler;