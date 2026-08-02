#pragma once
#include "core/base_assembler.hpp"

// 引入你需要的 Feature
#include "features/control/config.hpp"
#include "features/dog/config.hpp"
#include "features/mavlink/config.hpp"
#include "features/report/config.hpp"
#include "features/system/config.hpp"
#include "features/tracker/config.hpp"

// =================================================================
// 终极魔法：只需这一段配置，剩下的全交由编译器生成！
// =================================================================
using RobotConfig = ConfigGenerator<SystemConfig, ControlConfig, MavlinkConfig,
                                    DogConfig, TrackerConfig, ReportConfig>;
