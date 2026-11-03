#pragma once
#ifdef USE_ROS
#include <memory>

#include "core/base_tracker.hpp"
#include "utils/dirty_var.hpp"
#include "utils/fixed_string64.hpp"
#include "utils/state_registry.hpp"

struct AlgoContext {
    // =========================================================================
    // 纯净的数据载体 (Data Model)
    // =========================================================================

    DirtyVar<bool> planner_enable{false};
    DirtyVar<FixedString64> version{FixedString64("未知")};
    DirtyVar<FixedString64> detect_type{FixedString64("Disabled")};
    DirtyVar<bool> recording{false};
    // 检测期间航点巡航限速回调的 RAII 句柄，关闭检测时析构自动注销
    std::shared_ptr<SpeedLimitToken> speed_limit_token;

   public:
    // =========================================================================
    // 外部上报绑定 (Telemetry Binding)
    // =========================================================================
    explicit AlgoContext(StateRegistry& reg) {
        // 在构造时进行统一绑定，结构体不再长期持有 reg 的引用
        reg.bind("planner", planner_enable, 2.0);
        reg.bind("version", version, 0.5);
        reg.bind("detect_type", detect_type, 1.0);
        reg.bind("recording", recording, 1.0);
    }
};
#endif