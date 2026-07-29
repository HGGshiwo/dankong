#pragma once

#include "dk/event_listener.hpp"
#include "robot_context.hpp"
#include "states/hover_state.hpp"
#include "states/state_utils.hpp"
#include "utils/state_registry.hpp"

class Go2EventListener
    : public dk::BaseEventListener<RobotContext, Go2EventListener> {
   public:
    using AllowedEvents = std::tuple<>;

    void on_tick(double dt, RobotContext& ctx) override {
        if (ctx.gps_fix_type.load() < 3) {
            // 根据之前的映射更新gps
            auto datum = ctx.datum.getReliableDatum();
            if (!datum.has_value()) return;  // 还没有定位
            auto fake_gps = state_utils::enu_to_gps(datum->gps, datum->enu,
                                                    ctx.pos_enu.load());
            ctx.lon_lat_alt.store(fake_gps);  // 更新外推得到的GPS
        }
    }
};
