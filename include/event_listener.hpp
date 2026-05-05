#pragma once
#include <exception>

#include "./robot_context.hpp"
#include "./robot_event.hpp"
#include "./states/state_utils.hpp"
#include "dk/core.hpp"

class EventListener : public dk::BaseEventListener<RobotEvent, RobotContext, EventListener> {
   public:
    void on_event(PrearmEvent event, RobotContext& ctx) {
        state_utils::prearm_check(ctx)
            .then([event](bool res) {
                if (res) event.resolve({"success", "OK"});
                event.resolve({"error", "Unknown Error"});
            })
            .catch_error([event](std::exception_ptr exp) {
                try {
                    std::rethrow_exception(exp);
                } catch (const std::exception& e) {
                    event.resolve({"error", e.what()});
                }
            });
    }
};