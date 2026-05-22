#pragma once
#include <memory>

#include "./context.hpp"
#include "./events.hpp"
#include "core/engine.hpp"

class AlgoFeature {
   public:
    // 1. 声明属于自己的 Context（如果没有，就写 EmptyContext）
    using Context = AlgoContext;

    template <typename WebAdapter>
    static void register_web(std::shared_ptr<WebAdapter>& web) {
        web->template register_route<StopFollowEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_follow");

        web->template register_route<EnablePlandEvent, EventResult>(
            boost::beast::http::verb::post, "/start_pland");

        web->template register_route<DisablePlandEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_pland");

        web->template register_route<EnablePlannerEvent, EventResult>(
            boost::beast::http::verb::post, "/start_planner");

        web->template register_route<DisablePlannerEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_planner");

        web->template register_route<StartRecordEvent, EventResult>(
            boost::beast::http::verb::post, "/start_record");

        web->template register_route<StopRecordEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_record");

        web->template register_route<GetGimbalEvent, EventResult>(
            boost::beast::http::verb::get, "/get_gimbal");

        web->template register_route<SetGimbalEvent, EventResult>(
            boost::beast::http::verb::post, "/set_gimbal");

        web->template register_route<GetExposureEvent, EventResult>(
            boost::beast::http::verb::get, "/get_exposure");

        web->template register_route<SetExposureEvent, EventResult>(
            boost::beast::http::verb::post, "/set_exposure");

        // 该接口只是调试时候使用
        web->template register_route<SetWaypointEvent, EventResult>(
            boost::beast::http::verb::post, "/pland", 5000,
            [](SetWaypointEvent& event) {
                event.land_target_id = 0;
                event.finish_action = FinishAction::LAND;
            });

        web->template register_route<EnableDetectEvent, EventResult>(
            boost::beast::http::verb::post, "/start_detect");

        web->template register_route<DisableDetectEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_detect");

        web->template register_route<GetDetectEvent, EventResult>(
            boost::beast::http::verb::get, "/get_detect");
    }

    static void register_listeners(std::shared_ptr<Engine>& engine);
};

#include "./event_listener.hpp"

inline void AlgoFeature::register_listeners(std::shared_ptr<Engine>& engine) {
    auto listener = std::make_shared<AlgoEventListener>();
    engine->add_listener(listener);
}