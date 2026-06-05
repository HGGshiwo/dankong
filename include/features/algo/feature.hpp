#pragma once
#include <memory>

#include "./events.hpp"
#include "core/engine.hpp"
#include "core/tag.hpp"
#include "dk/adapters/ros.hpp"
#include "std_msgs/String.h"

class AlgoFeature {
   public:
    template <typename WebAdapter>
    static void setup(TagWeb, std::shared_ptr<WebAdapter>& web) {
        web->template register_route<StopFollowEvent, EventResult>(
            boost::beast::http::verb::post, "/stop_follow");

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

    static void setup(TagListeners, const std::shared_ptr<Engine>& engine);

    static void setup(
        TagRos, std::shared_ptr<dk::RosAdapter<RobotContext, Engine>>& ros) {
        ros->bind_event("/mavproxy/ws",
                        [](std_msgs::String::ConstPtr data) -> ReportEvent {
                            ReportEvent e;
                            e.data = data->data;
                            return e;
                        });
    }
};

#include "./event_listener.hpp"

inline void AlgoFeature::setup(TagListeners,
                               const std::shared_ptr<Engine>& engine) {
    auto listener = std::make_shared<AlgoEventListener>();
    engine->add_listener(listener);
}