#include "./events.hpp"
#include "dk/event_listener.hpp"
#include "robot_config.hpp"
#include "robot_context.hpp"
#include "utils/logger/fglog.hpp"

class SystemEventListener
    : public dk::BaseEventListener<RobotContext, SystemEventListener> {
   public:
    using AllowedEvents = std::tuple<GetConfigEvent, SetConfigEvent, LogEvent>;

    void on_event(const GetConfigEvent& event, RobotContext& ctx) {
        nlohmann::json j;
        j["config"]["_value"] = GlobalConfig.GetAllParams();
        event.resolve({"success", j});
    }

    void on_event(const LogEvent& event, RobotContext& ctx) {
        spdlog::info("[Log] {}", event.data);
        fglog::log("/drone/log", event.data);
        event.resolve({"success", "OK"});
    }

    void on_event(const SetConfigEvent& event, RobotContext& ctx) {
        for (auto& [k, v] : event.config.items()) {
            auto j = v["value"];
            GlobalConfig.UpdateParam(k, v["value"]);
        }
        event.resolve({"success", "OK"});
        GlobalConfig.save();
    }
};