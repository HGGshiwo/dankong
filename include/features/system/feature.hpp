#pragma once
#include <memory>

#include "./event_listener.hpp"
#include "./events.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "utils/get_executable_path.hpp"

class SystemFeature {
   public:
    static void setup(TagListeners, const std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<SystemEventListener>();
        engine->add_listener(listener);
    }

    static void setup(
        TagWeb, std::shared_ptr<dk::WebAdapter<RobotContext, Engine>>& web) {
        auto& cfg = GlobalConfig.GetConfig();

        web->enable_cors();

        // 注册基础路由
        web->register_file_route(boost::beast::http::verb::get, "/page_config",
                                 get_config_dir(cfg.json_path.get()));

        // 注册静态目录
        web->register_static_dir("/", get_config_dir(cfg.static_dir.get()));
        web->register_static_dir("/home", get_config_dir(cfg.static_dir.get()));

        web->template register_route<GetConfigEvent, EventResult>(
            boost::beast::http::verb::get, "/config/get");

        web->template register_route<SetConfigEvent, EventResult>(
            boost::beast::http::verb::post, "/config/set");

        web->template register_route<LogEvent, EventResult>(
            boost::beast::http::verb::post, "/log");

        // 注册默认 Websocket
        web->register_managed_ws_route(
            "/ws",
            [](std::shared_ptr<dk::WsConnection> conn, const std::string& msg) {
                try {
                    auto j = nlohmann::json::parse(msg);
                    if (j.contains("fglog_enable")) {
                        bool enable = j["fglog_enable"].get<bool>();
                        if (enable) {
                            fglog::fglog_conn_registry::get().enable(
                                conn->get_id());
                            spdlog::info(
                                "[fglog] Enabled for client connection ID: {}",
                                conn->get_id());
                        } else {
                            fglog::fglog_conn_registry::get().disable(
                                conn->get_id());
                            spdlog::info(
                                "[fglog] Disabled for client connection ID: {}",
                                conn->get_id());
                        }
                    }
                } catch (...) {
                    if (msg == "fglog_enable: true" ||
                        msg == "fglog_enable: 1") {
                        fglog::fglog_conn_registry::get().enable(
                            conn->get_id());
                        spdlog::info(
                            "[fglog] Enabled for client connection ID: {}",
                            conn->get_id());
                    } else if (msg == "fglog_enable: false" ||
                               msg == "fglog_enable: 0") {
                        fglog::fglog_conn_registry::get().disable(
                            conn->get_id());
                        spdlog::info(
                            "[fglog] Disabled for client connection ID: {}",
                            conn->get_id());
                    }
                }
            });
    }
};
