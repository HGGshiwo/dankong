#pragma once
#include <memory>

#include "./event_listener.hpp"
#include "./events.hpp"
#include "core/engine.hpp"
#include "core/global_config.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "utils/get_executable_path.hpp"

class SystemFeature {
   public:
    static void register_listeners(std::shared_ptr<Engine>& engine) {
        auto listener = std::make_shared<SystemEventListener>();
        engine->add_listener(listener);
    }

    template <typename WebAdapter>
    static void register_web(std::shared_ptr<WebAdapter>& web) {
        auto& cfg = GlobalConfig.GetConfig();

        web->enable_cors();

        // 注册基础路由
        web->register_file_route(boost::beast::http::verb::get, "/page_config",
                                 get_executable_dir() / cfg.json_path.get());

        // 注册静态目录
        web->register_static_dir("/",
                                 get_executable_dir() / cfg.static_dir.get());
        web->register_static_dir("/home",
                                 get_executable_dir() / cfg.static_dir.get());

        web->template register_route<GetConfigEvent, EventResult>(
            boost::beast::http::verb::get, "/config/get");

        web->template register_route<SetConfigEvent, EventResult>(
            boost::beast::http::verb::post, "/config/set");

        // 注册默认 Websocket
        web->register_managed_ws_route("/ws", [](auto, auto&) {});
    }
};
