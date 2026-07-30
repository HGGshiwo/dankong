#pragma once
#include <boost/filesystem/path.hpp>
#include <memory>
#include <set>

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

    static void add_files(boost::filesystem::path dir, std::string ext,
                          std::set<std::string>& file_set) {
        if (boost::filesystem::is_directory(dir)) {
            for (auto& entry : boost::make_iterator_range(
                     boost::filesystem::directory_iterator(dir), {})) {
                if (boost::filesystem::is_regular_file(entry.status())) {
                    if (entry.path().extension() == ext)
                        file_set.insert(entry.path().string());
                }
            }
        }
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

        web->register_safe_file_stream_route(
            boost::beast::http::verb::get, "/spdlog", []() {
                auto& cfg = GlobalConfig.GetConfig();

                std::set<std::string> log_files;
                auto spdlog_dir =
                    get_config_dir(cfg.log_dir.get()).parent_path();
                add_files(spdlog_dir, ".txt", log_files);
                return log_files;
            });

        web->register_safe_file_stream_route(
            boost::beast::http::verb::get, "/fglog", []() {
                auto& cfg = GlobalConfig.GetConfig();

                std::set<std::string> log_files;
                auto spdlog_dir =
                    get_config_dir(cfg.fg_log_dir.get()).parent_path();
                add_files(spdlog_dir, ".jsonl", log_files);
                return log_files;
            });

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
                            fglog::enable_ws_connection(conn->get_id());
                            spdlog::info(
                                "[fglog] Enabled for client connection ID: {}",
                                conn->get_id());
                        } else {
                            fglog::disable_ws_connection(conn->get_id());
                            spdlog::info(
                                "[fglog] Disabled for client connection ID: {}",
                                conn->get_id());
                        }
                    }
                } catch (...) {
                }
            });
    }
};
