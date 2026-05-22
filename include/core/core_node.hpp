#include <boost/filesystem/operations.hpp>
#include <cstdint>
#include <memory>

#include "./engine.hpp"
#include "dk/adapters/ros.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "dk/adapters/web.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/dog/command.hpp"
#include "ros/node_handle.h"
#include "states/init_state.hpp"
#include "utils/get_executable_path.hpp"

const std::string STAIC_DIR =
    (get_executable_path().parent_path() / "dk/frontend/dist").string();
const std::string UI_CONFIG_PATH =
    (get_executable_path().parent_path() / "config/ui.yaml").string();
const std::string JSON_PATH =
    (get_executable_path().parent_path() / "config.json").string();

const int PORT = 8000;
const int UDP_SERVER_PORT = 9111;

template <typename AssemblerType, typename ContextType>
class CoreNode {
    using RosAdapterType = dk::RosAdapter<ContextType, Engine>;
    using WebAdapterType = dk::WebAdapter<ContextType, Engine>;
    using UdpAdpterType = dk::UdpAdapter<ContextType, Engine, CommandType>;

   private:
    std::shared_ptr<Engine> engine_;
    std::shared_ptr<RosAdapterType> ros_adapter_;
    std::shared_ptr<WebAdapterType> web_adapter_;
    std::shared_ptr<UdpAdpterType> udp_adapter_;
    ros::NodeHandle nh_;

   public:
    CoreNode() : nh_() {
        // 1. 初始化基础引擎

        engine_ = std::make_shared<Engine>();
        engine_->get_context().engine = engine_;

        const std::string json_path =
            (fs::current_path() / "config.json").string();
        dk::generate_json_file(UI_CONFIG_PATH, JSON_PATH);

        web_adapter_ = std::make_shared<WebAdapterType>(engine_, PORT);

        web_adapter_->enable_cors();

        engine_->get_context().ws_manager = web_adapter_->get_manager();

        web_adapter_->register_file_route(boost::beast::http::verb::get,
                                          "/page_config", JSON_PATH);
        web_adapter_->register_static_dir("/", STAIC_DIR);
        web_adapter_->register_static_dir("/home", STAIC_DIR);
        web_adapter_->register_managed_ws_route("/ws", [](auto, auto&) {});

        // 2. 调用装配工厂，一键注册全部路由和回调！
        AssemblerType::setup_web(web_adapter_);

        if constexpr (AssemblerType::template has_ros<RosAdapterType>) {
            ros_adapter_ = std::make_shared<RosAdapterType>(engine_, nh_);
            AssemblerType::setup_ros(ros_adapter_);
        }

        if constexpr (AssemblerType::template has_udp<UdpAdpterType>) {
            udp_adapter_ = std::make_shared<UdpAdpterType>(
                engine_, UDP_SERVER_PORT,
                [](const std::vector<uint8_t>& data) -> CommandType {
                    return UdpPacketView(data).command();
                });
            AssemblerType::setup_udp(udp_adapter_);
        }

        if constexpr (AssemblerType::template has_init<RobotContext>) {
            AssemblerType::setup_init(engine_->get_context());
        }

        if constexpr (AssemblerType::template has_listeners<Engine>) {
            AssemblerType::setup_listeners(engine_);
        }

        // 3. 启动引擎
        engine_->start<InitState>(std::chrono::milliseconds(50));
    }
};