#include <mavsdk/connection_result.h>
#include <mavsdk/mavsdk.h>

#include <boost/filesystem/operations.hpp>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

#include "./engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/AsioTimeProvider.hpp"
#include "dk/ITimeProvider.hpp"
#include "dk/adapters/can/can_adapter.hpp"
#include "dk/adapters/mavsdk.hpp"
#include "dk/adapters/mqtt/adapter.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "dk/adapters/web.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/dog/command.hpp"
#include "features/mavlink/events.hpp"
#include "robot_context.hpp"
#include "states/init_state.hpp"
#include "utils/get_executable_path.hpp"
#include "utils/logger/fglog.hpp"
#include "utils/logger/spd_logger.hpp"
#include "utils/yaml_helper.hpp"

#ifdef USE_ROS
#include "dk/RosTimeProvider.hpp"
#include "dk/adapters/ros.hpp"
#ifdef USE_ROS1
#include "ros/node_handle.h"
#endif
#endif

// ==========================================
// 外部模板声明，指示编译器在其他编译单元进行显式实例化，加速编译并降低内存压力。
// ==========================================
extern template class dk::WebAdapter<RobotContext, Engine>;
extern template class dk::MavsdkAdapter<RobotContext, Engine>;
extern template class dk::UdpAdapter<RobotContext, Engine, CommandType>;
extern template class dk::CanAdapter<RobotContext, Engine>;
extern template class dk::MqttClientAdapter<RobotContext, Engine>;
#ifdef USE_ROS
extern template class dk::RosAdapter<RobotContext, Engine>;
#endif

struct CLIArgs {
    std::string config_path;
    std::optional<unsigned int> port = std::nullopt;
    std::optional<unsigned int> camera_port = std::nullopt;
    std::optional<std::string> mavsdk_url = std::nullopt;
};

template <typename AssemblerType, typename ContextType>
class CoreNode {
#ifdef USE_ROS
    using RosAdapterType = dk::RosAdapter<ContextType, Engine>;
#endif
    using MavsdkAdapterType = dk::MavsdkAdapter<ContextType, Engine>;
    using WebAdapterType = dk::WebAdapter<ContextType, Engine>;
    using UdpAdpterType = dk::UdpAdapter<ContextType, Engine, CommandType>;
    using CanAdapterType = dk::CanAdapter<ContextType, Engine>;
    using MqttAdapterType = dk::MqttClientAdapter<ContextType, Engine>;

   private:
    boost::asio::io_context global_io_;
    std::shared_ptr<dk::ITimeProvider> time_provider_;
    std::shared_ptr<Engine> engine_;

#ifdef USE_ROS
#ifdef USE_ROS1
    ros::NodeHandle nh_;
#elif defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> nh_;
#endif
    std::shared_ptr<RosAdapterType> ros_adapter_;
    std::thread asio_thread_;
#endif
    std::shared_ptr<WebAdapterType> web_adapter_;
    std::shared_ptr<UdpAdpterType> udp_adapter_;
    std::shared_ptr<CanAdapterType> can_adapter_;
    std::shared_ptr<MqttAdapterType> mqtt_adapter_;

    std::shared_ptr<MavsdkAdapterType> mavsdk_adapter_;
    std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> mavsdk_system_;

   public:
    CoreNode(CLIArgs args) {
#ifdef USE_ROS
#ifdef USE_ROS2
        nh_ = std::make_shared<rclcpp::Node>("dk_node");
#endif
        time_provider_ = std::make_shared<dk::RosTimeProvider>(nh_, global_io_);
#else
        time_provider_ = std::make_shared<dk::AsioTimeProvider>(global_io_);
#endif

        GlobalConfig.load(get_config_dir(args.config_path));

        auto& cfg = GlobalConfig.GetConfig();
        init_spd_logger();

        // Initialize MAVSDK
        mavsdk::Mavsdk::Configuration config(
            mavsdk::ComponentType::GroundStation);
        mavsdk_ = std::make_shared<mavsdk::Mavsdk>(config);

        std::string mavsdk_url = args.mavsdk_url.value_or(cfg.mavsdk_url.get());
        mavsdk::ConnectionResult connection_result =
            mavsdk_->add_any_connection(mavsdk_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            throw std::runtime_error(
                "Connection failed: " +
                std::to_string(static_cast<int>(connection_result)));
        } else {
            spdlog::info("Waiting to discover system...");
            mavsdk_system_ = mavsdk_
                                 ->first_autopilot(
                                     GlobalConfig.GetConfig().fcu_timeout.get())
                                 .value_or(nullptr);
            if (mavsdk_system_ == nullptr) {
                throw std::runtime_error("connect to fcu failed: " +
                                         mavsdk_url);
            }
        }

        // 1. 初始化基础引擎
        engine_ = std::make_shared<Engine>(global_io_, time_provider_);
        engine_->get_context().engine = engine_;
        engine_->get_context().mavsdk_system = mavsdk_system_;
#ifdef USE_ROS
#ifdef USE_ROS1
        engine_->get_context().nh = nh_;
#elif defined(USE_ROS2)
        engine_->get_context().node = nh_;
#endif
#endif

        boost::filesystem::path config_dir =
            get_config_dir(args.config_path).parent_path();
        // 生成配置文件 (供前端使用)
        auto yaml_cfg =
            YamlHelper::load_with_base(config_dir / cfg.ui_config.get());
        auto json_cfg = YamlHelper::yaml_to_json(yaml_cfg);

        YamlHelper::save(json_cfg, get_config_dir(cfg.json_path.get()));

        // 创建适配器
        unsigned int server_port = args.port.value_or(cfg.server_port);
        web_adapter_ = std::make_shared<WebAdapterType>(engine_, server_port);
        auto ws_mgr = web_adapter_->get_manager();
        engine_->get_context().ws_manager = ws_mgr;

        std::string mqtt_host = cfg.mqtt_host;
        unsigned int mqtt_port = cfg.mqtt_port;

        mqtt_adapter_ =
            std::make_shared<dk::MqttClientAdapter<RobotContext, Engine>>(
                engine_, mqtt_host, mqtt_port);
        engine_->get_context().mqtt_client = mqtt_adapter_;

        AssemblerType::template setup<TagMqtt>(engine_->get_context(),
                                               mqtt_adapter_);
        mqtt_adapter_->connect();

        dk::ConnectionManager::on_conn_removed = [](size_t id) {
            fglog::disable_ws_connection(id);
        };

        fglog::set_websocket_sender([ws_mgr](const nlohmann::json& msg) {
            if (!ws_mgr) return;
            try {
                ws_mgr->publish(msg, [](size_t id) {
                    return fglog::check_ws_connection(id);
                });
            } catch (...) {
            }
        });

        // 2. 调用装配工厂，一键注册全部路由和回调！
        AssemblerType::template setup<TagWeb>(web_adapter_);

#ifdef USE_ROS
        if constexpr (AssemblerType::template has_feature_for<TagRos>) {
            ros_adapter_ = std::make_shared<RosAdapterType>(engine_, nh_);
            AssemblerType::template setup<TagRos>(ros_adapter_);
        }
#endif

        if constexpr (AssemblerType::template has_feature_for<TagMavsdk>) {
            if (mavsdk_system_) {
                mavsdk_adapter_ = std::make_shared<MavsdkAdapterType>(
                    engine_, mavsdk_system_);
                AssemblerType::template setup<TagMavsdk>(mavsdk_adapter_);
            } else {
                spdlog::warn("MAVSDK System is null, skipped TagMavsdk setup.");
            }
        }

        if constexpr (AssemblerType::template has_feature_for<TagUdp>) {
            udp_adapter_ = std::make_shared<UdpAdpterType>(
                engine_, cfg.udp_server_port,
                [](const std::vector<uint8_t>& data) -> CommandType {
                    return UdpPacketView(data).command();
                });
            AssemblerType::template setup<TagUdp>(udp_adapter_);
        }

        if constexpr (AssemblerType::template has_feature_for<TagCan>) {
            can_adapter_ = std::make_shared<CanAdapterType>(
                engine_, GlobalConfig.GetConfig().can_name.get());
            AssemblerType::template setup<TagCan>(can_adapter_);
        }

        if constexpr (AssemblerType::template has_feature_for<TagInit>) {
            AssemblerType::template setup<TagInit>(engine_->get_context());
        }

        if constexpr (AssemblerType::template has_feature_for<TagListeners>) {
            AssemblerType::template setup<TagListeners>(engine_);
        }

        engine_->get_context().robot->set_target_type(
            mavsdk_adapter_->get_target_type());

        // 3. 启动引擎
        engine_->start<InitState>(std::chrono::milliseconds(50));

        if constexpr (AssemblerType::template has_feature_for<TagMavsdk>) {
            bool is_connected = mavsdk_system_->is_connected();
            engine_->get_context().fcu_connected.store(is_connected);
            engine_->dispatch_internal(FcuConnectedEvent{is_connected});
        }

#ifdef USE_ROS
        asio_thread_ = std::thread([this]() {
            auto work_guard = boost::asio::make_work_guard(global_io_);
            global_io_.run();
        });
#else
        global_io_.run();
#endif
    }

#ifdef USE_ROS2
    std::shared_ptr<rclcpp::Node> get_node() const { return nh_; }
#endif

    ~CoreNode() {
#ifdef USE_ROS
        global_io_.stop();
        if (asio_thread_.joinable()) {
            asio_thread_.join();
        }
#endif
    }
};