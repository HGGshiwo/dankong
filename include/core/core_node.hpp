#include <boost/filesystem/operations.hpp>
#include <cstdint>
#include <memory>
#include <thread>

#include "./engine.hpp"
#include "core/global_config.hpp"
#include "core/tag.hpp"
#include "dk/AsioTimeProvider.hpp"
#include "dk/ITimeProvider.hpp"
#include "dk/adapters/can/can_adapter.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "dk/adapters/web.hpp"
#include "dk/adapters/web/adapter.hpp"
#include "features/dog/command.hpp"
#include "robot_context.hpp"
#include "states/init_state.hpp"
#include "utils/get_executable_path.hpp"
#include "utils/logger.hpp"
#include "utils/yaml_helper.hpp"

#ifdef USE_ROS
#include "dk/RosTimeProvider.hpp"
#include "dk/adapters/ros.hpp"
#include "ros/node_handle.h"
#endif

template <typename AssemblerType, typename ContextType>
class CoreNode {
#ifdef USE_ROS
    using RosAdapterType = dk::RosAdapter<ContextType, Engine>;
#endif
    using WebAdapterType = dk::WebAdapter<ContextType, Engine>;
    using UdpAdpterType = dk::UdpAdapter<ContextType, Engine, CommandType>;
    using CanAdapterType = dk::CanAdapter<ContextType, Engine>;

   private:
    boost::asio::io_context global_io_;
    std::shared_ptr<dk::ITimeProvider> time_provider_;
    std::shared_ptr<Engine> engine_;

#ifdef USE_ROS
    ros::NodeHandle nh_;
    std::shared_ptr<RosAdapterType> ros_adapter_;
    std::thread asio_thread_;
#endif
    std::shared_ptr<WebAdapterType> web_adapter_;
    std::shared_ptr<UdpAdpterType> udp_adapter_;
    std::shared_ptr<CanAdapterType> can_adapter_;

   public:
    CoreNode(const std::string config_path) {
#ifdef USE_ROS
        time_provider_ = std::make_shared<dk::RosTimeProvider>(nh_, global_io_);
#else
        time_provider_ = std::make_shared<dk::AsioTimeProvider>(global_io_);
#endif

        GlobalConfig.load(get_config_dir(config_path));

        auto& cfg = GlobalConfig.GetConfig();
        init_logger();

        // 1. 初始化基础引擎
        engine_ = std::make_shared<Engine>(global_io_, time_provider_);
        engine_->get_context().engine = engine_;

        boost::filesystem::path config_dir =
            get_config_dir(config_path).parent_path();
        // 生成配置文件 (供前端使用)
        auto yaml_cfg =
            YamlHelper::load_with_base(config_dir / cfg.ui_config.get());
        auto json_cfg = YamlHelper::yaml_to_json(yaml_cfg);

        YamlHelper::save(json_cfg, get_config_dir(cfg.json_path.get()));

        // 创建适配器
        web_adapter_ =
            std::make_shared<WebAdapterType>(engine_, cfg.server_port);
        engine_->get_context().ws_manager = web_adapter_->get_manager();

        // 2. 调用装配工厂，一键注册全部路由和回调！
        AssemblerType::template setup<TagWeb>(web_adapter_);

#ifdef USE_ROS
        if constexpr (AssemblerType::template has_feature_for<TagRos>) {
            ros_adapter_ = std::make_shared<RosAdapterType>(engine_, nh_);
            AssemblerType::template setup<TagRos>(ros_adapter_);
        }
#endif

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

        // 3. 启动引擎
        engine_->start<InitState>(std::chrono::milliseconds(50));

#ifdef USE_ROS
        asio_thread_ = std::thread([this]() {
            auto work_guard = boost::asio::make_work_guard(global_io_);
            global_io_.run();
        });
#else
        global_io_.run();
#endif
    }

    ~CoreNode() {
#ifdef USE_ROS
        global_io_.stop();
        if (asio_thread_.joinable()) {
            asio_thread_.join();
        }
#endif
    }
};