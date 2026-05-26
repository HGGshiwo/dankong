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
#include "utils/yaml_helper.hpp"

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
    CoreNode(const std::string config_path) : nh_() {
        GlobalConfig.load(get_executable_dir() / config_path);

        auto& cfg = GlobalConfig.GetConfig();

        // 1. 初始化基础引擎
        engine_ = std::make_shared<Engine>();
        engine_->get_context().engine = engine_;

        boost::filesystem::path config_dir =
            (get_executable_dir() / config_path).parent_path();
        // 生成配置文件 (供前端使用)
        auto yaml_cfg =
            YamlHelper::load_with_base(config_dir / cfg.ui_config.get());
        auto json_cfg = YamlHelper::yaml_to_json(yaml_cfg);

        YamlHelper::save(json_cfg, get_executable_dir() / cfg.json_path.get());

        // 创建适配器
        web_adapter_ =
            std::make_shared<WebAdapterType>(engine_, cfg.server_port);
        engine_->get_context().ws_manager = web_adapter_->get_manager();

        // 2. 调用装配工厂，一键注册全部路由和回调！
        AssemblerType::setup_web(web_adapter_);

        if constexpr (AssemblerType::template has_ros<RosAdapterType>) {
            ros_adapter_ = std::make_shared<RosAdapterType>(engine_, nh_);
            AssemblerType::setup_ros(ros_adapter_);
        }

        if constexpr (AssemblerType::template has_udp<UdpAdpterType>) {
            udp_adapter_ = std::make_shared<UdpAdpterType>(
                engine_, cfg.udp_server_port,
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