#include <CLI/CLI.hpp>
#include <dk_auto_json.hpp>
#include <string>

#include "core/core_node.hpp"
#include "core/global_config.hpp"
#include "robot_assemble.hpp"
#include "robot_context.hpp"
#include "utils/get_executable_path.hpp"

int main(int argc, char** argv) {
    CLI::App app{"dk"};
    std::string config_path;

    app.allow_extras(true);
    app.add_option("-c,--config", config_path, "config file path");
    // 解析，如果传入了 -h 或 --help，会自动打印帮助并退出
    CLI11_PARSE(app, argc, argv);

    ros::init(argc, argv, "dk_node");
    CoreNode<RobotAssembler, RobotContext> node{config_path};

    // 启动 ROS 异步线程池 (处理网络 IO)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // 阻塞主线程
    ros::waitForShutdown();
    return 0;
}