#include <CLI/CLI.hpp>
#include <dk_auto_json.hpp>
#include <string>

#include "core/core_node.hpp"
#ifdef USE_ROS2
#include <rclcpp/rclcpp.hpp>
#endif
#include "core/global_config.hpp"
#include "robot_assemble.hpp"
#include "robot_context.hpp"
#include "utils/get_executable_path.hpp"

int main(int argc, char** argv) {
    CLI::App app{"dk"};
    CLIArgs args;

    app.allow_extras(true);
    app.add_option("-c,--config", args.config_path, "config file path");
    app.add_option("-p,--port", args.port, "web server port");
    app.add_option("--camera_port", args.camera_port, "camera port");
    app.add_option("--mavsdk_url", args.mavsdk_url, "fcu url");

    // 解析，如果传入了 -h 或 --help，会自动打印帮助并退出
    CLI11_PARSE(app, argc, argv);
#ifdef USE_ROS1
    ros::init(argc, argv, "dk_node");
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
#endif
    CoreNode<RobotAssembler, RobotContext> node{args};

#ifdef USE_ROS1
    // 启动 ROS 异步线程池 (处理网络 IO)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // 阻塞主线程
    ros::waitForShutdown();
#elif defined(USE_ROS2)
    rclcpp::spin(node.get_node());
    rclcpp::shutdown();
#endif
    return 0;
}