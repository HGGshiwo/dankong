#include <dk_auto_json.hpp>

#include "assemble_config.hpp"
#include "context_config.hpp"
#include "core/core_node.hpp"
#include "dk/logger.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "dk_node");
    dk::init_logger();

    CoreNode<RobotAssembler, RobotContext> node;

    // 启动 ROS 异步线程池 (处理网络 IO)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    // 阻塞主线程
    ros::waitForShutdown();
    return 0;
}