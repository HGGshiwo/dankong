#pragma once
#include <cstring>
#include <memory>
#include <vector>

#include "dk/adapters/udp/udp_client.hpp"
#include "features/dji/command.hpp"
#include "robot/irobot.hpp"
#include "robot_context.hpp"

// 纯 UDP 驱动的 DJI 无人机硬件逻辑实现
class DjiDrone : public IRobot {
   private:
    RobotContext& ctx_;
    uint32_t seq_;  // 数据包序列号累加器

    // 统一的打包辅函数，利用我们设计的 POD 结构体实现零拷贝组装
    template <typename PayloadType>
    std::vector<uint8_t> pack_dji_cmd(MsgId msg_id,
                                      const PayloadType& payload) {
        size_t total_size = sizeof(PacketHeader) + sizeof(PayloadType);
        std::vector<uint8_t> buffer(total_size);

        // 强转头部并赋值
        auto* header = reinterpret_cast<PacketHeader*>(buffer.data());
        header->magic = MAGIC_HEADER;
        header->msg_id = static_cast<uint16_t>(msg_id);
        header->seq = seq_++;
        header->payload_len = sizeof(PayloadType);

        // 拷贝 Payload
        std::memcpy(buffer.data() + sizeof(PacketHeader), &payload,
                    sizeof(PayloadType));

        return buffer;
    }

   public:
    DjiDrone(std::shared_ptr<IMavlink> mavlink, RobotContext& ctx)
        : IRobot(mavlink), ctx_(ctx), seq_(0) {
        // 由于是纯 UDP，这里的 mavlink 可能为空或者仅占位
    }

    // 核心速度控制：将 Eigen 向量直接映射到 DJI UDP 的 CmdVelocity 结构中
    bool cmd_vel(Eigen::Vector4d vel) override {
        CmdVelocity cmd_payload;
        // 注意：根据你的约定，确定 X, Y, Z, W 对应的物理量，这里默认 vx, vy,
        // vz, yaw_rate
        cmd_payload.vx = static_cast<float>(vel.x());        // 前后
        cmd_payload.vy = static_cast<float>(vel.y());        // 左右
        cmd_payload.vz = static_cast<float>(vel.z());        // 上下
        cmd_payload.yaw_rate = static_cast<float>(vel.w());  // 偏航角速度
        cmd_payload.frame = 0;  // 0: 机体坐标系(FLU)

        auto packet = pack_dji_cmd(MsgId::CMD_VELOCITY, cmd_payload);
        ctx_.udp_client->send(packet);
        return true;
    }

    bool inner_check_hover(unsigned int drone_state) override {
        // 对于无人机，可以直接读取我们上下文中的字符串状态来判断悬停
        std::string current_mode = ctx_.mode.load();
        return current_mode == "LOITER" || current_mode == "GUIDED" ||
               current_mode == "AUTO";
    }

    bool inner_is_landed(unsigned int drone_state) override {
        // 飞行模式为 LAND 或者 桨叶未解锁即认为降落
        return !ctx_.arm.load() || ctx_.mode.load() == "LAND";
    }

    // 无人机必须使用 3D 欧氏距离
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override {
        return (pos - goal).norm();
    }

    bool is_prearm_enable() override {
        return true;
    }  // 无人机通常需要解锁前检查

    bool is_alt_enable() override { return true; }  // 无人机拥有高度维度

    bool land() override {
        CmdBasic cmd_payload;
        cmd_payload.cmd_type = static_cast<uint8_t>(BasicCmdType::LAND);
        cmd_payload.param1 = 0.0f;

        auto packet = pack_dji_cmd(MsgId::CMD_BASIC, cmd_payload);
        ctx_.udp_client->send(packet);
        return true;
    }

    bool loiter() override {
        // 1. 发送切模式指令到悬停 (或者你们系统内部的停止指令)
        CmdSetMode cmd_payload = {0};
        std::strncpy(cmd_payload.mode_name, "LOITER",
                     sizeof(cmd_payload.mode_name) - 1);

        auto packet = pack_dji_cmd(MsgId::CMD_SET_MODE, cmd_payload);
        ctx_.udp_client->send(packet);

        // 2. 双重保险：同时下发零速度摇杆量
        cmd_vel(Eigen::Vector4d::Zero());
        return true;
    }

    bool takeoff(double alt) override {
        CmdBasic cmd_payload;
        cmd_payload.cmd_type = static_cast<uint8_t>(BasicCmdType::TAKEOFF);
        cmd_payload.param1 = static_cast<float>(alt);  // 指定起飞高度

        auto packet = pack_dji_cmd(MsgId::CMD_BASIC, cmd_payload);
        ctx_.udp_client->send(packet);
        return true;
    }
};