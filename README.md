# Dankong 机器人控制系统 - 架构与开发指南

`dankong` 是一个基于 `dk` 框架构建的工程化机器人应用。机器人系统的代码极易因为硬件接口、算法逻辑和外部通信的相互交织而变成“屎山”。
`dankong` 的核心设计理念就是**解耦**：将外部通信、硬件驱动与核心业务逻辑完全剥离，允许开发者像搭积木一样，模块化地增加机器人的新能力。

---

## 1. 架构理念：认识四大核心角色

要理解 `dankong`，你需要认识系统中的四个核心角色。我们可以按数据流向，从外到内来认识它们。

### 1.1 适配器 (Adapter) - 外部世界的“翻译官”

**设计理念**：业务逻辑绝不应该关心数据是从哪来的。无论是来自局域网的 UDP 数据包、还是 ROS 话题，适配器的任务就是将它们“翻译”成系统听得懂的语言。

**什么时候用什么？**
适配器处理两种截然不同的数据流，必须严格区分：

* **传感数据流 (Sensor Flow) - 观测**：例如 IMU、里程计、视觉图像等高频数据。这类数据**不要**去触发系统事件，而是通过 `bind_context` 直接静默更新到全局 `Context` 中，作为状态机的“观测数据”。
* **业务指令流 (Command Flow) - 驱动**：例如外部发来的“起飞”、“降落”、“急停”指令。这类数据必须通过 `bind_event` 翻译成 `dk` 框架的事件 (Event)，以此来主动触发状态机的跳转。

### 1.2 机器人底层 (Robot) - 物理硬件的“影子”

**设计理念**：业务状态机绝不直接操作硬件。硬件接口千奇百怪，我们需要一个统一的抽象层。

* **硬件无关性**：状态机内部只调用 `ctx.robot->send_command()`。至于底层到底是在 Linux ARM 板上通过 CAN 总线或是 USB-CAN 模块发送控制帧，还是在 Gazebo 环境里跑仿真，状态机一概不关心。这让代码可以在真实硬件和仿真环境间无缝切换。

### 1.3 功能包 (Feature) - 业务逻辑的“集装箱”

**设计理念**：当我们要给机器人增加一个新能力（比如“视觉跟随”），我们希望代码是内聚的，而不是散落在系统各处。

* **资源生命周期管理**：Feature 是一个静态大管家。它在系统启动时通过 `init` 函数，将相关的算法模型、控制器参数（自动解析 YAML 配置）实例化，并统一注入到 `RobotContext` 中，供状态机后续使用。

### 1.4 状态 (State) - 系统的“决策中枢”

**设计理念**：基于 HSM（层次化状态机）设计，确保系统在正确的时间做正确的事，且绝不残留脏数据。

* **局部性与安全**：状态内部定义的变量，在状态退出时会随着类实例一起自动销毁。千万不要在全局 `Context` 里塞满临时中间变量（比如某次计算的临时误差），临时变量只应属于当前状态，从而保证系统内存的绝对整洁。

---

## 2. 典型开发流程：以“自动降落”为例

理解了核心理念，我们来看一个完整的垂直切片开发流程：如何从零增加一个“自动降落”功能。

### 第一步：定义 Feature（准备粮草）

首先，我们在 `RobotContext` 中预留好降落需要的算法槽位，并在 `PlandFeature` 中完成配置加载与初始化。

```cpp
// include/features/pland_feature.hpp
class PlandFeature {
public:
    static void init(RobotContext& ctx) {
        // 从 YAML 中加载 p_gain 等参数，实例化降落控制器，并注入全局上下文
        ctx.land_controller = make_shared<PlandController>(GlobalConfig.GetConfig().land_p);
    }
};

```

### 第二步：配置 Adapter（连通外部神经）

我们需要将摄像头的图像接入系统，并将网页端的降落指令翻译为事件。

```cpp
// 场景 A: 高频传感器数据 -> 直接更新上下文（不触发事件）
ros_adapter->bind_context("/camera/image", [](auto msg, auto& ctx) {
    ctx.pland_image.store(msg->data); 
});

// 场景 B: 明确的业务指令 -> 翻译为事件（驱动状态机流转）
web_adapter->bind_event("/action/land", [](auto msg) {
    return RequestLandEvent{}; 
});

```

### 第三步：编写 State（核心决策逻辑）

利用层次化状态机的特性，我们将“异常处理”放在父状态，将“控制逻辑”放在子状态。

```cpp
// include/states/land_state.hpp

// 1. 父状态：负责兜底。任何降落阶段遇到 AbortEvent 都会被这里捕获。
class LandSuperState : public BaseState<RobotContext, LandSuperState, void> {
public:
    using AllowedEvents = std::tuple<AbortEvent>;
    
    StateAction on_event(const AbortEvent& e, RobotContext& ctx) {
        // 遇到严重错误，立刻跳转到急停状态
        return step<EmergencyStopState>(); 
    }
};

// 2. 子状态：负责干活。继承自 LandSuperState，专注于正常的降落闭环。
class LandingActiveState : public BaseState<RobotContext, LandingActiveState, LandSuperState> {
public:
    using AllowedEvents = std::tuple<TickEvent>; // 依靠系统底层的心跳事件驱动运算
    
    void on_enter(RobotContext& ctx) override {
        // 进入降落状态的第一件事：告诉硬件切换到纯下发模式
        ctx.robot->set_mode("LAND"); 
    }

    StateAction on_event(const TickEvent& e, RobotContext& ctx) {
        // 核心闭环：读取最新图像 -> 算法计算 -> 调用统一 Robot 接口下发指令
        auto cmd = ctx.land_controller->update(ctx.pland_image.load());
        ctx.robot->send_command(cmd);

        // 判断条件：如果落地，安全退出，切入待机状态
        if (ctx.is_landed()) {
            return step<StandbyState>();
        }
        return handled();
    }
};

```

---

## 3. 目录结构：在哪写代码？

保持良好的目录习惯，能让团队协作事半功倍：

* `dk/`: 框架核心底层引擎。（**通常不需要也不应该修改**）
* `include/core/`: 存放系统骨架，比如全局定义的 `RobotContext` 和 YAML 配置的映射类。
* `include/robot/`: 定义底层的 `IRobot` 接口以及具体的硬件/仿真器实现类（如 `CanRobot`, `GazeboRobot`）。
* `include/features/`: **日常开发的主战场 1**。按业务功能建文件夹（如 `pland`, `tracker`），管理该功能独有的数据和算法实例。
* `include/states/`: **日常开发的主战场 2**。状态机树的节点定义，处理真正的决策逻辑。
* `config/`: 所有的 YAML 配置文件存放处。
* `src/`: 主程序入口 `main.cpp`，以及将上述所有组件拼装起来的连线代码。