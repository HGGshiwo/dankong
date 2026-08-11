- 单控：[https://github.com/HGGshiwo/dankong.git](https://github.com/HGGshiwo/dankong.git)

- 事件引擎：[https://github.com/HGGshiwo/dk](https://github.com/HGGshiwo/dk)

- 前端：[https://github.com/HGGshiwo/dk/tree/main/frontend](https://github.com/HGGshiwo/dk/tree/main/frontend)

- 仿真：[https://github.com/HGGshiwo/gazebo_sim](https://github.com/HGGshiwo/gazebo_sim)

# Dankong 机器人控制系统 - 架构与开发指南

`dankong` 是一个基于 `dk` 框架构建的高内聚、低耦合机器人控制系统。针对机器人开发中硬件接口、算法逻辑与通信协议易交织耦合的痛点，系统确立了**分层解耦与模块化**的设计原则，使外部通信、底层硬件抽象与核心决策业务完全隔离，支持机器人能力的标准化拓展。

---

## 1. 核心架构与模块职责划分

系统在逻辑上划分为四个核心模块，构建单向清晰的数据流与控制流：

### 1.1 协议与数据适配层 (Adapter)
* **设计原则**：隔离外部通信协议（如 ROS 话题、Websocket、UDP/TCP 报文等），将外部数据标准化转换后接入核心系统。
* **数据流向分类**：
  * **传感观测流 (Observation Flow)**：如 IMU、里程计、视觉图像等高频数据。通过 `bind_context` 直接以无锁或线程安全方式更新至全局 `Context`，作为状态机的观测依据，不产生事件开销。
  * **业务指令流 (Command Flow)**：如外部下发的起飞、航线、急停等控制指令。通过 `bind_event` 转化为框架事件 (Event)，驱动分层状态机与任务编排引擎。

### 1.2 硬件抽象层 (HAL / Robot)
* **设计原则**：为上层业务提供统一、标准化的运动控制接口（`IRobot`），实现业务逻辑与具体硬件实现的解耦。
* **跨平台兼容**：状态机仅面向统一接口下发控制量，底层硬件实现可无缝在嵌入式硬件驱动（如 CAN 总线驱动、串口通信）与仿真环境（如 Gazebo、AirSim）之间切换。

### 1.3 功能特性模块 (Feature)
* **设计原则**：按业务功能（如视觉伺服、精准降落、路径规划等）进行高内聚封装。
* **生命周期管理**：在系统启动时通过静态 `init` 接口完成 YAML 配置文件解析、算法控制器实例化，并注入至 `RobotContext` 中供状态机调用。

### 1.4 分层状态机核心 (Hierarchical State Machine, HSM)
* **设计原则**：基于分层状态机（HSM）组织机器人的行为决策树，确保状态跃迁的安全可控与状态生命周期的严格清理。
* **局部性与隔离性**：状态运行期间的中间计算量封装于状态实例内部，随状态退栈自动销毁，避免在全局上下文中残留脏数据。

---

## 2. 标准功能开发流程：以“自动降落 (Pland)”为例

### 第一步：功能模块定义与初始化 (Feature Definition)
在 `RobotContext` 中声明所需控制器句柄，并在 `PlandFeature` 中完成参数加载与实例化：

```cpp
// include/features/pland_feature.hpp
class PlandFeature {
public:
    static void init(RobotContext& ctx) {
        // 读取配置参数，初始化降落控制器并注入全局上下文
        ctx.land_controller = std::make_shared<PlandController>(GlobalConfig.GetConfig().land_p);
    }
};
```

### 第二步：通信与数据绑定 (Adapter Binding)
配置通信适配器，将传感器数据与控制指令分别绑定到上下文和事件系统：

```cpp
// 场景 A: 高频传感器数据 -> 静默写入全局上下文
ros_adapter->bind_context("/camera/image", [](auto msg, auto& ctx) {
    ctx.pland_image.store(msg->data); 
});

// 场景 B: 外部控制指令 -> 转化为系统事件
web_adapter->bind_event("/action/land", [](auto msg) {
    return RequestLandEvent{}; 
});
```

### 第三步：状态机节点实现 (State Implementation)
利用分层状态机（HSM）的继承特性，在父状态实现全局安全兜底，在子状态执行闭环控制：

```cpp
// include/states/land_state.hpp

// 1. 父状态：负责异常捕获与安全兜底
class LandSuperState : public BaseState<RobotContext, LandSuperState, void> {
public:
    using AllowedEvents = std::tuple<AbortEvent>;
    
    StateAction on_event(const AbortEvent& e, RobotContext& ctx) {
        // 接收到终止事件，立即跳转至急停状态
        return step<EmergencyStopState>(); 
    }
};

// 2. 子状态：负责闭环控制逻辑
class LandingActiveState : public BaseState<RobotContext, LandingActiveState, LandSuperState> {
public:
    using AllowedEvents = std::tuple<TickEvent>;
    
    StateAction on_enter(RobotContext& ctx) override {
        ctx.robot->set_mode("LAND"); 
        return StateAction::handled();
    }

    StateAction on_event(const TickEvent& e, RobotContext& ctx) {
        // 闭环控制：读取感知数据 -> 计算控制量 -> 下发至硬件抽象层
        auto cmd = ctx.land_controller->update(ctx.pland_image.load());
        ctx.robot->send_command(cmd);

        // 终止条件判断
        if (ctx.is_landed()) {
            return step<StandbyState>();
        }
        return handled();
    }
};
```

---

## 3. 工程目录规范

* `dk/`: 框架底层核心引擎（事件分发、HSM 状态机、任务流水线调度）。
* `include/core/`: 系统基础设施，包含全局 `RobotContext` 与配置映射结构。
* `include/robot/`: 硬件抽象层 `IRobot` 接口及各机型/仿真实现（如 `CanRobot`, `GazeboRobot`）。
* `include/features/`: 功能特性模块，负责算法实例与业务参数的内聚管理。
* `include/states/`: 状态机树节点定义与业务决策逻辑实现。
* `config/`: 系统 YAML 配置文件。
* `src/`: 程序入口 `main.cpp` 及各模块组装逻辑。

---

## 4. `dk` 引擎核心接口与状态机开发指南

`dk` 是 Dankong 底层的高性能、事件驱动型分层状态机（HSM）与任务计划编排引擎。

```
                    外部事件 (Event) / 周期心跳 (Tick)
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │ 全局监听器 EventListener  │ (支持返回 StateAction::plan)
                    └─────────────┬─────────────┘
                                  │
                                  ▼
                    ┌───────────────────────────┐
                    │  引擎派发与 HSM 树形冒泡   │
                    └─────────────┬─────────────┘
                                  │
          ┌───────────────────────┼───────────────────────┐
          ▼                       ▼                       ▼
   StateAction::step       StateAction::plan       StateAction::next
      (立即状态跳转)       (声明式任务编排)            (推进任务队列)
```

---

### 4.1 状态生命周期函数（State Lifecycle）

每个状态继承自 `dk::BaseState<Context, Derived, Parent>`，具有严谨的生命周期钩子：

| 接口 | 执行时机 | 核心职责与返回值说明 |
| :--- | :--- | :--- |
| `static StateAction before_enter(Context& ctx, Args...)` | **状态构造与压栈前**（静态准入守卫） | **前置条件校验与任务插队**：<br>• 返回 `StateAction::handled()`：准入通过，正常进入当前状态。<br>• 返回 `StateAction::plan(...)`：声明前置任务（如插队 `ArmState`），引擎自动让渡并调度新任务。<br>• 返回 `StateAction::unhandled()`：条件不满足拒绝进入，状态机保持在原状态。 |
| `StateAction on_enter(Context& ctx)` | **状态压栈成功后** | **动作触发与初始化**：下发控制指令、启动定时器；支持返回 `StateAction::step` 实现瞬态二次跳转。 |
| `StateAction on_tick(double dt, Context& ctx)` | **周期性 Tick 调度** | **闭环计算与控制**：高频读取观测数据，更新控制器并下发指令。 |
| `StateAction on_event(const Event& e, Context& ctx)` | **关注的业务事件到达** | **事件驱动响应**：处理外部指令，返回相应 `StateAction` 控制状态跳转或任务编排。 |
| `void on_exit(Context& ctx)` | **状态退栈销毁前** | **资源释放与安全清理**：停止运动输出、重置标志位、释放局部资源。 |

---

### 4.2 决策意图 `StateAction`

状态函数与事件监听器统一通过返回 `StateAction` 向引擎表达执行意图：

```cpp
// 1. 立即跳转到目标状态（支持参数透传）
return StateAction::step<TargetState>(arg1, arg2);

// 2. 推进至任务计划队列中的下一个状态
return StateAction::next();

// 3. 消费当前事件（阻止继续向父状态冒泡）
return StateAction::handled();

// 4. 忽略当前事件（允许事件继续向上冒泡）
return StateAction::unhandled();

// 5. 声明式编排/重构任务流水线（支持 Lambda 闭包）
return StateAction::plan([](auto& plan, auto& ctx) {
    plan.clear()
        .template push_back<ArmState>()
        .template push_back<TakeoffState>(1.5)
        .template push_back<WaypointState>(route)
        .template fallback<HoverState>();
});
```

---

### 4.3 任务计划管理器 `PlanManager`

`ctx.engine->plan()` 提供了链式任务流水线调度能力：

* **`push_back<State>(args...)`**：在任务队列尾部追加步骤。
* **`push_front<State>(args...)`**：在队头插入高优先级任务。
* **`fallback<State>(args...)`**：设置任务队列执行完毕后的保底待命状态（如 `HoverState`）。
* **`clear()`**：清空当前队列中所有未执行的任务。
* **`start()` / `advance()`**：驱动队列执行当前队头任务。

---

### 4.4 经典应用场景代码示例

#### 示例 A：前置校验与动态任务插队（`before_enter`）
在进入起飞状态前若检测到未解锁，返回 `StateAction::plan` 在队头插入 `ArmState`，起飞任务自动在后方排队：

```cpp
// include/states/takeoff_state.hpp
class TakeoffState : public dk::BaseState<RobotContext, TakeoffState, void> {
public:
    static StateAction before_enter(RobotContext& ctx, const TakeoffEvent& e) {
        if (!ctx.arm.load()) {
            StateFlags flags{.is_takeoff = true};
            if (ctx.robot->should_arm_before_enter(flags)) {
                // 声明前置插队任务，引擎自动先调度 ArmState
                return StateAction::plan([](auto& plan, auto& ctx) {
                    plan.template push_front<ArmState>();
                });
            }
        }
        return StateAction::handled(); // 校验通过，正常进入起飞
    }

    StateAction on_enter(RobotContext& ctx) override {
        ctx.robot->takeoff(alt_);
        return StateAction::unhandled();
    }
};
```

#### 示例 B：在 EventListener 中声明式编排任务流水线
在全局监听器中接收外部控制事件，通过 `StateAction::plan` 直接返回整套动作流：

```cpp
// src/features/control/event_listener.cpp
StateAction ControlEventListener::on_event(const TakeoffEvent& event, RobotContext& ctx) {
    if (!ctx.engine->is_active_state<GroundState>()) {
        event.reject("只允许在地面状态起飞!");
        return StateAction::handled();
    }

    // 声明式返回任务编排，引擎自动接管、构建并驱动执行
    return StateAction::plan([event](auto& plan, auto& ctx) {
        plan.clear()
            .template push_back<TakeoffState>(std::tuple(std::move(event)))
            .template fallback<HoverState>();
    });
}
```

#### 示例 C：分层状态机（HSM）异常拦截与任务推进
子状态专注局部闭环控制，父状态统一处理全局异常保护：

```cpp
// 1. 父状态：负责低电量、通信中断等异常保护
class MissionSuperState : public BaseState<RobotContext, MissionSuperState, void> {
public:
    using AllowedEvents = std::tuple<LowBatteryEvent, LinkLostEvent>;

    StateAction on_event(const LowBatteryEvent& e, RobotContext& ctx) {
        spdlog::warn("Low battery detected! Initiating RTL...");
        return step<RTLState>(); // 异常拦截，强制切入返航状态
    }
};

// 2. 子状态：执行航线巡检任务
class InspectingState : public BaseState<RobotContext, InspectingState, MissionSuperState> {
public:
    using AllowedEvents = std::tuple<TickEvent, InspectionDoneEvent>;

    StateAction on_event(const InspectionDoneEvent& e, RobotContext& ctx) {
        return next(); // 巡检点完成，推进至计划队列中的下一个任务
    }
};
```