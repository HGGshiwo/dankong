#include "states/state_utils.hpp"

#include "dk/future.hpp"
#include "spdlog/spdlog.h"
#include "states/ground_state.hpp"
#include "states/hover_state.hpp"
#include "states/init_state.hpp"

bool is_prearm_msg(const std::string& text) {
    return text.rfind("PreArm: ", 0) == 0 || text.rfind("Arm: ", 0) == 0;
}

namespace state_utils {
dk::Future<bool> set_mode(RobotContext& ctx, FixedString64 mode) {
    using Promise = dk::Promise<bool>;
    if (ctx.mode.load() == mode) return Promise::resolve(ctx.engine, true);
    ctx.robot->set_mode(mode);

    return ctx.engine->wait_for(1000, [mode](const FlightModeEvent& e) -> bool {
        if (mode != e.cur) return false;
        return true;
    });
}

// TODO: 带上一个停止token，如果重复调用就返回或者做别的事情
dk::Future<bool> prearm_check(RobotContext& ctx) {
    if (!ctx.robot->is_prearm_enable()) {
        return arm_vehicle(ctx);
    }
    ApmParam param = ctx.robot->get_param("ARMING_CHECK", 0);
    int value = IMavlink::unpack<int>(param);
    spdlog::info("prearm_check: ARMING_CHECK={}", value);
    if (value == 0) return dk::Promise<bool>::resolve(ctx.engine, true);  // 禁用起飞检查

    auto check_sensor_health = [&ctx](uint32_t sensor_health) -> bool {
        int bits = 0x10000000;
        if ((sensor_health & bits) == bits) {
            spdlog::info("prearm_check: sensor_health check pass!");
            return true;
        }
        return false;
    };

    if (check_sensor_health(ctx.sensor_health)) {
        return dk::Promise<bool>::resolve(ctx.engine, true);
    }

    // 检查是否已经通过prearm check
    int bits = 0x10000000;
    if ((ctx.sensor_health & bits) == bits) {
        return dk::Promise<bool>::resolve(ctx.engine, true);
    }

    auto p = std::make_shared<dk::Promise<bool>>(ctx.engine);

    // 等待错误信息
    ctx.engine->wait_for(
        1000,
        [p](const StatusTextEvent& status_text) -> bool {
            if (is_prearm_msg(status_text.text)) {
                p->reject(status_text.text);
                return true;
            }
            return false;
        },
        [p, check_sensor_health](const SysStatusEvent& sys_status) -> bool {
            if (check_sensor_health(sys_status.data)) {
                p->resolve(true);
                return true;
            }
            return false;
        });

    ctx.engine->post_future_task([&ctx]() { ctx.robot->run_prearm_checks(); });
    return p->get_future();
}

dk::Future<bool> arm_vehicle(RobotContext& ctx) {
    auto promise = std::make_shared<dk::Promise<bool>>(ctx.engine);

    auto future = dk::Promise<bool>::resolve(ctx.engine, true);
    if (ctx.robot->is_prearm_enable()) {
        spdlog::info("do prearm check");
        future = prearm_check(ctx).catch_error([promise](std::exception_ptr error) -> void {
            spdlog::error("prearm check failed!");
            promise->reject(error);
        });
    }

    return future.then([&ctx, promise]() -> auto {
        if (ctx.arm) return dk::Promise<bool>::resolve(ctx.engine, true);
        // 等待解锁
        auto source = dk::CancellationTokenSource();
        source.cancel_after(3000, ctx.engine);
        auto token = source.get_token();
        ctx.robot->arm();
        return ctx.engine->wait(
            token,
            [promise, token](const ArmEvent& arm_event) -> bool {
                if (token.is_cancelled()) {
                    promise->reject("Wait for arm timeout!");
                    return true;
                }
                if (arm_event.armed) {
                    promise->resolve(true);
                    return true;
                }
                return false;
            },
            [promise, token](const StatusTextEvent& e) -> bool {
                if (token.is_cancelled()) {
                    promise->reject("Wait for arm timeout!");
                    return true;
                }
                if (is_prearm_msg(e.text)) {
                    spdlog::error("arm failed: {}!", e.text);
                    promise->reject(e.text);
                    return true;
                }
                return false;
            });
    });
}

std::shared_ptr<dk::IState<RobotEvent, RobotContext>> check_state(const RobotContext& ctx) {
    if (!ctx.odom_ok) return InitState::instance();
    if (ctx.robot->check_hover(ctx.arm, ctx.pos.get().z())) return HoverState::instance();
    return GroundState::instance();
}

dk::Future<bool> takeoff_vehicle(RobotContext& ctx, double alt) {
    return set_mode(ctx, "GUIDED").then([&ctx, alt]() -> auto {
        if (ctx.robot->check_hover(ctx.arm, ctx.pos.get().z())) {
            spdlog::info("vechicle already in air, change alt to {}", alt);
            Eigen::Vector3d pos = ctx.pos.get();
            pos.z() = alt;
            ctx.robot->send_cmd(pos, std::nullopt, std::nullopt, std::nullopt, std::nullopt, CmdFrame::ENU);
            return dk::Promise<bool>::resolve(ctx.engine, true);
        }
        return arm_vehicle(ctx)
            .then([&ctx, alt]() -> bool {
                ctx.robot->takeoff(alt);
                spdlog::info("send takeoff command done!");
                return true;
            })
            .then([&ctx, alt]() -> bool {
                auto pos = ctx.pos.get();
                ctx.takeoff_pos.set({pos.x(), pos.y(), alt});
                spdlog::info("record takeoff pos: x={:.2f} y={:.2f} z={:.2f}", pos.x(), pos.y(), alt);
                return true;
            });
    });
}

bool check_alt(RobotContext& ctx, double target) {
    double TAKEOFF_RATE = 0.1;
    double TAKEOFF_DIFF_THRSH = 0.6;
    auto pos = ctx.pos.get();
    return std::fabs(pos.z() - target) < std::fmax(target * TAKEOFF_RATE, TAKEOFF_DIFF_THRSH);
}

Eigen::Vector3d gps_to_enu(RobotContext& ctx, double lat, double lon, double alt) {
    Eigen::Vector3d diff = gps_to_enu_body(lat, lon, alt);
    Eigen::Vector3d enu;
    auto pos = ctx.pos.get();
    enu += pos;
    return enu;
}

Eigen::Vector3d gps_to_enu_body(double lat, double lon, double alt) {
    return Eigen::Vector3d::Zero();
}

void do_land(RobotContext& ctx) {
    ctx.robot->land();
}

}  // namespace state_utils