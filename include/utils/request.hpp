#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "dk/engine.hpp"
#include "dk/future.hpp"
#include "robot_context.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

// 封装一个简单的响应结构体，方便外部判断和获取数据
struct HttpResponse {
    int status_code;
    std::string body;
    std::string error_msg;

    // 辅助函数：判断请求是否真正成功
    bool success() const {
        return error_msg.empty() && status_code >= 200 && status_code < 300;
    }
};

inline dk::Future<HttpResponse> send_request(
    std::shared_ptr<dk::IEngine<RobotContext>> engine, http::verb method,
    std::string ip, const std::string& url, uint16_t port,
    const json& req_data = json{}) {
    return engine->post_background_task<HttpResponse>(
        [port, engine, method, url, req_data, ip]() -> HttpResponse {
            HttpResponse result{0, "", ""};
            try {
                // 1. 直接构造环回地址端点，省去 resolver 带来的延迟和开销
                tcp::endpoint endpoint(net::ip::make_address(ip), port);

                // 2. 绑定外部传入的 ioc，创建并建立 TCP 流
                beast::tcp_stream stream(engine->get_ioc());
                stream.connect(endpoint);

                // 3. 构造 HTTP 请求包 (HTTP/1.1)
                http::request<http::string_body> req{method, url, 11};
                req.set(http::field::host, ip);
                req.set(http::field::user_agent,
                        "Control-Node-Internal-Client");

                // 4. 如果有 JSON 数据，序列化并放入 Body
                if (!req_data.is_null() && !req_data.empty()) {
                    req.set(http::field::content_type, "application/json");
                    req.body() = req_data.dump();
                    req.prepare_payload();  // 关键：自动计算并设置
                                            // Content-Length 请求头
                }

                // 5. 执行同步发送
                http::write(stream, req);

                // 6. 声明接收缓冲区与响应容器，执行同步阻塞接收
                beast::flat_buffer buffer;
                http::response<http::string_body> res;
                http::read(stream, buffer, res);

                // 7. 提取结果
                result.status_code = res.result_int();
                result.body = res.body();

                // 8. 优雅地关闭 TCP Socket
                beast::error_code ec;
                stream.socket().shutdown(tcp::socket::shutdown_both, ec);

                // not_connected 是正常关流时的预期错误，可忽略
                if (ec && ec != beast::errc::not_connected) {
                    result.error_msg = "Shutdown warning: " + ec.message();
                }

            } catch (const std::exception& e) {
                // 捕获连接失败、拒绝连接等系统级异常
                result.error_msg = e.what();
            }

            return result;
        });
}