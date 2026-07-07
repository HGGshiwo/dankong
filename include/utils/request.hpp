#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/beast/version.hpp>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "dk/engine.hpp"
#include "dk/future.hpp"
#include "robot_context.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
using tcp = net::ip::tcp;
using json = nlohmann::json;

struct HttpResponse {
    int status_code;
    std::string body;
    std::string error_msg;

    bool success() const {
        return error_msg.empty() && status_code >= 200 && status_code < 300;
    }
};

// 辅助结构体：存储解析后的 URL 信息
struct ParsedUrl {
    std::string protocol;
    std::string host;
    std::string port;
    std::string path;
    bool is_https() const { return protocol == "https"; }
};

// 辅助函数：解析完整的 URL (如 https://192.168.1.1:8080/api/test)
inline ParsedUrl parse_url(const std::string& full_url) {
    ParsedUrl result;
    std::string remaining = full_url;

    // 1. 提取协议 (http or https)
    auto pos = remaining.find("://");
    if (pos != std::string::npos) {
        result.protocol = remaining.substr(0, pos);
        remaining = remaining.substr(pos + 3);
    } else {
        result.protocol = "http";  // 默认 fallback 为 http
    }

    // 2. 提取路径 (Path)
    pos = remaining.find('/');
    if (pos != std::string::npos) {
        result.path = remaining.substr(pos);
        remaining = remaining.substr(0, pos);  // 剩下 host:port 或 host
    } else {
        result.path = "/";
    }

    // 3. 提取主机(Host)和端口(Port)
    pos = remaining.find(':');
    if (pos != std::string::npos) {
        result.host = remaining.substr(0, pos);
        result.port = remaining.substr(pos + 1);
    } else {
        result.host = remaining;
        result.port = result.is_https() ? "443" : "80";  // 根据协议给默认端口
    }

    return result;
}

// 辅助函数：URL 编码
inline std::string url_encode(const std::string& value) {
    std::ostringstream escaped;
    escaped.fill('0');
    escaped << std::hex;
    for (char c : value) {
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '-' ||
            c == '_' || c == '.' || c == '~') {
            escaped << c;
        } else {
            escaped << std::uppercase << '%' << std::setw(2)
                    << static_cast<int>(static_cast<unsigned char>(c));
        }
    }
    return escaped.str();
}

// 修改后的主函数：参数替换为 const std::string& full_url
inline dk::Future<HttpResponse> send_request(
    std::shared_ptr<dk::IEngine<RobotContext>> engine, http::verb method,
    const std::string& full_url, const json& req_data = json{},
    bool is_form = false) {
    return engine->post_background_task<HttpResponse>([engine, method, full_url,
                                                       req_data, is_form]()
                                                          -> HttpResponse {
        HttpResponse result{0, "", ""};
        try {
            // 1. 解析完整的 URL
            ParsedUrl parsed = parse_url(full_url);

            // 2. 使用 resolver 解析主机名。
            // 这里的 host 既可以是 "192.168.1.1" 也可以是 "www.baidu.com"。
            // Asio 的 resolver 会自动识别，如果是纯 IP 则不会产生 DNS
            // 查询开销。
            tcp::resolver resolver(engine->get_ioc());
            auto const results = resolver.resolve(parsed.host, parsed.port);

            // 3. 构造 HTTP 请求包
            http::request<http::string_body> req{method, parsed.path, 11};
            req.set(http::field::host, parsed.host);
            req.set(http::field::user_agent, "Control-Node-Internal-Client");

            // 4. 处理数据：JSON 或 Form 格式
            if (!req_data.is_null() && !req_data.empty()) {
                if (is_form) {
                    req.set(http::field::content_type,
                            "application/x-www-form-urlencoded");
                    std::string form_body;
                    for (auto it = req_data.begin(); it != req_data.end();
                         ++it) {
                        if (!form_body.empty()) form_body += "&";
                        std::string val = it.value().is_string()
                                              ? it.value().get<std::string>()
                                              : it.value().dump();
                        form_body +=
                            url_encode(it.key()) + "=" + url_encode(val);
                    }
                    req.body() = form_body;
                } else {
                    req.set(http::field::content_type, "application/json");
                    req.body() = req_data.dump();
                }
                req.prepare_payload();
            }

            beast::flat_buffer buffer;
            http::response<http::string_body> res;

            // 5. 根据协议类型发起连接和请求
            if (parsed.is_https()) {
                ssl::context ctx(ssl::context::tlsv12_client);
                ctx.set_verify_mode(
                    ssl::verify_none);  // 内部节点互联一般不校验本地证书
                beast::ssl_stream<beast::tcp_stream> stream(engine->get_ioc(),
                                                            ctx);

                // 关键增强：设置 SNI (Server Name Indication)。
                // 很多现代 HTTPS 服务器（如 Nginx、云服务）依赖 SNI
                // 路由，否则握手会直接断开。
                if (!SSL_set_tlsext_host_name(stream.native_handle(),
                                              parsed.host.c_str())) {
                    beast::error_code ec{static_cast<int>(::ERR_get_error()),
                                         net::error::get_ssl_category()};
                    throw beast::system_error{ec};
                }

                // 建立连接并握手
                beast::get_lowest_layer(stream).connect(results);
                stream.handshake(ssl::stream_base::client);

                http::write(stream, req);
                http::read(stream, buffer, res);

                beast::error_code ec;
                stream.shutdown(ec);
                if (ec && ec != beast::errc::not_connected &&
                    ec != ssl::error::stream_truncated) {
                    result.error_msg = "SSL Shutdown warning: " + ec.message();
                }
            } else {
                beast::tcp_stream stream(engine->get_ioc());
                // HTTP 直连
                stream.connect(results);

                http::write(stream, req);
                http::read(stream, buffer, res);

                beast::error_code ec;
                stream.socket().shutdown(tcp::socket::shutdown_both, ec);
                if (ec && ec != beast::errc::not_connected) {
                    result.error_msg = "Shutdown warning: " + ec.message();
                }
            }

            // 6. 提取结果
            result.status_code = res.result_int();
            result.body = res.body();

        } catch (const std::exception& e) {
            result.error_msg = e.what();
        }

        return result;
    });
}