#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "utils/thread_runner.hpp"

// Linux/POSIX Socket 依赖 (如果是 Windows 需要换成 winsock2.h)
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

#include "utils/dirty_var.hpp"

class NtripClient : public IThreadRunner {
   public:
    // 回调函数签名：对外输出 RTCM 数据块 (参数: 数据指针, 数据长度)
    using RtkCallback = std::function<void(const uint8_t*, size_t)>;
    DirtyVar<bool>& running_;

    NtripClient(std::shared_ptr<dk::ITimeProvider> time_provider,
                DirtyVar<bool>& running)
        : IThreadRunner(std::move(time_provider), true), running_(running) {}

    ~NtripClient() override {
        stop();  // 确保析构时切断网络
    }

    /**
     * @brief 启动 NTRIP 客户端 (外部在 gps_fix_type >= 3 时调用)
     * @param initial_lat 首次请求 VRS 的准确纬度
     * @param initial_lon 首次请求 VRS 的准确经度
     */
    void run(const std::string& host, int port, const std::string& user,
             const std::string& pwd, const std::string& mountpoint,
             double initial_lat, double initial_lon, RtkCallback callback) {
        // 1. 启动前先强制停止并清理上次的连接
        stop();

        // 2. 保存配置
        host_ = host;
        port_ = port;
        user_ = user;
        pwd_ = pwd;
        mountpoint_ = mountpoint;
        lat_ = initial_lat;
        lon_ = initial_lon;
        callback_ = std::move(callback);

        is_connected_ = false;

        // 3. 启动线程运行器 (例如 10Hz 频率循环读取)
        start(10);
    }

    /**
     * @brief 提供给外部的方法：定时更新坐标给服务器以维持 VRS 和心跳
     * 通常建议每 10 秒调用一次
     */
    void send_location_update(double lat, double lon) {
        if (!is_connected_ || socket_fd_ < 0) return;
        std::string gga = generate_gga(lat, lon);
        send(socket_fd_, gga.c_str(), gga.size(), 0);
    }

   protected:
    void on_start() override {
        socket_fd_ = -1;
        is_connected_ = false;
        running_.store(true);
    }

    void on_stop() override {
        close_socket();
        running_.store(false);
    }

    void on_step(double dt) override {
        // 如果还没连接，先执行连接和鉴权逻辑
        if (!is_connected_) {
            if (connect_and_auth()) {
                is_connected_ = true;
                // 连接成功后，立即发送初始坐标获取差分数据
                send_location_update(lat_, lon_);
            } else {
                // 连接失败，休眠等待下一次 on_step 重试 (可以避免疯狂重连占用
                // CPU)
                return;
            }
        }

        // 如果已连接，尝试读取数据
        uint8_t buffer[2048];
        int bytes_read = recv(socket_fd_, buffer, sizeof(buffer), 0);

        if (bytes_read > 0) {
            // 收到数据，触发回调交给 MAVSDK
            if (callback_) {
                callback_(buffer, bytes_read);
            }
        } else if (bytes_read == 0) {
            // 服务器主动断开了连接
            std::cerr
                << "[NtripClient] Server closed connection. Reconnecting..."
                << std::endl;
            close_socket();  // 触发下次 on_step 重新连接
        } else {
            // bytes_read < 0
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                // 发生了真正的网络错误
                std::cerr << "[NtripClient] Socket error. Reconnecting..."
                          << std::endl;
                close_socket();
            }
            // 如果是
            // EAGAIN，说明只是当前没数据，非阻塞模式下的正常现象，直接退出等待下次
            // on_step
        }
    }

   private:
    std::string host_, user_, pwd_, mountpoint_;
    int port_ = 0;
    double lat_ = 0.0, lon_ = 0.0;
    RtkCallback callback_;

    int socket_fd_ = -1;
    bool is_connected_ = false;

    void close_socket() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
        is_connected_ = false;
    }

    bool connect_and_auth() {
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) return false;

        struct sockaddr_in server_addr {};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        server_addr.sin_addr.s_addr = inet_addr(host_.c_str());

        // 如果传入的是域名而不是 IP，需要解析 (简单起见这里假设 host 是 IP)
        // 若需支持域名，可使用 gethostbyname 或 getaddrinfo

        // 1. 建立 TCP 连接
        if (connect(socket_fd_, (struct sockaddr*)&server_addr,
                    sizeof(server_addr)) < 0) {
            close_socket();
            return false;
        }

        // 2. 发送 HTTP GET 鉴权请求
        std::string auth = base64_encode(user_ + ":" + pwd_);
        std::string request = "GET /" + mountpoint_ +
                              " HTTP/1.0\r\n"
                              "User-Agent: NTRIP MyMavsdkClient/1.0\r\n"
                              "Authorization: Basic " +
                              auth +
                              "\r\n"
                              "Accept: */*\r\nConnection: close\r\n\r\n";

        if (send(socket_fd_, request.c_str(), request.size(), 0) < 0) {
            close_socket();
            return false;
        }

        // 3. 读取响应，等待 "200 OK" (这里允许短阻塞，因为在独立线程里)
        char resp_buf[512];
        int resp_len = recv(socket_fd_, resp_buf, sizeof(resp_buf) - 1, 0);
        if (resp_len <= 0) {
            close_socket();
            return false;
        }
        resp_buf[resp_len] = '\0';
        std::string response(resp_buf);

        if (response.find("200 OK") == std::string::npos) {
            std::cerr << "[NtripClient] Auth failed: " << response << std::endl;
            close_socket();
            return false;
        }

        // 4. 鉴权成功，将 Socket 设置为非阻塞模式，供后续 on_step 顺畅读取
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        return true;
    }

    // --- 辅助方法 ---

    // 极其简易的 Base64 编码 (NTRIP 必备)
    std::string base64_encode(const std::string& in) {
        std::string out;
        int val = 0, valb = -6;
        for (unsigned char c : in) {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0) {
                out.push_back(
                    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz012345"
                    "6789+/"[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6)
            out.push_back(
                "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
                "+/"[((val << 8) >> (valb + 8)) & 0x3F]);
        while (out.size() % 4) out.push_back('=');
        return out;
    }

    // 生成标准的 NMEA GGA 语句
    std::string generate_gga(double lat, double lon) {
        char buf[256];
        double lat_min = (std::abs(lat) - std::floor(std::abs(lat))) * 60.0;
        double lon_min = (std::abs(lon) - std::floor(std::abs(lon))) * 60.0;

        int lat_deg = static_cast<int>(std::floor(std::abs(lat)));
        int lon_deg = static_cast<int>(std::floor(std::abs(lon)));

        const char* lat_dir = lat >= 0 ? "N" : "S";
        const char* lon_dir = lon >= 0 ? "E" : "W";

        // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        snprintf(buf, sizeof(buf),
                 "$GPGGA,000000.00,%02d%011.8f,%s,%03d%011.8f,%s,1,10,1.0,100."
                 "0,M,0.0,M,,",
                 lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir);

        // 计算异或校验和
        uint8_t checksum = 0;
        for (int i = 1; buf[i] != '\0'; ++i) {
            checksum ^= buf[i];
        }

        char final_buf[256];
        snprintf(final_buf, sizeof(final_buf), "%s*%02X\r\n", buf, checksum);
        return std::string(final_buf);
    }
};