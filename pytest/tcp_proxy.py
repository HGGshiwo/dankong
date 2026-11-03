#!/usr/bin/env python
# coding=utf-8
"""线程化 TCP 转发代理: 用于 FCU 断联重连测试。

在测试进程内监听一个端口, 把收到的连接双向转发到目标端口 (SITL)。
stop() 会关闭监听与所有已建连接 —— dankong 端立刻表现为 FCU 断联;
restart() 重新监听 —— MAVSDK 的 TcpClientConnection 每秒自动重试,
重连后指令流经同一连接对象恢复。
"""
import socket
import struct
import threading
import time


class TcpProxy:
    def __init__(self, listen_port, target_port, host="127.0.0.1"):
        self.listen_port = listen_port
        self.target_port = target_port
        self.host = host
        self._stop = threading.Event()
        self._acceptor = None
        self._conns = []  # [(sock_a, sock_b)]
        self._lock = threading.Lock()

    def start(self):
        self._stop.clear()
        self._acceptor = threading.Thread(target=self._accept_loop, daemon=True)
        self._acceptor.start()

    @staticmethod
    def _close_rst(sock):
        """以 RST 而非 FIN 关闭: SITL 的 TCP serial 服务端收到 FIN 不关闭
        自己一侧 (CLOSE-WAIT 挂死, 一次一连接的槽位被占, 不再 accept 新连接),
        收到 ECONNRESET 才会释放槽位。"""
        try:
            sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0)
            )
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def stop(self):
        self._stop.set()
        # 自连一个立刻被关闭的连接, 唤醒阻塞在 accept 的监听线程
        try:
            socket.create_connection((self.host, self.listen_port), timeout=0.2).close()
        except OSError:
            pass
        if self._acceptor:
            self._acceptor.join(timeout=3)
        self._close_all()

    def restart(self):
        self.stop()
        self.start()

    def _accept_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.listen_port))
        srv.listen(8)
        srv.settimeout(0.5)
        try:
            while not self._stop.is_set():
                try:
                    client, _ = srv.accept()
                except socket.timeout:
                    continue
                # 保持客户端连接, 重试等待目标端口就绪 (SITL 重启窗口期
                # 5762 可能暂时关闭); 立即关闭会导致 MAVSDK 反复断联
                target = None
                deadline = time.time() + 15
                while time.time() < deadline and not self._stop.is_set():
                    try:
                        target = socket.create_connection(
                            (self.host, self.target_port), timeout=1
                        )
                        break
                    except OSError:
                        time.sleep(0.5)
                if target is None:
                    client.close()
                    continue
                pair = (client, target)
                with self._lock:
                    self._conns.append(pair)
                for a, b in ((client, target), (target, client)):
                    threading.Thread(
                        target=self._pipe, args=(a, b, pair), daemon=True
                    ).start()
        except OSError:
            pass  # stop() 关闭 socket
        finally:
            srv.close()

    def _pipe(self, src, dst, pair):
        try:
            while True:
                data = src.recv(4096)
                if not data:
                    break
                dst.sendall(data)
        except OSError:
            pass
        finally:
            self._close_pair(pair)

    def _close_pair(self, pair):
        with self._lock:
            if pair in self._conns:
                self._conns.remove(pair)
        for s in pair:
            self._close_rst(s)

    def _close_all(self):
        with self._lock:
            pairs = list(self._conns)
            self._conns.clear()
        for pair in pairs:
            self._close_pair(pair)
