# sudo service mosquitto restart
import json
import re
import time

import paho.mqtt.client as mqtt

# ==================== 配置区域 ====================
BROKER_HOST = "127.0.0.1"  # 替换为你的 EMQX Broker 地址
BROKER_PORT = 1883
CLIENT_ID = "cloud_backend_sim_001"
demo_device = "UAV-S-001"

# 后端需要订阅的主题 (Topic, QoS)
SUBSCRIBE_TOPICS = [
    ("device/+/state", 0),  # 接收：设备状态上报 2.0
    ("device/+/progress", 1),  # 接收：任务进度上报 2.0
    ("device/+/abnormal", 1),  # 接收：设备异常上报 2.0
    ("$events/client_connected", 0),  # 接收：EMQX 系统事件-客户端上线
    ("$SYS/broker/log/#", 0),  # 接收：Mosquitto 系统日志-用于捕获客户端上线
]

# ==================== 后端下发函数 (PUBLISH 2.0) ====================


def send_register_v2(client, device_code):
    """
    1. 发送注册 2.0 版本消息
    Topic: register | QoS: 1
    """
    topic = "$exclusive/register"
    payload_dict = {"deviceCode": device_code}
    payload_str = json.dumps(payload_dict, ensure_ascii=False)
    client.publish(topic, payload_str, qos=1)
    print(f"[后端 -> 发送注册 2.0] Topic: {topic} | Payload: {payload_str}")


def send_task_v2(client, device_code):
    """
    2. 下发任务 2.0 版本消息 (支持室内地图与坐标系)[cite: 1]
    Topic: device/{deviceCode}/task | QoS: 1[cite: 1]
    """
    topic = f"device/{device_code}/task"
    payload_dict = {
        "deviceCode": device_code,
        "deviceId": 10,
        "pathEventInfo": [
            [
                {"eventParam": "record_", "eventStatus": "on", "eventType": "video"},
                {"eventParam": "25", "eventStatus": "on", "eventType": "gimbal"},
            ],
            [],
            [],
            [],
            [{"eventStatus": "off", "eventType": "video"}],
        ],
        "pathInfo": [
            [-0.296, -7.498, 0],
            [2.082, -7.229, 0],
            [3.903, -19.817, 0],
            [-3.448, -20.84, 0],
            [-5.279, -8.173, 0],
            [-0.325, -7.518, 0],
        ],
        "mapCode": "C-4",
        "publishTime": int(time.time() * 1000),
        "serialNo": str(int(time.time() * 1000)),
        "taskId": 157,
        "taskName": "园区日常巡检",
    }
    payload_str = json.dumps(payload_dict, ensure_ascii=False)
    client.publish(topic, payload_str, qos=1)
    print(f"[后端 -> 下发任务 2.0] Topic: {topic} | TaskId: 157")


def send_notification_v2(
    client, notice_type="SYSTEM_NOTICE", message="默认全量系统通知"
):
    """
    3. 下发通知 2.0 版本消息[cite: 1]
    Topic: notification | QoS: 0[cite: 1]
    """
    topic = "notification"
    payload_dict = {
        "type": notice_type,
        "obj": {"msg": message, "timestamp": int(time.time() * 1000)},
    }
    payload_str = json.dumps(payload_dict, ensure_ascii=False)
    client.publish(topic, payload_str, qos=0)
    print(f"[后端 -> 发送通知 2.0] Topic: {topic} | Msg: {message}")


def send_video_stream_cmd_v2(client, device_code):
    """
    4. 云端控制播放设备视频流[cite: 1]
    Topic: device/{deviceCode}/video | QoS: 1[cite: 1]
    """
    topic = f"device/{device_code}/video"
    payload_dict = {
        "streamAction": "on",
        "videoCode": "front",
        "streamUrl": f"rtsp://hggshiwo.top:xxxx/video/{device_code}/front",
    }
    payload_str = json.dumps(payload_dict, ensure_ascii=False)
    client.publish(topic, payload_str, qos=1)
    print(f"[后端 -> 视频推流指令] Topic: {topic}")


def send_speak_cmd_v2(client, device_code):
    """
    5. 云端和设备对话指令[cite: 1]
    Topic: device/{deviceCode}/speak | QoS: 1[cite: 1]
    """
    topic = f"device/{device_code}/speak"
    payload_dict = {
        "deviceSpeak": f"rtsp://hggshiwo.top:xxxx/speak/device/{device_code}",
        "cloudSpeak": f"rtsp://hggshiwo.top:xxxx/speak/cloud/{device_code}",
    }
    payload_str = json.dumps(payload_dict, ensure_ascii=False)
    client.publish(topic, payload_str, qos=1)
    print(f"[后端 -> 语音对讲指令] Topic: {topic}")


# ==================== MQTT 5.0 回调处理 ====================


def on_connect(client: mqtt.Client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        print("=== [MQTT 5.0] 后端云服务连接成功！ ===")
        # 订阅全部相关主题[cite: 1]
        client.subscribe(SUBSCRIBE_TOPICS)
        print(f"=== 成功订阅主题: {[t[0] for t in SUBSCRIBE_TOPICS]} ===")
    else:
        print(f"连接失败，返回原因码 Reason Code: {reason_code}")


def on_message(client: mqtt.Client, userdata, msg):
    topic = msg.topic
    payload_str = msg.payload.decode("utf-8", errors="ignore")
    CONNECT_PATTERN = re.compile(r"New client connected from .* as (\S+) \(")
    # 1. 检测设备新连接事件 (通过 EMQX 系统主题 $events/client_connected)[cite: 1]
    # 捕获 Mosquitto 新连接日志
    if topic.startswith("$SYS/broker/log/"):
        match = CONNECT_PATTERN.search(payload_str)
        if match:
            new_client_id = match.group(1)
            # 排除后端自己
            if new_client_id != CLIENT_ID:
                print(
                    f"\n[！新连接建立 (Mosquitto $SYS)] 检测到客户端上线: {new_client_id}"
                )
                # 向 register 主题下发 2.0 注册消息
                send_register_v2(client, demo_device)
        return

    # 2. 处理设备上报状态[cite: 1]
    elif "/state" in topic:
        print(f"\n[收到设备状态上报 2.0] Topic: {topic} | Payload: {payload_str}")

    # 3. 处理任务进度[cite: 1]
    elif "/progress" in topic:
        print(f"\n[收到任务进度上报 2.0] Topic: {topic} | Payload: {payload_str}")

    # 4. 处理异常信息[cite: 1]
    elif "/abnormal" in topic:
        print(f"\n[收到设备异常上报 2.0] Topic: {topic} | Payload: {payload_str}")

    else:
        print(f"\n[收到其他消息] Topic: {topic} | Payload: {payload_str}")


# ==================== 主运行逻辑 ====================


def main():
    # 使用 MQTT 5.0 协议初始化 Client[cite: 1]
    client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv5)

    client.on_connect = on_connect
    client.on_message = on_message

    print(f"正在尝试连接 Broker: {BROKER_HOST}:{BROKER_PORT} (MQTT 5.0)...")
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)

    # 开启后台通信线程
    client.loop_start()

    # 为了演示简单，在后端启动数秒后模拟向某个目标设备发送指令
    time.sleep(2)

    # print("\n--- 正在演示后端主动触发下发 2.0 报文 ---")
    # send_register_v2(client, demo_device)
    # time.sleep(1)
    # send_task_v2(client, demo_device)
    # time.sleep(1)
    send_notification_v2(client, "NOTICE", "后端服务已上线并运行 2.0 协议")
    # time.sleep(1)
    # send_video_stream_cmd_v2(client, demo_device)
    # time.sleep(1)
    # send_speak_cmd_v2(client, demo_device)
    # print("--- 所有下发演示完毕，进入长连接监听模式 ---\n")

    try:
        while True:
            i = input("输入任意字符+回车发送任务")
            send_task_v2(client, demo_device)
            continue
    except KeyboardInterrupt:
        print("\n后端云服务模拟器正常退出。")
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
