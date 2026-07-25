# Web、HTTP 与 OTA

## 服务结构

固件使用 ESP-IDF `esp_http_server`。控制页面、配置和抓拍接口运行在主 HTTP 端口；MJPEG `/stream` 运行在主端口加 1 的服务器上。内嵌前端在缺省端口 80 时连接 `http://<设备>:81/stream`，断流后等待 2 秒并附加时间戳重连。

两个主要页面为：

- `/gpio`：RoboFish FPV 驾驶页面，对应恢复文件 `html_01_0x0000100b.html`；
- `/partner`：Partner Lab 动作脚本页面，对应 `html_02_0x0001f6ad.html`。

完整路由及处理函数地址见 [http_routes.tsv](metadata/http_routes.tsv)。相机服务器的静态 `httpd_uri_t` 表位于 DROM `0x3c0df58c` 附近，是处理函数地址的直接证据。

## 驾驶控制协议

驾驶页通过 GET `/control?var=<名称>&val=<整数>` 发送动作：

| UI 动作 | `var` | `val` |
| --- | --- | ---: |
| 前进 | `servo_forward` | 1 |
| 后退 | `servo_backward` | 1 |
| 左转 | `servo_left` | 1 |
| 右转 | `servo_right` | 1 |
| 停止 | `servo_stop` | 1 |
| 上浮俯仰 | `pitch` | 45 |
| 下潜俯仰 | `pitch` | 135 |
| 俯仰复位 | `pitch` | 90 |
| 水泵开/关 | `pump` | 1/0 |
| 模式轮换 | `mode` | 0 |
| Demo 启动/取消 | `demo` | 1 |

`/control` 同时保留 ESP32 Camera 的传感器参数分派，包括 `framesize`、`quality`、`contrast`、`brightness`、`saturation`、`gainceiling`、`colorbar`、`hmirror`、`vflip`、`awb_gain`、`agc_gain`、`aec_value`、`aec2`、`raw_gma`、`lenc`、`special_effect`、`wb_mode`、`ae_level` 和 `led_intensity`。

前端对相同非 stop 动作实施 80 ms 节流；`stop` 只有在运动状态变为停止时才发送。请求使用 `cache: no-store` 和 `keepalive: true`，用于降低快速操控对图传连接的干扰。

## 页面显示并非完整遥测

驾驶页每 5 秒更新一次显示，但代码明确表明：

- 深度恒为 `0.0 m`；
- 速度恒为 `0.0 m/s`；
- 电量恒为 `95%`；
- 温度在 20～28 °C 之间随机生成；
- RSSI 在 -40～-80 dBm 之间随机生成。

所以这些字段不能用来判断真实传感器、供电或链路状态。`/get_ip` 返回的 STA/AP 地址和固件版本则来自后端，每 10 秒轮询一次。

## Wi-Fi 与 captive portal

二进制直接包含 SoftAP：

```text
SSID: RoboFish-ND1
Password: 12345678
AP address: 192.168.4.1（由页面和默认行为共同确认）
```

`/save_wifi?ssid=...&pass=...` 保存 STA 凭据后重启；`/get_ip` 的 JSON 结构为：

```json
{"sta_connected":1,"sta_ip":"...","sta_ssid":"...","ap_ip":"...","version":"4.0.0-partner"}
```

`/generate_204`、`/gen_204`、`/hotspot-detect.html`、`/ncsi.txt`、`/connecttest.txt` 和 `/fwlink` 等 captive portal 探测路径会把用户引导到驾驶页。

## Partner API

Partner Lab 使用：

- `GET /partner_script`：读取当前 JSON；
- `POST /partner_script`：校验并保存 JSON；
- `POST /partner_run`：启动脚本，失败时返回 `409 Conflict`；
- `POST /partner_stop`：安全停止；
- `GET /partner_status`：返回脚本名、步骤、循环和剩余时间，页面每秒轮询。

状态格式由二进制字符串直接恢复：

```json
{"version":"4.0.0-partner","running":true,"name":"...","step":1,"steps":6,"pass":1,"repeat":1,"remaining_ms":1000}
```

POST 脚本处理会检查 `content_len` 为 1～3500，分配 `len+1` 字节，循环接收请求体，处理接收超时，补 NUL 后调用解析/保存函数，并在所有路径释放缓冲区。

## Web OTA

上传接口为 `POST /ota_upload`，正文是原始 `application/octet-stream` 应用镜像，请求头为：

```text
X-OTA-Password: <password>
```

认证代码将请求头计算为 32 字节摘要，再与固件中的 SHA-256 值比较：

```text
1a9df07f78b20777c713269df01ff1fb0a6d036a6963231a52796c9160dc6b3a
```

没有发现明文 OTA 密码。本分析没有尝试猜解该摘要。摘要仍可能受到离线字典攻击，因此它不等同于硬件根信任；此外驾驶、Partner 和 Wi-Fi 保存接口未见同类认证。

确认的 OTA 流程是：停止执行器 → 获取下一 OTA 分区 → `esp_ota_begin` → 循环接收并写入 → `esp_ota_end` 验证 → 设置启动分区 → 返回成功 JSON → 延时约 1 秒重启。错误路径覆盖缺少/错误密码、无分区、begin/receive/write/validation/set boot partition 失败。
