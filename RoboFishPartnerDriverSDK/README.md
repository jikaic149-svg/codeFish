# RoboFish Partner 固件（纯 ESP-IDF）

这是依据 `CameraTest.ino.bin` 静态逆向证据重建的可编译固件工程。目标是恢复外部功能和硬件行为，不声称还原原作者逐行源码，也不追求二进制一致。

## 已恢复功能

- XIAO ESP32-S3 Sense 摄像头、抓拍、BMP 和端口 81 MJPEG 图传；
- 原驾驶页 `/gpio`、Partner Lab `/partner`、PWA manifest 和 service worker；
- BIONIC、PROP、HYBRID 三种推进模式，网页前后左右、停止、俯仰、水泵和 Demo 控制；
- Partner JSON 脚本校验、NVS 保存和非阻塞执行；
- `drive_cfg` 的 `esc_rev`、`steer_rev`、`tail_rev` 方向配置，以及隐藏 `tail_reverse` 控制协议；
- 15 秒启动豁免、连续失联 3 秒后的停止/关闭水泵/45°上浮保护；
- GPIO7 的 8 像素 GRB/800 kHz RGB 状态灯；
- 鱼 AP、电脑 STA 的本地控制与认证 Web OTA；
- 双 OTA 分区、启动确认和 rollback。

原固件默认网络：

```text
SSID: RoboFish-ND1
AP password: 12345678
AP address: 192.168.4.1
```

## 工程与依赖

- ESP-IDF 5.3.2，target `esp32s3`，C++17；
- 8 MB Flash、QIO 80 MHz、PSRAM 关闭；
- `main/idf_component.yml` 声明官方 `espressif/esp32-camera`；
- `dependencies.lock` 当前锁定 `esp32-camera 2.1.7` 和 `esp_jpeg 1.3.1`；
- `app0`、`app1` 各 `0x330000` 字节。

标准构建：

```powershell
idf.py -B build\idf build
```

应用镜像为 `build/idf/RoboFishPartnerDriverSDK.bin`。VS Code 构建任务会复制为：

```text
build/RoboFishPartnerDriverSDK-OTA-yyyyMMdd-HHmmss.bin
```

只把这个应用镜像用于 Web OTA；不要把 bootloader 或分区表上传到 `/update`。

## 页面与接口

| 方法 | 路由 | 用途 |
| --- | --- | --- |
| GET | `/gpio` | 驾驶与图传页面 |
| GET | `/partner` | Partner 脚本页面 |
| GET | `/status` | 相机状态 |
| GET | `/control?var=...&val=...` | 相机和机器鱼共用控制协议 |
| GET | `/capture`、`/bmp` | JPEG/BMP 抓拍 |
| GET | `:81/stream` | MJPEG 图传 |
| GET | `/get_ip` | AP/STA 地址和版本 |
| GET | `/save_wifi` | 保存可选 STA 凭据并重启 |
| GET/POST | `/partner_*` | Partner 脚本读写、运行、停止和状态 |
| GET/POST | `/update` | Basic Auth Web OTA |
| POST | `/ota_upload` | 原驾驶页兼容的原始 bin OTA |

原固件 OTA 密码无法从 SHA-256 摘要恢复；本重建版使用源码配置中的明文凭据，默认用户名 `admin`、密码 `RoboFishSDK2026`。

## VS Code 无线 OTA

用 VS Code 打开上级 `codeFish` 目录。任务和“运行和调试”面板均提供：

- 检查 ESP-IDF 5.3.2 环境；
- 生成时间命名 OTA 文件；
- 让 Windows 电脑作为 STA 连接机器鱼 AP；
- 隐藏输入 AP 密码与 OTA 密码，输入 `UPLOAD` 后烧录并显示设备反馈；
- 编译并无线 OTA 的组合入口。

拓扑固定为：

```text
机器鱼（AP） ← 电脑（STA） → http://192.168.4.1
```

## 硬件映射

| 功能 | GPIO / 参数 |
| --- | --- |
| 尾舵 1 / 尾舵 2 | GPIO5 / GPIO6，中心 86° |
| 胸鳍俯仰 | GPIO4，上浮 45°、复位 90°、下潜 135° |
| 水泵 | GPIO9，低有效 |
| ESC 油门 / 转向 | GPIO43 / GPIO44，停止 90° |
| RGB 灯条 | GPIO7，8 像素，GRB，800 kHz，亮度 40 |
| 摄像头 XCLK / SCCB | GPIO10、GPIO40、GPIO39，12 MHz |
| 摄像头 D0..D7 | 15、17、18、16、14、12、11、48 |
| VSYNC / HREF / PCLK | 38 / 47 / 13 |
| 补光灯 | GPIO21 |

RGB 状态灯：启动中为蓝色，启动失败为红色；启动成功后保持 IDLE，未连接为蓝色、已连接为青色。首次选择推进模式后，BIONIC 为绿色、PROP 为红色、HYBRID 为紫色；失联保护为橙色。IDLE 下前进、后退和左右转命令会被拒绝。

## 验证边界

编译成功只证明源码、组件和链接闭环。尚需在实机上确认摄像头方向与持续图传、DRAM 余量、RGB 时序、舵机与 ESC 方向、水泵极性、AP/STA、网页控制、失联保护和两条 OTA 路径。首次测试动力部分时应卸载螺旋桨或使机构离水且无负载。
