# codeFish RoboFish Partner

面向 Seeed Studio XIAO ESP32-S3 的机器鱼固件交付包。项目同时提供可直接使用的 Partner Edition 预编译固件、供合作方二次开发的底层执行器驱动 SDK，以及预编译固件的可复现静态逆向分析资料。

> 重要说明：完整机器鱼功能只以预编译固件形式提供；可修改的 SDK 源码刻意不包含摄像头、游动算法、动作模式和运动参数。

## 项目内容

```text
codeFish/
├─ CameraTest.ino.bin                 RoboFish Partner Edition v4.0.0 完整固件
├─ PARTNER_USER_GUIDE.md              完整固件及动作脚本使用说明
├─ reverse_analysis/                  固件段、网页、反汇编、伪代码和中文逆向报告
└─ RoboFishPartnerDriverSDK/
   ├─ RoboFishPartnerDriverSDK.ino    Arduino 程序入口
   ├─ RoboFishDriver.h/.cpp           舵机、电调和水泵底层驱动
   ├─ OtaService.h/.cpp               Wi-Fi AP/STA 与 Web OTA 服务
   ├─ sdk_config.h                    引脚、限位、超时和网络配置
   ├─ README.md                       SDK 详细说明
   └─ build/                          ESP32-S3 编译及烧录产物
```

## 两套固件的区别

| 内容 | `CameraTest.ino.bin` | `RoboFishPartnerDriverSDK` |
| --- | --- | --- |
| 交付形式 | 预编译应用固件 | 完整 SDK 源码及编译产物 |
| 摄像头与视频流 | 包含 | 不包含 |
| 网页驾驶界面 | 包含 | 不包含 |
| 前进、后退、转向 | 包含 | 需要自行实现 |
| 仿生/螺旋桨/混合模式 | 包含 | 不包含 |
| 动作脚本 | 包含 | 不包含 |
| 舵机、电调、水泵基础控制 | 包含 | 包含 |
| Web OTA | 包含 | 包含 |

完整固件中的内嵌资源显示其版本为 `RoboFish FPV Controller v4.0.0 Partner Edition`，提供 `/gpio` 驾驶页面、`/partner` 合作方实验页面、`/stream` 视频流以及动作脚本接口。项目中没有这套完整固件的对应源代码。

## 完整固件逆向分析

[`reverse_analysis/`](reverse_analysis/) 保存了 `CameraTest.ino.bin` 的静态逆向结果，包括 ESP32-S3 镜像与段布局、9737 条 ASCII 字符串、4 个完整 Web 资源、HTTP 路由、摄像头和执行器引脚、FreeRTOS 任务、50 Hz CPG 控制、失联自动上浮、Demo/Partner 状态机、OTA 流程、关键 Xtensa 反汇编及人工恢复伪代码。

分析脚本可以从原始 `.bin` 重新生成段、字符串、指针和关键函数反汇编。所有结论按“二进制直接确认 / 高可信推断 / 无法静态确认”区分；恢复伪代码用于解释行为，不是原始源码，也不能直接替代固件编译。

原始完整固件 SHA-256：

```text
bc55582a37e8cc3abce227b09bf2bc9f66d6d93fda4782f00bc25daecdb79009
```

## Partner Edition 功能

设备默认创建热点 `RoboFish-ND1`。连接后可访问：

- `http://192.168.4.1/gpio`：实时驾驶和视频界面
- `http://192.168.4.1/partner`：Partner Lab 动作脚本界面

动作脚本采用 JSON 格式，支持：

- `mode_bionic`、`mode_prop`、`mode_hybrid`
- `forward`、`backward`、`left`、`right`、`stop`
- `pitch`、`pump_on`、`pump_off`

固件限制单个脚本最多 32 步、重复 1～10 次，总执行时间最长 5 分钟。详细格式见 [PARTNER_USER_GUIDE.md](PARTNER_USER_GUIDE.md)。

## 底层驱动 SDK

SDK 提供以下 API：

```cpp
fish.setTailServo1(angle);
fish.setTailServo2(angle);
fish.setTailServos(angle1, angle2);
fish.setPitchServo(angle);
fish.setPump(enabled);
fish.setEscThrottle(command);
fish.setEscSteering(command);
fish.centerTail();
fish.stopPropulsion();
fish.emergencyStop();
fish.update();
```

`fish.update()` 必须在 `loop()` 中持续调用，以执行推进和水泵超时保护。SDK 默认不会让机器鱼游动，开发者需要自行实现随时间变化的关节协调、推进控制和安全状态机。

### 硬件映射

| 执行器 | GPIO | 默认/限制 |
| --- | ---: | --- |
| 尾部舵机 1 | 5 | 中心 86°，20～152° |
| 尾部舵机 2 | 6 | 中心 86°，20～152° |
| 胸鳍俯仰舵机 | 4 | 中心 90°，45～135° |
| 水泵继电器 | 9 | 低电平有效，最长开启 10 秒 |
| 电调油门 | 43 | 中立 90，范围 35～150 |
| 电调转向 | 44 | 中心 88，范围 40～136 |

非中立推进命令需要在 2 秒内刷新，否则驱动会自动回到中立。OTA 上传开始时，驱动会紧急停止并锁定全部执行器，直到设备重启。

## SDK Web OTA

SDK 启动后会创建热点 `RoboFish-Partner-SDK`，升级页面为：

```text
http://192.168.4.1/update
```

OTA Web 服务运行在独立 FreeRTOS 任务中，并支持可选的 STA 路由器联网。上传时应选择 Arduino 导出的应用固件 `RoboFishPartnerDriverSDK.ino.bin`，不能上传 bootloader、partition 或 merged 固件。

热点密码及 OTA 账号保存在 `sdk_config.h` 中。它们是可修改的本地访问配置，不应被视为安全密钥；部署前请按实际网络环境修改。

## 编译环境

- Board：`XIAO_ESP32S3`
- Arduino ESP32 Core：`3.3.7`
- CPU：240 MHz
- Flash：8 MB
- Partition：双 OTA 应用分区 + SPIFFS
- PSRAM：关闭
- 优化：`-Os`
- 外部依赖：`ESP32Servo`

`build/esp32.esp32.XIAO_ESP32S3` 已包含应用固件、bootloader、分区表、合并固件、ELF、MAP、烧录参数和 `sdkconfig`。

## SDK 不包含的内容

- 摄像头和图传
- CPG、正弦摆尾、相位差和转弯算法
- `forward/backward/left/right` 等完整动作逻辑
- 仿生、螺旋桨和混合模式状态机
- Demo、动作脚本及 Partner Lab 页面
- 自动上浮、深度控制、遥测和自主控制
- 完整版固件中的推进标定与专有运动参数

更多 SDK 使用说明见 [RoboFishPartnerDriverSDK/README.md](RoboFishPartnerDriverSDK/README.md)。
