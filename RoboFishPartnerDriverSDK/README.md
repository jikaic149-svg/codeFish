# RoboFish Partner Driver SDK 1.0.0

完整源码的独立版本。它与当前完整机器鱼固件互不影响。

## 本版本包含

- 两个尾部舵机的单独角度控制
- 一个胸鳍舵机的上浮、回中和下潜角度控制
- 水泵继电器开关
- 电调油门及转向的基础信号输出
- 执行器归中、推进停止及紧急停止
- 推进指令2秒超时、水泵10秒超时
- 独立后台Web OTA服务
- AP及可选STA联网

## 本版本不包含

- 摄像头和图传
- `forward/backward/left/right`等完整动作
- CPG、正弦摆尾、相位差和转弯算法
- 仿生、螺旋桨、混合模式逻辑
- Demo动作、动作脚本和合作方实验页面
- 自动上浮、深度控制、遥测或自主控制
- 完整版固件中的推进标定与运动参数

可以看到如何向单个执行器发送基础指令，但需要自行实现随时间变化的关节协调、运动状态机和游动算法。

## 编译配置

- Board：`XIAO_ESP32S3`
- Flash Size：`8MB`
- Partition Scheme：`Default with spiffs (3MB APP/1.5MB SPIFFS)`
- PSRAM：可以关闭；本版本不使用摄像头或PSRAM
- CPU Frequency：`240MHz (WiFi)`
- 依赖库：`ESP32Servo`

## Web OTA

开机后固件始终创建：

- SSID：`RoboFish-Partner-SDK`
- 默认AP密码：`12345678`
- OTA地址：`http://192.168.4.1/update`
- 默认OTA用户：`admin`
- 默认OTA密码：`RoboFishSDK2026`

以上内容都在 `sdk_config.h` 中，合作方拿到源码后可以自行修改，因此它们不是用来向合作方保密的凭据。

OTA运行在独立FreeRTOS任务中，即使合作方的 `loop()` 运算较慢，通常仍能访问升级页面。上传开始时，驱动层会停止并锁定全部执行器，升级完成后自动重启。

OTA需要Arduino导出的应用固件 `.bin`，不能上传bootloader、partition或merged固件。

这里的“远程OTA”指通过机器鱼热点或同一局域网刷写，不包含直接暴露到公网的云端OTA。若合作机构不在现场，需要由现场电脑连接设备网络后操作，或另外建立VPN/远程桌面。

## 基础驱动API

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

`fish.update()`必须在 `loop()` 中持续调用，用于执行推进和水泵超时保护。非中立电调油门需要至少每2秒重新发送一次，否则会自动回到中立。

## 重要限制

本SDK源码和机器鱼硬件，他们可以删除软件限幅、修改引脚或重新实现任何算法。
