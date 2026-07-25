# 硬件与运行时结构

## 执行器映射

下表由 `hardware_init` 的立即数、Servo attach 调用和中文启动日志共同直接确认：

| 执行器 | XIAO 标号 | GPIO | PWM/电气行为 |
| --- | --- | ---: | --- |
| 尾部舵机 1 | D4 | 5 | Servo，500～2500 µs |
| 尾部舵机 2 | D5 | 6 | Servo，500～2500 µs |
| 胸鳍俯仰舵机 | D3 | 4 | Servo，500～2500 µs |
| 水泵继电器 | D0 | 9 | 低电平有效 |
| ESC 油门 | D6 | 43 | Servo PWM，900～2100 µs |
| ESC 转向 | D7 | 44 | Servo PWM，900～2100 µs |

初始化时尾舵机目标和胸鳍输出均写为 86°，ESC 两路写为 90，水泵关闭。普通停止会把尾部目标恢复为 86°、ESC 目标恢复为 90；驾驶页面的俯仰“复位”命令则明确发送 90°。这些值属于完整预编译固件，不应与仓库中独立 Driver SDK 的默认参数混为一谈。

舵机和 ESC 的公开写入路径会把角度限制在 0～180。尾部和 ESC 另外存在渐进逼近目标的更新函数，用于减少 50 Hz 控制循环中的瞬时跳变。

## 摄像头配置

`setup` 在栈上构造 `camera_config_t`，立即数对应 XIAO ESP32-S3 Sense 摄像头总线：

| 信号 | GPIO | 证据 |
| --- | ---: | --- |
| PWDN | -1 | config 字段 |
| RESET | -1 | config 字段 |
| XCLK | 10 | config 字段 |
| SCCB SDA | 40 | config 字段 |
| SCCB SCL | 39 | config 字段 |
| D0 | 15 | config 字段 |
| D1 | 17 | config 字段 |
| D2 | 18 | config 字段 |
| D3 | 16 | config 字段 |
| D4 | 14 | config 字段 |
| D5 | 12 | config 字段 |
| D6 | 11 | config 字段 |
| D7 | 48 | config 字段 |
| VSYNC | 38 | config 字段 |
| HREF | 47 | config 字段 |
| PCLK | 13 | config 字段 |
| 补光 LED | 21 | 初始化调用 |

XCLK 频率常量是 `0x00b71b00`，即 12,000,000 Hz；像素格式配置为 JPEG。帧尺寸、JPEG 质量、帧缓冲数量和抓取模式会根据可用外部内存分支调整。由于具体 `framesize_t` 枚举来自编译时的 esp32-camera 版本，报告保留原始枚举值，不把它强行翻译成一个可能错误的分辨率名称。

## 启动顺序

`setup()` 位于 `0x4200447c`，恢复出的顺序是：

1. 串口以 115200 baud 启动，等待 2000 ms；
2. 输出版本、芯片和 XIAO 引脚检查信息；
3. 构造摄像头配置并调用 `esp_camera_init`，失败时打印具体错误提示；
4. 初始化补光灯、相机 HTTP 服务、Wi-Fi AP/STA 和应用路由；
5. 读取持久化配置，包括尾部方向标志和 Partner 脚本；
6. 初始化舵机、ESC、水泵与状态灯，使输出进入安全中位；
7. 创建 `CPGTask`；
8. 保存启动时间并打印失联保护参数。

相机初始化失败会提前返回，不会继续创建运动控制任务。

## FreeRTOS 与 Arduino 调度

任务创建参数可直接恢复为：

```cpp
xTaskCreatePinnedToCore(
    CPGTask,
    "CPGTask",
    4096,
    nullptr,
    2,
    nullptr,
    1
);
```

任务入口 `0x42003a8c` 的循环非常短：

```cpp
void CPGTask(void *) {
    Serial.println("CPG Task Started!");
    for (;;) {
        controlTick();
        delay(20);
    }
}
```

因此运动输出名义更新频率为 50 Hz。这里使用动态任务创建，栈大小参数为 4096 字节。

Arduino `loop()` 位于 `0x42004864`：

```cpp
void loop() {
    failsafeWatchdog();
    demoTick();
    partnerTick();
    delay(10);
}
```

失联保护、Demo 和 Partner 脚本都由 10 ms 主循环非阻塞推进；CPG/执行器插值则由独立的 20 ms 任务处理。这种分工保证脚本持续数秒时仍能继续检查链路并刷新执行器。

## 网络运行时

固件同时支持 SoftAP 和保存后的 STA。SoftAP 标识为 `RoboFish-ND1`；STA 凭据保存在 NVS。失联判断不是简单检查 STA：只要 SoftAP 仍有客户端，或 STA 处于连接状态，链路就被视为存在。具体时序见 [04_control_state_machines.md](04_control_state_machines.md)。
