# RoboFish 重建固件架构

```text
app_main
  → RoboFishRuntime
    → NVS
    → CameraService
    → RoboFishDriver（BSP / device owners）
    → MotionController（20 ms、Core 1、优先级 2、静态 4096 B 栈）
    → RgbStatusStrip
    → Partner ScriptEngine
    → OtaService（APSTA、端口 80 HTTP owner）
    → WebAssets + ProductWebService + Camera routes
    → Camera stream server（端口 81）
```

## 职责边界

- `src/bsp` 持有 LEDC、RMT 和 MCU 外设资源；
- `src/device` 持有尾舵、俯仰、ESC 和水泵，分别使用静态互斥锁；
- `MotionController` 持有模式、Demo、链路保护和 50 Hz 输出整形；
- `CameraService` 是 `/status`、`/control` 及图像路由 owner，未知控制变量回调给产品层；
- `ProductWebService` 只处理运动控制、Partner API、根重定向和 captive portal；
- `OtaService` 是 Wi-Fi、NVS 初始化和端口 80 HTTP server owner；其他模块只向同一 server 注册路由；
- `ScriptEngine` 使用固定 JSON 缓冲、静态锁和 NVS，不创建独立任务；
- `main` 只编排启动、10 ms tick 和 OTA 镜像确认。

## 运行时约束

- 所有设备锁和业务等待均为有限超时，不使用 `portMAX_DELAY`；
- Motion 任务每 20 ms 计算一次有界输出，Demo、Partner 和 failsafe 由 10 ms 主循环非阻塞推进；
- 有效人工控制先停止 Partner，并可清除 failsafe；未知 `/control` 变量不会停止 Partner；
- 启动时从 NVS `drive_cfg` 读取 ESC、转向和尾部方向标志；只有原镜像确认的 `tail_reverse` 暴露在线修改；
- OTA 开始前停止 Partner、Demo、推进和水泵，然后锁定设备命令；
- 新 OTA 只有在核心运行时成功后才标记有效，否则保留 pending 供 bootloader rollback；
- 摄像头初始化失败时不武装运动任务，但端口 80 仍用于故障观察与恢复 OTA。

## 构建边界

顶层固定 ESP32-S3，官方摄像头依赖由 Component Manager 解析。QIO 配置生成的 ESP 镜像头和烧录参数显示为 DIO 是 ESP-IDF 的预期行为：ROM/一级加载阶段使用 DIO，二级 bootloader 再切换 QIO。

还原依据、已确认常量和无法静态确认的边界见 `../reverse_analysis/`。
