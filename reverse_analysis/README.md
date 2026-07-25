# CameraTest.ino.bin 静态逆向分析

本目录是 `../CameraTest.ino.bin` 的可复现静态逆向结果。原固件未被修改、重签名或烧录；分析结论来自 ESP32-S3 镜像结构、明文资源、指针关系和 Xtensa 指令交叉验证。

## 结论摘要

- 目标是未加密的 ESP32-S3 应用镜像，大小 1,111,664 字节，镜像校验和和 SHA-256 footer 均有效。
- 产品标识为 `RoboFish FPV Controller v4.0.0 Partner Edition`，包含摄像头图传、机器鱼运动控制、三种驱动模式、Demo、失联自动上浮、Partner 动作脚本和 Web OTA。
- 固件包含一个固定在 Core 1 的 `CPGTask`，栈 4096 字节、优先级 2，以 20 ms 周期运行控制更新，名义频率 50 Hz。
- 硬件执行器为 3 个舵机、水泵继电器和 2 路 ESC；摄像头引脚与 XIAO ESP32-S3 Sense 映射一致。
- Web 前端中的深度、速度、电量、温度和 RSSI 并非完整真实遥测：深度/速度固定为 0，电量固定 95%，温度和 RSSI 为前端随机数。
- Partner JSON 最多 3500 字节、32 步、重复 1～10 次、总时长最长 300000 ms，并保存于 NVS 命名空间 `partner_cfg`。
- OTA 要求请求头 `X-OTA-Password`；镜像只保存密码 SHA-256 值，没有发现明文 OTA 密码。

## 报告索引

- [01_image_layout.md](01_image_layout.md)：镜像、应用描述符、段布局和资源统计
- [02_web_http.md](02_web_http.md)：网页、HTTP 路由、控制参数和 OTA
- [03_hardware_runtime.md](03_hardware_runtime.md)：引脚、摄像头、执行器、任务与启动流程
- [04_control_state_machines.md](04_control_state_machines.md)：模式、CPG、Demo、失联保护和 Partner 引擎
- [05_recovered_pseudocode.cpp](05_recovered_pseudocode.cpp)：关键逻辑的人工恢复伪代码
- [06_limitations.md](06_limitations.md)：证据边界、未恢复内容和注意事项
- [metadata/http_routes.tsv](metadata/http_routes.tsv)：路由与处理函数速查表
- [metadata/key_functions.tsv](metadata/key_functions.tsv)：关键函数地址速查表

## 目录内容

```text
reverse_analysis/
├─ metadata/       镜像清单、路由和函数地址
├─ segments/       从应用镜像按加载段原样切出的二进制
├─ strings/        ASCII、UTF-8 和重点字符串表
├─ references/     32 位段内指针及重点字符串引用
├─ web_assets/     从 DROM 恢复的 HTML、manifest 和 service worker
├─ disassembly/    官方 Xtensa objdump 关键函数文本（本地 Rizin 数据库不上传）
└─ tools/          可重复执行的提取、指针和反汇编脚本
```

## 证据等级

- **直接确认**：镜像字段、字节、字符串、静态表或官方 objdump 指令可直接验证。
- **高可信推断**：由多个调用点、常量、状态变量和前端行为共同推出，但缺少原始符号名。
- **无法静态确认**：需要真实设备、NVS/分区转储、网络流量或原始源代码才能确认。

## 复现

在仓库根目录执行：

```powershell
python reverse_analysis/tools/analyze_firmware.py CameraTest.ino.bin reverse_analysis
python reverse_analysis/tools/build_pointer_map.py reverse_analysis
python reverse_analysis/tools/generate_disassembly.py reverse_analysis <xtensa-esp-elf-objdump路径>
```

反汇编使用与 Arduino ESP32 Core 3.3.7 同代的 Espressif Xtensa binutils 2.43.1。`disassembly/key_functions/` 按控制流入口拆分，避免把函数间填充字节误当成指令；`/control` 内仍存在内联跳转表，阅读时应结合注释和字符串引用。

原始固件 SHA-256：

```text
bc55582a37e8cc3abce227b09bf2bc9f66d6d93fda4782f00bc25daecdb79009
```
