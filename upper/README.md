# RobotFish 工作站

本目录复用只读来源 `C:/Users/rlyoulo/Desktop/robofish_new/upper` 的顶层工作站源码，保留原作者信息。没有复制原工程的数据、模型 DLL、画像素材、EmberProbe 或固件工具。原目录未修改。

双击 `start.cmd`，或在本目录运行 `python -B launch.py`，打开 `http://127.0.0.1:8766/`。新服务身份为 `robotfish-local`，独立于旧工作站的 8765；不会终止其他实例。可用 `ROBOFISH_HOST_PORT` 同时调整启动器与服务端口。

本版服务修订为 `workstation-5-wifi`。已有旧版上位机服务运行时，启动器会提示先保存数据并结束旧服务；重新启动后刷新浏览器。不能仅刷新旧服务上的网页来更新后台控制接口。

在“设备设置”输入 RobotFish 的 IPv4 地址并点击“检测”。单 ESP32-S3 身份为 `firmware=robotfish`、`hardware=robotfish-esp32s3`、`project_name=robotfish`。遥测免登录，六路值来自 `/motion_status` 的 `last_written`，表示驱动写入目标，不是位置反馈。无相机、STM32、DAP、SPI 管理链及电池采样时明确显示未提供；运动指令、遥测与 OTA 均经 2.4G Wi-Fi，LoRa 已禁用。

输入设备控制密码（与 OTA 密码相同，需手动输入）并点击“连接控制”。连接会创建新的控制会话并先全停；返回“操控”页后选择仿生、双螺旋桨或混合模式。按住方向按钮或 WASD 前后转向，R/F 浮沉，可同时按；按住每约150毫秒发送一个新序号目标，松开归零，空格或“全停”立即停止并回中。模式切换本身不启动或续期运动。

失焦、切页或隐藏窗口会清除按键并发送零目标。网络错误或控制权被其他终端接管后，需要重新点击“连接控制”，不会自动恢复旧目标。超过3秒没有新的有效目标时，固件停止双桨并保持舵机。关闭上位机或网络断开仍由此超时规则保护。控制密码只保存在本地服务内存；勾选保存时才使用现有 Windows DPAPI 密钥存储。

控制接口为 `POST /api/v1/control`，由本地服务通过 `X-OTA-Password` 发送用户输入密码。`acquire` 使用随机 `session`；`target`、`mode`、`stop` 使用同一会话和严格递增的 `sequence`。上位机不会重试失败的运动指令，也不会向未声明 `wifi_control_allowed=true`、`control_protocol=1` 的旧固件开放运动控制。

“开发”页为识别到的 RobotFish 显示 Wi-Fi OTA。选择本工程的 `build/robotfish.bin` 并手动输入现有 OTA 密码，不预填密码。PC 检查 ESP32-S3、`robotfish` 项目名、0x330000 字节上限、镜像段边界、校验和与附加 SHA256；这是完整性检查，BIN 没有签名。设备接收后，必须同时报告目标版本、应用槽切换与 `ota_confirmed=true`、`runtime_ready=true` 才显示完成。同版本重刷也要求换槽。首次 USB 安装使用工程的 `idf.py flash`。

旧 `rlyoulink` 的 Cookie 登录和 `.amphipkg` 路径保留。独立复制的本目录不包含其签名工具；需要时设置 `ROBOFISH_FLASH_HOME` 指向旧工程 `embedding/s3/tools`。旧 STM32 离线模型与 EmberProbe 功能未随本交付提供。

离线检查：`python -B -m unittest discover -s tests -v`。测试使用本机临时 HTTP 夹具，不接触硬件。实际 2.4G、执行器和 OTA 重启仍需上板验收。

如本机已安装 Playwright 与 Edge，可运行 `python -B tools/verify_browser.py` 检查页面六路数据、BIN 选择与模拟 OTA；脚本仅访问本机临时端口，不占用 8765/8766。
