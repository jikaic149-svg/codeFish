# RobotFish ESP-IDF：2.4 GHz Wi-Fi 控制

当前版本 **2.1.0-wifi**，运行于单块 XIAO ESP32-S3。运动指令、状态回传和 OTA 全部使用 Wi-Fi；不编译、不初始化 LoRa。工程以原始 `RobotFish(20260903-033000)(1).ino` 和 XIAO V4.0 原理图为硬件基线，运动核心迁自 `robofish_new/embedding/f4/firmware`。

## 与原工程及历史版本的区别

仓库中的 `RoboFishPartnerDriverSDK` 与本目录是两套独立工程；本次新增本目录，没有替换原工程。

| 项目 | 原 `RoboFishPartnerDriverSDK` | 本 `RobotFish-ESP-IDF` |
|---|---|---|
| 依据 | 原 `CameraTest.ino.bin` 的功能等价重建 | 用户指定的 ino／原理图与旧 F4 运动源码 |
| 主要用途 | 相机、原驾驶网页与 Partner 脚本 | 单 S3 四舵机、双独立电调协同运动 |
| 运动入口 | 原 `/gpio` 与 `/partner` 等入口 | 设备网页、PC 键盘／按钮，经 Wi-Fi 完整目标接口 |
| 相机／Partner／水泵／RGB | 保留原工程功能 | 本固件不启用；水泵 GPIO9 不输出 |
| 传感器及双板链路 | 以原工程配置为准 | 不搬入 F4 ADC、SPI、Bootloader 或传感器闭环 |
| 默认热点 | `RoboFish-ND1` | `RobotFishLOAR1`，名称沿用旧配置 |
| 页面 | `/gpio`、`/partner`、`/update` | `/` 操控与升级；`/motion_status` 查状态 |
| 交付版本 | 原文档为 `4.0.0-partner` | 当前 `2.1.0-wifi`，COM26 烧录启动已验证 |

本工程内部也经历过一次通信变更：

| 行为 | 历史 `2.0.0-idf` | 当前 `2.1.0-wifi` |
|---|---|---|
| 运动下发 | LoRa 文本命令 | 2.4 GHz Wi-Fi 完整三轴目标 |
| Wi-Fi 职责 | 遥测与 OTA | 运动、遥测与 OTA |
| LoRa 硬件 | 需要初始化 UART／接收服务 | 不编译、不初始化，不需要模块 |
| 网页／上位机操控 | 只读遥测，实物运动仍走 LoRa | WASD、R/F、组合按钮，按住续期、松开全停 |
| 3 秒无新目标 | 双桨中立、舵机保持 | 同样保留，另加会话、序号与过期排队请求校验 |
| 接线和运动算法 | 四舵机／双桨，旧 F4 运动核心 | 保持相同；未因换通信重新整定运动配方 |

原 ino 曾持续保持文本运动命令；迁移时按用户确认改为 3 秒续期。`robofish_new` 的旧 F4 工程仅提供运动核心与上位机来源，其 ADC、电压前馈、SPI 和 F4 启动／调试链没有一并迁入。当前发布的 OTA／USB 镜像均为 `2.1.0-wifi`，按本目录接线、项目名及升级流程使用。

## 操作

1. 上电，等待约 2 秒完成电调中立解锁。
2. 连接热点 `RobotFishLOAR1`，Wi-Fi 密码 `12345678`。
3. 打开 `http://192.168.4.1`，用户名 `fish`，密码 `fishota`。
4. 点击“启用 Wi-Fi 控制”，设备先全停，再选择螺旋桨、仿生或混合模式。
5. 按住屏幕按钮，或使用 **W/S 前后、A/D 左右、R/F 上浮下潜**，允许组合按键。
6. **松开停止，空格或“全停”使双桨停止、四舵机回中。** 切走窗口、隐藏或离开操控页时发送停止。

网页约每 200 ms、上位机约每 150 ms 发送新序号完整目标。超过 **3000 ms** 无新有效目标，双桨中立，舵机保持最后成功提交角度。错误或会话被接管后需要重新连接，不自动恢复旧动作。升降沿用原动作分配，可能伴随前进，不是定深控制。

## 电脑上位机

双击 [upper/start.cmd](upper/start.cmd)，或执行 `python -B upper/launch.py`，打开 `http://127.0.0.1:8766/`。设备设置输入 `192.168.4.1` 并检测，手动填写控制密码 `fishota`，点击“连接控制”，再返回操控页使用按钮或键盘。遥测免登录，运动需要认证会话。

服务修订为 `workstation-5-wifi`；已有旧服务运行时，先保存数据并退出旧服务，再启动本版。详情见 [upper/README.md](upper/README.md)。六路显示是驱动写入目标，不是实测位置或推力。没有相机、STM32、DAP 或真实电压、姿态、深度采样时，不生成对应测量值。

## 接线与控制

| 原理图信号 | XIAO | GPIO | 用途 |
|---|---|---:|---|
| DJ2 | D4 | 5 | 前尾 |
| DJ3 | D5 | 6 | 后尾 |
| DJ1 | D3 | 4 | 左胸鳍 |
| D2 | D2 | 3 | 右胸鳍 |
| PWM1 / P1 | D6 | 43 | 左电调／螺旋桨 |
| PWM3 / P3 | D9 | 8 | 右电调／螺旋桨 |
| TX / RX | D0 / D1 | 1 / 2 | LoRa 未使用，不初始化 |
| GPIO_IN1 | D10 | 9 | 水泵禁用，不配置、不驱动 |

PWM 为 50 Hz、14 位。舵机 500–2500 μs，ESC 为 1000/1500/2000 μs；左右安装方向 +1/−1 仅在 BSP 转换一次。四个纯计算模块保留旧 F4 频幅、混合分配、胸鳍辅助、限速与换向中立逻辑。未引入 F4 ADC、SPI、Bootloader、看门狗或视觉任务；没有电压采样，不执行电压前馈。

主循环 10 ms，唯一运动输出 20 ms。HTTP 经静态 8 槽命令／结果 FIFO 传目标，不直接写执行器。STOP／新会话至少成功提交一帧六路中立，才继续后续运动；超过 150 ms 未处理的 HTTP 命令拒绝晚执行。模式更新不启动、不续期。

## HTTP 接口

- `GET /get_ip`：身份、版本、运行槽、启动确认、Wi-Fi 控制能力。
- `GET /motion_status` 或 `/status`：目标、最后成功输出、提交结果、会话和租约。
- `GET /` 或 `/update`：操控和 OTA 页面，Basic 登录 `fish` / `fishota`。
- `POST /api/v1/control`：JSON 控制，Basic 或 `X-OTA-Password` 鉴权。
- `POST /ota_upload`：原生应用 BIN，鉴权方式同上。

先发送 `{"action":"acquire","session":"客户端随机ID"}`，其余操作使用同一会话和严格递增的 1–4294967295 序号：

```json
{"action":"mode","session":"客户端随机ID","sequence":1,"mode":3}
{"action":"target","session":"客户端随机ID","sequence":2,"axes":{"surge":30,"yaw":0,"heave":0}}
{"action":"stop","session":"客户端随机ID","sequence":3}
```

模式 1/2/3 为仿生／螺旋桨／混合。三轴必须完整给出，均为 −100～100 有限数值，全零等同全停。旧序号、错误会话、非法输入、队列忙和 OTA 锁定返回错误。HTTP 200 表示目标已接受；PWM 结果通过状态查询读取。

## 编译与安装

在已导出的 **ESP-IDF 5.3.2** 环境中进入本目录，执行 `idf.py build`。也可运行 `powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1`；该本机便捷脚本固定使用 `D:/ESP32/ESP-IDF-v5.3.2`、`D:/ESP32/.espressif`、`D:/CMAKE`、`D:/Ninja`，其他机器调整路径或直接使用 `idf.py build`。脚本不下载、不自动烧录。

生成 `build/robotfish.bin`。本次已验证并烧录的 [OTA 应用](artifacts/RobotFish-ESP32S3-OTA-20261017-111715.bin) 为 863712 字节，SHA256：

```text
d00d8ef53d54ba77e581a6462722bb19c9124cd5c656b70c1a3260723e0c0292
```

首次安装执行 `idf.py -p COM端口 flash`，或在 `artifacts/usb/` 用 esptool：

```powershell
python -m esptool --chip esp32s3 -p COM端口 write_flash '@flash_args'
```

地址：Bootloader 0x0、分区表 0x8000、OTA 数据 0xe000、应用 0x10000。不要将单个应用 BIN 当作整片镜像。OTA 可在设备网页或上位机选择本应用 BIN；写前确认六路中立，失败保持运动锁定，验证后切备用槽。上位机等待版本、换槽和健康确认同时成立才显示完成。新镜像未确认时，后续复位由 Bootloader 回滚。

## 验证与边界

已完成 ESP-IDF 构建、真实 BIN 检查和 COM26 四镜像写后校验。复位日志确认 `2.1.0-wifi`、Wi-Fi 服务就绪且无 LoRa UART 初始化。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/control/run.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/wifi_http/run.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/radio_board/run.ps1
python -B -m unittest discover -s upper/tests -v
```

检查包括 131 条状态、442 条真实控制循环、97 条 cJSON、BSP 和 22 项 PC。Edge 本机夹具验证实际内置页面与 PC 持续下潜超过 3 秒、松手／失焦停止、组合轴、409 锁定、390px 和模拟 OTA；已安装 Playwright／Edge 时可运行 `python -B upper/tools/verify_browser.py`。

运动核心沿用与旧 F4 的 1620 组／51840 周期对等结果；该对比测试需旧源码，可用 `-LegacyFirmware` 指定，正常构建不需旧目录。实机 Wi-Fi 运动、PWM 波形、负载／水池动作和 OTA 断电尚未验证。详情见 [validation.json](validation.json)。
