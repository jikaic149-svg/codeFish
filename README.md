# RobotFish ESP-IDF

完整配置、接线、烧录、测试和操作经过：[LoRa 与 IMU 配置及操作记录](docs/history/2026-10-22/lora-imu-setup-and-test.md)。

当前源码版本 **2.7.7-steering-priority**，运行于单块 XIAO ESP32-S3。手动命令以 **LoRa 为主、认证 Wi-Fi 为备用**，画线自动循迹使用 **Wi-Fi**，命令结果、执行器状态和 JY61P 数据通过 **Wi-Fi HTTP 回传**；OTA 继续使用 Wi-Fi。S3 本地 yaw PID 支持指定航向、静止和前进保持，默认 Kp/Ki/Kd 为 **2.5 / 0.02 / 0.1**，死区 **±1°**，角度环转向分配增益 **1.5**，纠偏范围 **±100%**，保留用户最终确认正确的原极性 **+1**。闭环上电默认关闭，pitch／roll 仅预览。完整参数与行为见[姿态控制说明](docs/guides/attitude-control.md)。

当前编译产物：[robotfish.bin](build/firmware/robotfish.bin)，版本 `2.7.7-steering-priority`，961600 字节，SHA256：`20f9f9b30a8958f76242afb09858262b198db965f78295bbe6da05498c612e59`。2026-11-20 目录整理后当前固件集中在 `build/firmware/`，原构建缓存留在 `build/archive/firmware-cache/`，不直接复用旧绝对路径；未覆盖现有镜像。`2.6.4-yaw-sign` 反向对照版已按用户确认撤回。最新进度与待办先读 [process.md](process.md)，详细经过按日期保存在 [docs/history/](docs/history/README.md)。

上位机入口为 `upper/start.cmd`；视觉默认加载 [黑色机器鱼 best.pt](vision/models/fish_body_yolo/best.pt)，检测与数据来源见[模型说明](vision/models/fish_body_yolo/README.md)。仓库保留源码、测试、模型和当前应用固件，本地录像、日志、训练中间文件、旧固件／硬件备份及个人运行数据不随本次提交上传。

当前上位机为 `workstation-37-layout`（目录迁移修订，控制算法不变）：画线循迹默认0.08m/s，前向量同时给双桨和摆尾（1∶1指令比例，不代表推力份额）；IMU角度PID只叠加尾偏置，差动保持0，收油时周期摆尾归零。保留2.7.6的1.5倍目标摆幅；2.7.7在关节行程不足时优先保留转向偏置，剩余行程用于摆动，避免大摆幅挤掉转向量。推进20%、满转向时，前节名义偏置24°、半摆幅8°，仍受原速率约束。原角度PID、死区和尾偏置增益保持。速度前馈使用10+10／15+15／20+20三段混合推进参考，并保留速度反馈；该数据来自加大摆幅之前，新版仍需水测。须更新2.7.7固件并重开 `upper/start.cmd`。动力测试页顶部直接提供摆尾联合推进和静态尾角对照入口，内含实时画面、选鱼与鱼头自动置零。完整操作和边界见[循迹说明](docs/guides/vision-control.md)与[动力测试说明](docs/guides/dynamics-calibration.md)。

早期 USB 实机记录的固件为 **2.3.2-lora-imu**，当时按用户要求仅使用 Type-C 串口做实机测试，不更换电脑 Wi-Fi。JY61P 已按实际 **115200/8N1** 接入，三轴有效输出及手动转动响应已确认；LoRa 已在软件内交换 TX/RX 分配以适配现有接线，定点接收 10/10 通过。原 validation 日志的有效结论已摘入 [2026-11-20 历史记录](docs/history/2026-11-20/process.md)，随后原目录移入回收站；旧诊断镜像不再作为当前交付入口。这些结果不代表新版姿态控制已经上板验收。

USB 观察仅在 `ROBOTFISH_USB_DIAGNOSTICS=ON` 时编译，Type-C COM26 每 100ms 输出一行 `USBSTAT {...}`，包含 roll/pitch/yaw、数据年龄、CRC/接收错误、LoRa 接收计数和最后命令。此构建选项默认 OFF，当前测试缓存已启用；关闭时以 `-DROBOTFISH_USB_DIAGNOSTICS=OFF` 重新构建。

## 当前串口与回传

已通过 easyeda-agent 回读工程“比赛机器鱼V4.0 XIAOesp32”的 `PCB1_1` 及其原理图，接口映射如下：

| 用途 | ESP32 外设 | ESP32 TX | ESP32 RX | 参数 |
|---|---|---|---|---|
| PCB 的 GND／5V／TX／RX 四针口接 LoRa | UART1 | GPIO2 / D1 / PCB TX → LoRa RXD | GPIO1 / D0 / PCB RX ← LoRa TXD | 115200，8N1 |
| RGB／GPIO_IN1 接 JY61P | UART2 | GPIO7 / D8 / RGB → JY61P RX | GPIO9 / D10 / GPIO_IN1 ← JY61P TX | 115200，8N1 |

当前软件按现有接线配置：板 TX/GPIO2 为 ESP32 发送、板 RX/GPIO1 为 ESP32 接收。因此 LoRa TXD 接板 RX，LoRa RXD 接板 TX，不用改线。LoRa 的 VCC、GND、M0 高和 M1 低继续沿用已验证的六线固定方案。JY61P 共地，UART 使用 3.3V 电平；参考 [官方规格书](https://wit-motion.yuque.com/wumwnr/docs/agok0k)，当前模块按用户确认的 115200 bps 接收；其他模块的实际速率须同步修改 `main/config/BoardConfig.h`。

**GPIO9 的现有硬件支路：** 原理图 U16 为 EL817S1，GPIO_IN1 接其 LED 阳极，LED 阴极经 R1（1 kΩ）接 GND。若 U16／R1 已焊装，接陀螺仪前先隔离该支路；RGB 接口原外设也须停用。本轮只读核对 PCB，没有修改硬件设计。GPIO9 不再允许启用水泵输出。

连接原热点后，设备网页 `http://192.168.4.1/` 显示命令结果、输出和陀螺仪数据；登录与 OTA 密码沿用原设置。也可轮询 `GET /lora_status` 或 `/motion_status`。返回最新快照，包含 `command_result`、`lora.last_ack` 和 `imu`；这不是逐包可靠推送服务。LoRa 端不发送 ACK。历史 2.3.2 的 Wi-Fi 运动入口返回 `403 / lora_control_only`；当前源码支持认证 Wi-Fi 备用控制，LoRa 健康时保留主控优先级，全停仍可受理。

当前鱼端实际可达的定点地址是 **0x0102**、信道 0x20；电脑发送端需先写入字节 `01 02 20`，再跟应用层 ASCII 命令。应用载荷中的逻辑鱼节点仍为 1，不能把这两个地址字段混用。

JY61P 接收器校验 WIT 的 11 字节数据帧，提供加速度（g）、角速度（deg/s）和欧拉角（deg），各组独立显示有效标志和数据年龄。原始遥测保留传感器坐标；新增反馈只使用明确配置后的鱼体坐标，不自动写模块配置或校准。帧格式参考 [WIT 官方通信协议](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol)。

2.3.0 初版构建与验证仅作历史证据，见 [历史索引](docs/history/README.md)。该历史版本使用 3 秒指令租约；当前版本按用户新要求改为保持最后指令，普通缓启停、急停、输出故障分级和 OTA 中立屏障继续沿用。

当前 `main/config/AppConfig.h` 的 `kLeaseMs=0` 表示运动目标持续有效，直到新目标、全停或原有输出故障／OTA 等明确停止事件；长时间没有新指令不会自行停桨或清除鱼鳍状态。遥测以 `motion_timeout_ms=0` 声明这一策略。LoRa 的 3000 ms 健康判断只用于主备控制权选择，不取消已接受的目标；Wi-Fi 接管已有 LoRa 动作时仍先经过成功中立输出，再接收新的目标。上位机仍采用按住操作、松开发送零目标。

## 电池检测现状（2026-10-24 EDA 回读）

用户确认使用 **2S 锂电，标称 7.4 V／1500 mAh**，要求保留现有板子和接线。已通过 easyeda-agent 只读连接“比赛机器鱼V4.0 XIAOesp32”的 PCB1_1 及其关联原理图，回读30个实际器件与连接：VBAT 接电调／舵机电源和 78L05、LM2941 输入，没有 VBAT 分压到 MCU 的采样支路；R24／R25（PCB 标注3.9K／1K）连接的是 LM2941 的6V反馈，不是电池测量。现有电调接口是 PWM＋GND，当前协议没有电压回传。

普通 XIAO ESP32-S3 的电池端也未连接到 ADC，见 [Seeed 官方电池说明](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/#battery-usage)。在“不改硬件和接线”约束下，无法获得真实电池电压，因此不能实现真实剩余电量估算或实时电压前馈；1500 mAh 是标称容量，不代表当前剩余量。没有添加虚构电压、按时间扣电量或固定电压冒充前馈，也未修改原理图／PCB。上位机继续明确显示电池未测，已有 IMU、输出和通信数据不受影响。

## 姿态辅助代码（默认关闭，待实测）

姿态调度复用鱼端现有20ms输出周期，不增加任务或网络往返；yaw PID每100ms更新，三轴使用同一PID实现和独立状态，实际只接yaw输出，范围±100%。角度和误差统一到 `[-180°,180°)`，跨零及±180°走短路径，恰好半圈固定取−180°；D项排除目标阶跃。所有输出继续经过原有运动分配、机械限位与变化速率限制。

人工开启 yaw 后，静止和前进时均允许航向修正；普通松键归零保留目标，倒车和手动转向优先，松开转向后记录新航向。全停关闭闭环并清除旧目标；控制权切换、传感器失效或重新开启也会清除旧目标和积分。角度和角速度必须同时有效且不超过 200 ms；失效撤掉自动修正，保留人工目标。

参数集中在 `main/config/AttitudeConfig.h`。用户实测为左转 yaw 增大、抬头 pitch 减小、右翻 roll 增大；后台日志中的静置 roll 接近 ±180°，据此采用鱼体前／右／下映射 `{+1,+2,+3}`。例如原始 `roll=-175.748°、pitch=7.020°` 转成鱼体 `roll=4.252°、pitch=-7.020°`，不会把它当作 176° 的横滚误差。日志不能证明鱼体当时精确水平，零偏仍为 0，不自动将该姿态归零；安装方向和水平偏差需实机复核。俯仰、航向命令方向为 +1；横滚执行器方向及鳍片差动量仍为 0，横滚反馈不输出。PID 参数为调参起点，航向目标是本次开启后的相对方向，不代表绝对北向。

LoRa v2 开关命令为 23／24，PID 参数命令为25；Wi-Fi 动作为 `attitude_on`／`attitude_off`／`yaw_pid`。单独调参不启用闭环；开启会授权静止纠偏。设备身份声明能力与标定状态，`/motion_status` 的 `attitude_control` 报告各轴状态、鱼体姿态、修正量、航向目标和数据年龄，并回读 `yaw_command_sign=+1`。上位机在安装映射未确认、旧固件或旧后台时禁用开启入口；上电默认关闭。

根目录旧固件 `tests/` 已按用户要求移除，不再提供其回归脚本入口；保留的上位机／视觉检查见下文与 `upper/README.md`。主机软件验证不等同于真实运动验收。

## 代码目录与入口

| 路径 | 用途 |
| --- | --- |
| `main/` | ESP32-S3 固件：BSP、Device、Algorithm、Mode、通信服务和集中配置 |
| `upper/src/`、`upper/web/` | 上位机后台、网页；启动器仍在 `upper/` |
| `vision/src/`、`vision/configs/` | 视觉观测、引导、录像源码与配置 |
| `vision/models/`、`vision/experiments/` | 部署模型与本机训练／评估数据分离 |
| `upper/tests/`、`upper/tools/` | 上位机及视觉回归测试、开发验证工具 |
| `docs/` | 当前说明、按日期归档的开发记录和正式报告 |
| `process.md`、`docs/history/` | 精简当前进度、按日期归档的详细过程 |
| `build/firmware/`、`build/releases/` | 当前固件构建、未来编译导出的应用副本 |
| `build/records/` | 实测录像、CSV 和标定结果，不是可随意删除的缓存 |
| `build/checks/`、`build/analysis/` | 主机检查输出、离线分析／仿真 |
| `build/archive/`、`build/workstation/` | 旧缓存／发布材料、本机工作站产物 |

生产视觉入口为 `vision/src/observer_runtime.py`，由上位机启动。录像测试统一到 `upper/tests/test_video_recording.py`。根目录 `tests/`、`materials/`、`build-host/`、`artifacts/`、`validation/` 已按授权移入 Windows 回收站；不影响保留的 `upper/tests/`。

## 编译、升级与离线验证

在 ESP-IDF **5.3.2** 环境中，从工程根目录执行 `idf.py -B build/firmware build`。中文路径下需为当前进程启用 Python UTF-8（PowerShell：`$env:PYTHONUTF8='1'`）；`build.ps1` 已在构建子进程中设置，不改变系统配置。

也可使用 `build.ps1`；当前脚本仍引用本机 `D:/ESP32`、`D:/CMAKE` 和 `D:/Ninja`，换电脑前需要适配。脚本成功编译后将本次应用 BIN 导出至 `build/releases/`，不重建根 `artifacts/`。编译不自动烧录。首次安装使用 `idf.py -B build/firmware -p COM端口 flash`，OTA 使用匹配的应用 BIN；单个应用 BIN 不是完整 Flash 镜像。

现有离线检查入口：

```powershell
python -B -m unittest discover -s upper/tests -v
python -B upper/tests/test_video_recording.py
python -B tools/test_portable_actions.py
node upper/tests/test_dynamics_steps.js
```

视觉检查需使用具有 OpenCV、NumPy 等依赖的 Python。现有测试有各自工具链及版本前提；入口列表不表示本次全部重跑通过。软件回归、驱动指令和历史通信试验不能替代当前配方的水中运动验收。

## 文档与历史

- [完整文档与时间索引](docs/README.md)
- [上位机操作说明](upper/README.md)
- [当前项目进度](process.md)与[按日期归档的详细历史](docs/history/README.md)
- [2026-10-22 LoRa／IMU 实机记录](docs/history/2026-10-22/lora-imu-setup-and-test.md)
- [2026-10-25 运动性能报告](docs/reports/2026-10-25-motion-performance/机器鱼运动性能与循迹问题评估报告.html)
- [整理前完整工程说明](docs/history/2026-10-26/project-readme.md)：保留旧版协议、接线、升级和验证描述，历史说明按各自版本理解。

Windows完整开发便携包已生成到桌面 `RobotFish-Dev`，项目docs按要求排除。双击包内Editor.cmd，在`.vscode/launch.json`选择启动上位机、编译固件、USB烧录（自动串口）、OTA升级（自动发现）或环境检查与补全；不开放YOLO训练等其他操作入口。每项操作结束给出实际结论并保存到`state/results/`，环境补全使用包内离线备份；说明见包内README.txt。初始化、配置、缓存和输出限定交付目录；本源工程继续保留原本机路径默认值，便携包通过局部环境变量与启动器适配。

便携环境版本：ESP-IDF 5.3.2、Python 3.13.14、PyTorch 2.11.0+cpu、Ultralytics 8.4.118、OpenCV 5.0.0、VS Code 1.143.1。完整清单见[便携环境版本](tools/portable_environment.json)；完整运行环境通过本仓库GitHub Release附件分发，Git保留源码与打包脚本。
