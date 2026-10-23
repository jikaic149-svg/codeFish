# LoRa 与 IMU 配置及操作记录

本文保留 2026-10-22 的 **2.3.2-lora-imu 实机记录**，不代表当前源码已烧录。当前源码为 2.5.0-attitude，新增姿态辅助代码默认关闭，安装映射待实测；本轮仅离线验证，未启动实鱼控制环路。当前能力和待测项见 [工程说明](../README.md)。

记录日期：2026-10-22。工程：`C:/Users/rlyoulo/Desktop/SAKANA/RobotFish-ESP-IDF`。

## 1. 先看最终结论

- 当前固件：**2.3.2-lora-imu**，已写入本次目标 ESP32。
- LoRa：真实定点接收 **10/10**；重复帧、旧序号和错误地址检查通过。
- IMU：JY61P，**115200/8N1**；roll、pitch、yaw 都有有效数据，并已观察到手动转动后的变化。
- 最终通过的软件与硬件测试使用 **Type-C COM26**；COM28 连接 LoRa 发送端。
- 用户明确要求 **不要更换电脑 Wi-Fi**。该要求提出后，未再切换或配置电脑 Wi-Fi。
- 运动命令从 LoRa 接收，Wi-Fi 保留遥测和 OTA；USB 是本次额外启用的只读观察接口。
- 只发送停止命令 17，未测试非零推进、负载或水池运动。

当前板上启用了 USB 观察输出；默认生产构建的 `ROBOTFISH_USB_DIAGNOSTICS` 为 OFF，但当前 build 缓存是 ON。

## 2. 设备身份与端口

| 项目 | 本次实际值 |
|---|---|
| 主控 | XIAO ESP32-S3，芯片 revision v0.2 |
| ESP32 MAC | `e8:f6:0a:8b:38:e4` |
| ESP32 Type-C | COM26，USB Serial/JTAG |
| Flash | 8 MB，esptool 实际识别 |
| PSRAM | 芯片识别为嵌入式 8 MB；本轮未另做 PSRAM 压力测试 |
| LoRa 发送端 | COM28，CP210x USB 转 TTL，用户确认仍连接 LoRa 模块 |
| EDA 工程 | 比赛机器鱼V4.0 XIAOesp32 |
| 实际回读 PCB | PCB1_1，UUID `230722f06b4c2713` |

COM27、COM29 出现在旧记录中，不能据此认作当前板。以后端口号变化时，先按 MAC/USB 序列号确认身份。

## 3. 最终接线：以本表为准

### 3.1 LoRa 与 PCB 四针口

| LoRa 引脚 | PCB 接点 | ESP32 功能 |
|---|---|---|
| VCC | 5V | 模块供电 |
| GND | GND | 共地 |
| TXD | **板 RX**，GPIO1 / D0 | UART1 RX，接收模块数据 |
| RXD | **板 TX**，GPIO2 / D1 | UART1 TX，发送到模块 |
| M0 / MD0 | 3.3V | 两端均由用户确认如此 |
| M1 / MD1 | GND | 两端均由用户确认如此 |
| AUX | 不接 | 不依赖 AUX 判定 |

**现在按常规 TXD→板 RX、RXD→板 TX 接线，不用互换线。**

前期软件曾分配 TX=GPIO1、RX=GPIO2，因此聊天中出现过“板 TX 接模块 TX”的旧说明。用户后来选择软件适配现有接线，2.3.2 已改为 **ESP TX=GPIO2、ESP RX=GPIO1**。旧接线建议已被本表取代。

### 3.2 JY61P 与 RGB / GPIO_IN1

| JY61P 引脚 | PCB 接点 |
|---|---|
| TX | GPIO9 / D10 / GPIO_IN1，U18 第 4 脚 |
| RX | GPIO7 / D8 / RGB，U18 第 2 脚 |
| GND | GND，可取 U18 第 6 脚 |
| VCC | 模块允许 3.3～5V，可取合适电源；本轮未用万用表测量实际供电电压 |

- UART2：ESP TX=GPIO7、ESP RX=GPIO9，115200、8 数据位、无校验、1 停止位。
- **RGB 信号脚接 IMU 的 RX**。RGB 口旁的 5V、GND 是电源脚。
- GPIO_IN1 在原设计中还连接 U16 光耦 LED 阳极，LED 阴极经 R1 1kΩ 接地。**用户已确认隔离该支路**；本轮未修改 PCB，也未记录具体拆焊位置。
- GPIO9 已用于 IMU RX，水泵功能保持关闭，并有编译断言避免重新启用水泵复用。
- 串口信号按 3.3V TTL 接入；各设备共地。

## 4. 通信参数与地址

| 参数 | 当前配置/验证范围 |
|---|---|
| LoRa 模块型号 | A39-T400A30D1a；具体版本沿用既有模块识别记录 |
| ESP32↔LoRa UART1 | 115200 / 8N1，TX GPIO2、RX GPIO1 |
| ESP32↔JY61P UART2 | 115200 / 8N1，TX GPIO7、RX GPIO9 |
| 无线工作方式 | 定点传输，实际按带目标地址的帧发送并成功 |
| 当前可达鱼端射频地址 | **0x0102**，本次空口收包验证 |
| 信道 | 0x20，按该信道完成测试 |
| 空中速率 | 2.4 kbps，来自此前模块回读配置；本轮未进入参数模式重读 |
| 发送端本机地址 | 本轮未重新读寄存器，不按旧端口归属推断 |
| AES | 沿用此前配对配置；未改写或公开密钥 |
| AES 参考指纹 | `f185089c96377f0b3cf6aab354a11c518eddb3c0ab4069e615cc2c2bb2a89d27`，来自既有回读记录 |
| LoRa 自动重传 | 此前回读配置为关闭；本轮不写模块参数 |
| 鱼端 LoRa ACK | 关闭，USB 实测 `rf_tx=0` |

**射频地址 0x0102 与应用层逻辑鱼节点 1 是不同字段。**

电脑向当前鱼端写入的串口帧是：

```text
01 02 20 + ASCII("100,1,来源,序号,命令[,旋钮]\n")
```

前三字节为二进制目标地址和信道，后面才是 ASCII 文本。测试停止命令例子：

```python
frame = bytes.fromhex("01 02 20") + b"100,1,23120,1,17\n"
```

本次先探测 0x0101，未到达鱼端；改发 0x0102 后 10/10 成功。不要沿用旧独立测试板的模块位置/地址归属假设。参数模式才使用两模式脚均低和配置串口 9600；本轮正常通信没有切入参数模式。

## 5. 应用命令与接收规则

| 命令号 | 含义 |
|---|---|
| 1 | 水平运动缓停，保留鱼鳍浮沉档 |
| 2 / 3 / 4 | 前进 / 左转 / 右转 |
| 5 | 螺旋桨→仿生→混合循环 |
| 6 | 上浮鱼鳍档，保留水平运动 |
| 7 | 从上浮回正常；再次收到则下潜 |
| 9 | 后退 |
| 10 / 11 / 12 / 13 | 前左 / 前右 / 后左 / 后右 |
| 14 / 15 / 16 | 螺旋桨 / 仿生 / 混合模式 |
| 17 | 急停全部运动并清除鱼鳍保持档 |
| 8 / 18 / 19 | 水泵不可用，明确拒绝 |

- 文本以换行结束，最大行长 32；来源、序号为 16 位无符号值。
- 序号按来源递增，支持 65535→0 回绕；重复或过旧帧不重新执行，也不续租。
- 保留最多 4 个来源的序号历史，近期事务缓存 4 条。
- 旋钮可省略，携带时映射到原 50～100% 档位。
- 单独切模式不启动运动，也不延长运动租约。
- 上述运动命令表是协议说明；**本轮硬件只实际发送了命令 17**。

## 6. IMU 数据解释

使用 WIT 私有串口协议：帧头 0x55，整帧 11 字节，最后一字节为前 10 字节累加和的低 8 位。

| 类型 | 输出 | 单位/换算 |
|---|---|---|
| 0x51 | 三轴加速度 | 有符号原始值 / 32768 × 16，单位 g |
| 0x52 | 三轴角速度 | 有符号原始值 / 32768 × 2000，单位 °/s |
| 0x53 | 三轴欧拉角 | 有符号原始值 / 32768 × 180，单位 ° |
| X / Y / Z 角度 | roll / pitch / yaw | 保留传感器原生坐标系 |

数据仅作为遥测，不反馈给运动闭环。本轮没有写入 IMU 波特率、输出频率、校准或保存命令；115200 是用户实际设置并经合法帧验证的工作速率。USB 快照发布为 10Hz，不能把它当成传感器内部输出频率。

## 7. USB 观察与复测

当前诊断版在 COM26 输出：

```text
USBSTAT {"ms":...,"accepted":...,"imu_frames":...,"roll":...,"pitch":...,"yaw":...}
```

| 字段 | 含义 |
|---|---|
| accepted / rejected | LoRa 接受/拒绝计数 |
| duplicates / stale | 重复帧/过旧序号计数 |
| source / sequence / code / result | 最近交给控制器的命令；result=0 为接受 |
| lora_errors / rf_tx | 接收流中断计数 / 是否启用 LoRa 回复 |
| imu_baud / imu_bytes / imu_frames | MCU 接收速率、已读字节、合法数据包计数 |
| imu_crc_errors / imu_rx_errors | 累计校验错误 / 接收流中断计数 |
| angles_valid / angle_age_ms | 姿态是否有效 / 姿态年龄 |
| roll / pitch / yaw | 原生三轴欧拉角，° |
| left / right / written_mask | 双桨请求百分比 / 六路提交掩码，63 表示六路均提交成功 |
| stack_free | USB 观察任务的最小空余栈，字节 |

计数从 MCU 启动累积。判断当前通信时，应比较测试窗口内是否新增错误，以及有效帧是否持续增加；不要把上电前或重新设置接收速率时的历史错误当成当前持续故障。

USB 观察任务运行在 CPU1，优先级 1，每 100ms 只读现有快照，静态栈 4096 字节。本次观察到最小空余约 1284 字节。原控制 owner 不通过 USB 接收运动命令。

测试构建还支持主机显式发送单字符，仅改变 MCU 的临时 UART2 接收速率，不写 IMU 参数：`1=115200`、`9=9600`、`2=19200`、`3=38400`、`5=57600`、`4=230400`。本轮只实际发过 `1`；掉电/重启回到配置文件的 115200。

本次 LoRa+IMU 串口测试脚本已保存为 [firmware_usb_bench.py](../tests/lora_pair/firmware_usb_bench.py)：

```powershell
python -B ./tests/lora_pair/firmware_usb_bench.py
```

复测前：模块上电、模式脚保持正常、使用带 USB 观察的固件，关闭其他占用 COM26/COM28 的串口软件。脚本使用固定来源 23120 和测试序号，**重复运行前重启 ESP32 清除来源序号历史**。它只发送停止命令，并把原始结果保存到系统临时目录。不要把旧的 `tests/lora_pair/bench.py` 桥接固件流程与当前主固件 USBSTAT 流程混用。

## 8. 当前 Wi-Fi 与运动配置

Wi-Fi 功能保留，但本轮最终硬件复测没有使用它：

| 项目 | 当前值 |
|---|---|
| AP SSID / 密码 | `RobotFishLOAR1` / `12345678` |
| 默认 AP IP | 192.168.4.1 |
| OTA Basic 用户 / 密码 | `fish` / `fishota` |
| 身份接口 | GET /get_ip |
| 遥测接口 | GET /lora_status、/motion_status、/status |
| 网页 / 升级 | GET /、/update；POST /ota_upload |
| Wi-Fi 运动入口 | POST /api/v1/control，正确鉴权后返回 403 / lora_control_only |
| 控制 / 回传 | control_transport=lora、telemetry_transport=wifi |
| STM32、电池 ADC、水泵、板载相机 | 当前主工程不提供这些功能 |

本次未完成 Wi-Fi HTTP 回传的实机验证，也没有为此重新切换电脑网络。

保留的执行器配置：

| 功能 | GPIO |
|---|---|
| 前尾 / 后尾 | 5 / 6 |
| 左鳍 / 右鳍 | 4 / 3 |
| 左桨 / 右桨 | 43 / 8 |

PWM 为 50Hz、14 位；舵机脉宽 500～2500µs，电调 1000/1500/2000µs，上电中立解锁 2000ms。前尾范围 58～122°、后尾 45～135°，中心均为 90°；左鳍 45～145°、右鳍 35～155°。软件角度/驱动提交成功不是实测机械位置。

主循环 10ms 轮询，20ms 输出；有效运动指令租约 3000ms。普通双桨启停斜率 200%/s，满幅到中立约 500ms；尾部过渡 50°/s、鱼鳍 90°/s。连续六路输出不完整 5 次进入中立恢复，15 次锁定；OTA 要求六路中立提交成功。上述运动/安全逻辑本轮沿用原配置。

## 9. 构建环境与烧录操作

本机使用：ESP-IDF `D:/ESP32/ESP-IDF-v5.3.2`，工具目录 `D:/ESP32/.espressif`，Python 环境 `idf5.3_py3.13_env`，CMake `D:/CMAKE/bin`，Ninja `D:/Ninja`；主机 C++ 回归编译器 `C:/msys64/ucrt64/bin/g++.exe`。目标 esp32s3，C++17，FreeRTOS Tick=1000Hz，支持静态分配，USB Serial/JTAG 控制台。

已有构建入口：

```powershell
Set-Location "C:/Users/rlyoulo/Desktop/SAKANA/RobotFish-ESP-IDF"
powershell -NoProfile -ExecutionPolicy Bypass -File ./build.ps1
```

该入口沿用已有 CMake 缓存；当前缓存中的 USB 观察为 ON。需要明确切换选项时，在同一 ESP-IDF 环境中执行：

```powershell
$env:IDF_PATH = "D:/ESP32/ESP-IDF-v5.3.2"
$env:IDF_TOOLS_PATH = "D:/ESP32/.espressif"
$env:IDF_PYTHON_ENV_PATH = "D:/ESP32/.espressif/python_env/idf5.3_py3.13_env"
$env:IDF_COMPONENT_MANAGER = "0"
$env:PATH = "$env:IDF_PYTHON_ENV_PATH/Scripts;D:/CMAKE/bin;D:/Ninja;" + $env:PATH
. "$env:IDF_PATH/export.ps1"
idf.py -B build -DIDF_COMPONENT_MANAGER=0 -DROBOTFISH_USB_DIAGNOSTICS=ON build
```

关闭 USB 观察时，将最后一行的 ON 改为 OFF。上述命令只构建，不自动烧录。

本轮首次按生成的 `build/flash_args` 写入四个镜像；之后同分区布局下仅更新应用。实际地址：

| 内容 | 地址 |
|---|---|
| bootloader | 0x000000 |
| partition table | 0x008000 |
| OTA data | 0x00e000 |
| app0 应用 | 0x010000 |
| app1 | 0x340000 |
| SPIFFS | 0x670000 |
| coredump | 0x7f0000 |

当前布局的应用更新命令示例，在工程根目录运行：

```powershell
& "D:/ESP32/.espressif/python_env/idf5.3_py3.13_env/Scripts/python.exe" -m esptool --chip esp32s3 --port COM26 --baud 460800 write_flash 0x10000 build/robotfish.bin
```

先确认目标 MAC，且串口未被其他程序占用。不要把 app BIN 当整片镜像写到 0x0。

## 10. 备份、镜像与校验

### Git 初版

工程根目录已初始化 Git，分支 main，提交 **aaf80f1**，备注“初版无刷鱼，esp32版本”，共 109 个文件。备份覆盖当时固件、上位机、测试和构建配置；历史归档、缓存和生成物未纳入。后续本轮改动保留在工作区，没有自动追加提交或推送。

### 原板 Flash

- 目标：COM26 / MAC e8:f6:0a:8b:38:e4。
- 原应用：robotfish 2.2.6-vision。
- 备份范围：0x000000～0x0e7fff，共 **950272 字节**，覆盖本轮所有被改写扇区；并非整片 8MB 备份。
- 文件：[原始 Flash](../artifacts/hardware-backups/COM26-e8f60a8b38e4-before-2.3.1-20261022.bin)。
- SHA256：`5e29c2c699254d3c0c874d7b8cff68b1302ed11dba07a9983962a650bfb734e3`。

这是从 0x0 读取的原始扇区镜像，不能按 app BIN 写到 0x10000。只有需要回退到本轮开始状态时，才对同一目标按原范围写回；写回会恢复该范围内当时的 NVS/OTA 数据。它不覆盖范围以外后来发生的改动。

### 本轮主要生成镜像

| 阶段 | 文件名 | 字节数 |
|---|---|---|
| 2.3.0 初次 LoRa/IMU 构建，IMU 9600，未在本轮烧录该镜像 | RobotFish-ESP32S3-OTA-20261022-162704.bin | 880656 |
| 2.3.1，IMU 115200，先写入测试 | RobotFish-ESP32S3-OTA-20261022-183002.bin | 880656 |
| 2.3.1，加入 USB 观察 | RobotFish-ESP32S3-OTA-20261022-191246.bin | 881792 |
| 2.3.1，加入显式 MCU 接收速率诊断 | RobotFish-ESP32S3-OTA-20261022-193156.bin | 882400 |
| **2.3.2，软件适配 LoRa 接线，当前板上版本** | **RobotFish-ESP32S3-OTA-20261022-200659.bin** | **882400** |

当前固件：[2.3.2 BIN](../artifacts/RobotFish-ESP32S3-OTA-20261022-200659.bin)。

SHA256：`b26f6b56e716a1e59fba610cafff7c9d612a6f96336be0384ae4f54b4bb44b1f`。写后哈希验证及本地 BIN 头、项目、芯片、校验和和 SHA256 检查均通过。所有镜像摘要另保存在配置快照中。

## 11. 操作经过与原因

| 顺序 | 操作与结果 |
|---|---|
| 1 | 修改前创建 Git 初版备份 aaf80f1。 |
| 2 | 用 easyeda-agent 只读连接当前 PCB 和关联原理图，确认网络到 GPIO 的映射；未编辑 PCB。 |
| 3 | 将原 Wi-Fi 控制主程序接入 LoRa 接收，并增加 JY61P 接收器；Wi-Fi 保留结果/状态与 OTA。 |
| 4 | 初版按 JY61P 厂家默认 9600 准备；用户确认实际 115200 后同步修改固件和回归。 |
| 5 | 识别 COM26 与 MAC，备份所有拟擦写扇区，写入并验证镜像。 |
| 6 | 曾尝试临时连接鱼热点读 HTTP，未连上，已恢复原网络。用户明确禁止切换 Wi-Fi 后，改为 Type-C 方案，没有再重试切网。 |
| 7 | 增加可关闭的 USB 只读观察，直接检查主程序中的 LoRa/IMU 状态。 |
| 8 | 最初 IMU 有持续接收错误、无合法姿态帧；用户确认此前未上电。上电后合法帧增长、区间新增错误归零。 |
| 9 | 确认用户已隔离 GPIO9 光耦支路。传感器手动转动后，三轴值均发生变化。 |
| 10 | LoRa 仍无收包；依次确认两端 M0/M1 和 COM28 发送端连接，最终确认现有信号线与旧软件引脚方向相反。 |
| 11 | 用户选择软件适配，不改线；改为 GPIO2 TX / GPIO1 RX，构建并烧录 2.3.2。 |
| 12 | 当前地址 0x0102 下收到 10/10 停止帧，防重、旧序号、错误地址检查通过；IMU 同时持续更新。 |

诊断期间没有更改 LoRa 信道、地址、AES、模块模式寄存器，也没有写 IMU 参数。曾加入临时 MCU 接收速率请求能力，但实际只选择了已经确认的 115200。

## 12. 实测结果与验证范围

| 项目 | 结果 |
|---|---|
| LoRa 定点接收 | 地址 0x0102 下 10/10 |
| 重复帧 | 接受计数不变，duplicates 增加 |
| 旧序号 | 不执行，stale/rejected 增加 |
| 错误地址 0x017E | 鱼端命令计数不变 |
| LoRa UART 接收错误 | 最终测试窗口为 0 |
| 主机观察回执时间 | 约 199～240ms，包含 USB 发布/主机调度，不是纯空口时延 |
| IMU 连续合法帧 | 最终窗口约 601 个已识别数据包/秒，三种类型合计 |
| IMU 新增 CRC / 接收错误 | 最终窗口均 0；启动前已有的累计中断不计作新增 |
| 初次静止观察 | 450 个新鲜快照；roll 范围 0.104°、pitch 0.104°、yaw 0.834°，非标定精度 |
| 手动转动前后比较 | 相对初始姿态，roll 18.765°、pitch 19.990°、yaw 113.087° 的变化范围 |
| 转动观察有效样本 | 23 条中首条因显式重置接收而无效，剩余 22 条有效且新鲜 |
| 输出状态 | 双桨始终 0，written_mask=63 |
| 固件构建/回归 | 299 条控制状态 + 8066 条主循环通过；原运动核心 60561 条/51840 周期、BSP、网页检查保留通过证据 |

这些结果证明当前通信链路和三轴数据响应可用；未进行独立量角精度、航向长期漂移、实际推进/水池动作或 Wi-Fi HTTP 的本轮实机测试。

## 13. 配置入口与原始证据

完整配置原文、文件 SHA256、SDK 配置、分区、构建开关及镜像摘要保存在 [配置快照](../validation/lora-imu-config-snapshot-20261022.json)。普通阅读先看本文件；需要核对细节再查快照。

| 文件 | 用途 |
|---|---|
| [BoardConfig.h](../main/config/BoardConfig.h) | GPIO、UART2 速率、PWM、脉宽和水泵复用限制 |
| [RadioConfig.h](../main/config/RadioConfig.h) | LoRa UART 速率、逻辑节点、行长、缓存、防重和关闭的 ACK 参数 |
| [AppConfig.h](../main/config/AppConfig.h) | 热点、认证、循环周期、租约、输出故障和 OTA 参数 |
| [RuntimeConfig.h](../main/config/RuntimeConfig.h) | 中心角、机械限位、方向、档位和摆频 |
| [ControlConfig.h](../main/config/ControlConfig.h) | 运动斜率与分配参数 |
| [MotionAllocationConfig.h](../main/config/MotionAllocationConfig.h) | 将校准值组合为运动配方/几何 |
| [主循环](../main/robotfish.cpp) | 串口轮询、唯一运动 owner 与快照发布 |
| [USB 观察](../main/Service/Usb/UsbTelemetry.cpp) | USBSTAT 格式及显式接收速率请求 |
| [实机报告](../validation/lora-imu-usb-20261022.json) | 最终逐包记录、原始姿态样本和各阶段结果 |
| [process.md](../process.md) | 开发操作时间线，保留被后续方案覆盖的历史 |
| [早期模块回读](../validation/lora-fixed-pair-config.json) | 旧配对参数，端口/模块物理归属不能直接套用到当前接线 |

参考资料：[用户提供的 A39 PDF](http://www.ashining.com.cn/datasheets/a39t400a30d1acn.pdf)、[JY61P 官方规格书](https://wit-motion.yuque.com/wumwnr/docs/agok0k)。JY61P 规格正文已读取；A39 链接在本环境获取时出现拒绝/超时，本轮 A39 参数依据既有回读与实际空口测试记录，没有假设已读取该 PDF 全文。
