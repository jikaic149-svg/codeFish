# 2026-10-22 开发记录

来源：归档前根目录 `process.md`。以下正文按原文保留；历史决策只适用于各自版本和试验条件。

<!-- original-process-entry: 016 sha256: 383e43a18398099da2d5abd7fb43f3a59de7324f8e6627050cf471f4f0427ccf -->
## 2026-10-22：A39 LoRa 收发端配置

用户指定使用桌面 LoRa 串口工作台 1.3.0，明确当前模块为接收端，正常通信采用 115200，实际收发测试由用户完成。通过该工作台的 HEX 发送功能，在 COM28、配置串口 9600／8N1 下，对 A39-T400A30D1a-9.26 写入并回读确认：工作串口 115200／8N1、信道 0x20；空中速率保持 2.4 kbps、透明传输。两次写入均收到模块成功应答，证据为 validation/lora-receiver-configured.json，原读取基线为 validation/lora-module-A.json。

发送端换接后识别为 A39-T400A30D1a-9.24。COM28 曾返回 Windows 错误 31，重启对应 CP2102 USB 设备后恢复。通过同一工作台完成发送端 115200／8N1、信道 0x20、空速 2.4 kbps、透传及本地／目标组号 1、地址 0 的写入和回读；与接收端 AES 密钥指纹一致。证据为 validation/lora-sender-configured.json。

工作台调参页的 DX-LR32 AT 协议与 A39 不同，本次使用原始 HEX 寄存器指令。两端配置完成，COM28 已释放，工作台电脑端速率预置为 115200。用户只有一个 USB 转 TTL，两端逐块配置；实际无线收发留给用户测试。正常工作须 MD0 为高、MD1 为低，配置模式固定使用 9600。机器鱼固件未因本次模块配置启用 LoRa。

<!-- original-process-entry: 017 sha256: 361607563fd008444debdd77b6b7d9c8e5fd274edef6b3d1e2770bb89b6fc24a -->
## 2026-10-22：定点双向无线联调完成

用户新增一块 ESP32-S3，要求实际验证双向收发，并明确仅这一对模块采用定点通信，覆盖上节的透传选择。接线按 GPIO1 TX→LoRa RXD、GPIO2 RX←LoRa TXD；用户不接 GPIO7，模式脚采用外部固定电平。配置时两脚均接地，正常时 MD0 接 3.3V、MD1 接地；裸模块不能将两脚都悬空作为可靠正常模式。

新增独立 tests/lora_pair 测试固件和 bench.py，未修改机器鱼主程序。新增 ESP32 为 COM29、MAC dc:b4:d9:3b:2a:b8，与此前烧录机器鱼的设备不同。烧录前按实际擦写扇区范围备份原始 Flash；诊断固件扩大时也先追加备份未改写区域。测试完成后已恢复原始 0x00000～0x42fff 共 274432 字节，并通过 esptool verify_flash 校验；备份位置、范围及 SHA256 见 validation/lora-pair-esp32-backup.json。

两端均已写入并回读确认定点模式 0x0002、信道 0x20、115200／8N1、2.4 kbps。ESP32 端 A39 9.26 为本机 0x0101、目标 0x0102；USB 转 TTL 端 A39 9.24 为本机 0x0102、目标 0x0101。更换并核对此次配对专用 AES 密钥，未使用广播地址，关闭模块自动重传。密钥仅存于当前用户临时目录，公开记录仅含指纹。validation/lora-fixed-pair-config.json 的 complete=true。

联调初期 ESP32 端外部 LoRa 没有回应。UART1 内部硬件回环通过；明确关闭内部回环后，用户将 D0、D1 短接，115200 下带随机标识的物理外部回环也完整通过。用户撤掉短接，按 LoRa RXD→D0、TXD→D1、共地、两模式脚均低重新连接后，两端身份读取和配置成功。诊断证据为 validation/lora-esp32-uart-diagnostic.json。

用户将两端切为 MD0 高、MD1 低后，完成真实定点空口测试：ESP32→USB 端 10/10，USB→ESP32 端 10/10，带独立随机标识和 CRC 的载荷逐字节一致。两个方向各发一帧到错误地址 0x017E，接收端均没有输出，定点地址过滤通过。未做自动重发，未将本地回显计作空口成功。完整逐包记录为 validation/lora-fixed-pair-test.json，all_passed=true、complete=true，包含测试固件和主机脚本 SHA256。日常保持 MD0 接 3.3V、MD1 接 GND，不需要接 GPIO7；LoRa 配置保留，ESP32 原固件已恢复。此结果验证模块链路，机器鱼控制程序未因此新增 LoRa 协议接入。

<!-- original-process-entry: 018 sha256: 1f0b702ab06fdb8c4edcc38ccb385a69bebff75a526495ff1b4cbcdb79dbf75e -->
## 2026-10-22：M0/M1 悬空实验完成，最终保留六线固定方案

用户要求继续实测两块模块的 M0/M1 全部悬空，故重新启用同一独立 ESP32 测试桥；配对地址、信道、速率和专用密钥均未改动。原始 Flash 备份保留，当前测试桥仍在板上，最新恢复状态以 validation/lora-pair-esp32-backup.json 为准。

用户确认两块模式线全部断开、LoRa 尚未断电后测试：ESP32→USB 0/3，USB→ESP32 0/3，均没有收到载荷，记录为 validation/lora-floating-powered.json。随后按用户确认完成整套断电再上电，模式脚仍悬空，两个方向再次均为 0/3，记录为 validation/lora-floating-cold_start.json。此前正常固定电平下双向各 10/10 的基准记录保留，不将悬空失败覆盖成基准失败。

用户最终明确“保留六根线的解法”“之前已经测试过了”，故停止追加恢复复测，沿用已有正常方案验证结果。六线定稿：LoRa RXD→D0、TXD→D1、VCC→稳定 5V、GND 共地、M0→3.3V、M1→GND；AUX 不接，不接额外模式控制线。两端定点地址为 0x0101↔0x0102，115200／8N1、信道 0x20、空速 2.4 kbps、专用 AES 密钥匹配。已同步 README 的六线接法与测试记录索引。当前独立测试板保留串口桥接程序，原镜像备份仍保留；本次用户定稿仅记录方案，不再发送测试包或改写固件。

<!-- original-process-entry: 019 sha256: 71ef54b7dc092a27cd2ca5cc0c36251fbe7ab789a11e2a7864896e78c6a751b3 -->
## 2026-10-22：六线固定方案追加复测通过

用户恢复会话后明确要求再测试一次，覆盖上一节不追加复测的决定。复用 tests/lora_pair/bench.py 和板上串口桥，在 COM29（ESP32）与 COM28（USB 转 TTL）执行双向各 10 包定点空口测试：两个方向均为 10/10，包含本轮随机标识及 CRC 的载荷逐字节一致；两个方向各一帧到错误地址 0x017E 均被过滤。

本次未改写模块参数或烧录固件，独立测试板继续保留串口桥。新结果单独保存为 validation/lora-fixed-pair-retest.json，run_id=5f1679ff、all_passed=true、complete=true；原通过基准及悬空失败记录保留。已同步 README 和 validation/lora-wiring-decision.json。本次验证范围仍为模块链路，未验证机器鱼 LoRa 运动控制。

<!-- original-process-entry: 020 sha256: 34574ef218a8ad19f309d9ed337760293ffff958389c0fb4f564a0377c739b27 -->
## 2026-10-22：Wi-Fi 静态连接检查与 COM27 烧录

用户要求先制作静态测试，只排查 Wi-Fi 连接建立，不连接 ESP32 热点。用户移除其他板后，唯一 ESP32 为 COM27、ESP32-S3 v0.2、8 MB Flash、MAC e8:3d:c1:fa:ee:f0；此次按用户要求将机器鱼主固件烧录到该设备，未操作此前 COM29 的独立 LoRa 测试板。

新增 tests/wifi_connection/regression.py，HTTPConnection 全部替身并禁止真实 socket，测试仅允许 acquire 连接报文。覆盖 AP-only 身份、遥测与控制分离、两种密码、网络不可达、旧固件、运行未就绪和会话被替换。工程自带上位机与原 robofish_new/upper 各 8 项通过，已有真实 cJSON Wi-Fi 协议 97 项通过。没有修改固件或上位机业务逻辑。

当前 2.2.6-vision 编译成功，应用 867216 字节，SHA256 为 7158903167335fafef83c0d8f09df539e480ce16e36da34335082a1a03dd441b。四镜像写入哈希校验通过，USB 启动日志确认该版本、softAP、192.168.4.1 DHCP 与 HTTP 路由注册成功。Windows 原生 WlanScan 收到扫描完成通知后，可见 RobotFishLOAR1、BSSID e8:3d:c1:fa:ee:f1、信道 1、信号 80%；此前 netsh 列表未刷新，不能将旧列表当作热点未广播的证据。

当前电脑仍连接西湖大学-2.4G、地址 172.16.20.147，无 192.168.4.0/24 直连路由。源码只设置 WIFI_MODE_AP，没有连接校园网的 STA 流程；sta_connected=false、sta_ip=0.0.0.0 为固定身份字段，不能据此判断 AP 故障。当前可确认电脑与设备没有建立同一网络连接；本轮没有复现实机关联或认证失败，不能据静态结果排除这些阶段的问题。未连接热点、未访问鱼端 HTTP、未发送实鱼运动命令。

证据为 validation/wifi-static-20261022.json 及同名前缀的 flash、boot、scan、pc-wifi 日志。静态测试可执行 python -B tests/wifi_connection/regression.py -v；使用 --upper 指向原上位机可复用同组测试。后续若继续检查实际关联、DHCP 取址或控制认证，需要另行进行连接测试。

烧录完成后的最终回读发现，工程出现了非本任务写入的 LoRa／IMU 并行改动，涉及 main/CMakeLists.txt、robotfish.cpp、network.h、ControlState.h 等；其中控制循环和 networkBegin 接口已发生变化。这些文件修改时间晚于本轮 BIN 生成时间，未纳入本轮镜像，未被本任务覆盖、重新构建或烧录。以上检查结论仅对应已记录 SHA256 的 2.2.6-vision Wi-Fi 镜像，不代表这些后续源码改动已经验证。

<!-- original-process-entry: 021 sha256: 63576f50ced1d506bc04c7b20416f2f67d6cd2c0e5594c7378064ba52a257e23 -->
## 2026-10-22：2.3.0 LoRa 接收、Wi-Fi 回传与 JY61P 串口

用户要求修改前先用 Git 备份，已在工程根目录初始化仓库并提交当前固件、上位机、测试与构建配置，共 109 个文件；提交 aaf80f1，备注“初版无刷鱼，esp32版本”。历史归档和构建产物未纳入。随后用户纠正：LoRa 继续使用 PCB 的 GND／5V／TX／RX 四针接口，GPIO7／GPIO9 另作陀螺仪串口，型号由用户提供的维特资料确认是 JY61P。

通过本地 easyeda-agent v0.21.2 连接现有编辑器，只读回查“比赛机器鱼V4.0 XIAOesp32”工程 PCB1_1 及其关联原理图：四针 RX 网→XIAO D0/GPIO1、TX 网→D1/GPIO2；RGB→D8/GPIO7、GPIO_IN1→D10/GPIO9。GPIO_IN1 同接 U16 光耦 LED 阳极，阴极经 R1 1k 接地；若已焊装，接 JY61P 前须隔离该支路。未修改 PCB。读取官方 JY61P 规格书确认 UART 出厂默认 9600 bps，使用 WIT 私有 11 字节协议。

生产构建启用 UART1 的 LoRa 接收和现有行解析、按来源去重与序号回绕；GPIO1 TX、GPIO2 RX、115200/8N1。主循环唯一拥有 UART 和运动状态，仍按 10 ms 轮询、20 ms 输出。LoRa 命令按原 ino 映射，浮沉保持独立、普通停止缓停、17 急停；未接的水泵命令明确拒绝。既有 3 秒失联、5/15 次输出故障和 OTA 六路中立确认均保留。HTTP 运动入口改为 lora_control_only，移除生产命令/结果 FIFO；Wi-Fi 使用现有快照发布最新命令结果和 910 回执，不发送 LoRa ACK，不保证逐包推送。

UART2 初始化为 GPIO7 TX、GPIO9 RX、9600/8N1，编译时禁止 GPIO9 水泵复用。新增有限预算非阻塞接收及 JY61P 累加校验、丢帧重同步，独立发布加速度 g、角速度 deg/s、原生欧拉角 deg、各组有效标志及年龄；不反馈到运动控制，不自动改传感器参数。设备网页改为遥测和 OTA。涉及 main/CMakeLists、BoardConfig、ControlState、robotfish、network、LoRaCommand、新 ImuUart/Jy61p/ImuStatus，以及 tests/control 的新主循环回归。

验证：ESP-IDF 5.3.2 构建通过；新 LoRa/IMU 实际主循环共 8066 条检查（正常/OTA 1787，失联 5140，恢复 479，锁定 660），原控制状态 299 条，运动核心 60561 条／51840 周期、实际 BSP 和网页回归均通过。覆盖双 UART 编号/引脚/波特率、无串口回传、重复/旧帧、软启停、急停、失联回正、部分输出失败、恢复不重放、故障锁定、OTA 屏障及 IMU 校验/重同步。

构建版本 2.3.0-lora，OTA 为 artifacts/RobotFish-ESP32S3-OTA-20261022-162704.bin，880656 字节，SHA256 25af76c9ab815c660bbee8d24aec85913f253dc9fa8e61215b8f476ecc289df6。未在本轮烧录、连接鱼热点或下发实鱼运动命令；模块实际波特率、光耦支路处理、陀螺仪接线和整机无线联调仍待硬件验证。前节 COM27 的 2.2.6-vision 烧录记录保留，不能当作本版已烧录证据。验证摘要见 validation/lora-wifi-imu-20261022.json。

<!-- original-process-entry: 022 sha256: 67fc45a44236206cb434e3eda5f61e3b29c0e56d45825a58d60d0434fe9f34a7 -->
## 2026-10-22：实机 USB 测试、IMU 115200 与 LoRa 接线修正（进行中）

用户确认 IMU 已设为 115200，要求实际测试 LoRa 和 yaw/pitch/roll。最新明确约束：“不要更换wifi”“测试IMU也可以用串口，我连接了typec”；后续本任务仅使用 Type-C COM26 和发送端 COM28，不再切换、连接或更改电脑 Wi-Fi。此前曾尝试一次热点 HTTP 联调但未连上，脚本已恢复原网络；该失败记录仅说明当次 HTTP 不可达，不表示 LoRa 或 IMU 测试通过。

已通过 esptool 核对 COM26 为 ESP32-S3 v0.2、8MB Flash、MAC e8:f6:0a:8b:38:e4。原程序为 robotfish 2.2.6-vision。烧录前按四镜像真实擦写扇区覆盖范围备份 0x000000～0x0e7fff，共 950272 字节；文件 artifacts/hardware-backups/COM26-e8f60a8b38e4-before-2.3.1-20261022.bin，SHA256 5e29c2c699254d3c0c874d7b8cff68b1302ed11dba07a9983962a650bfb734e3。2.3.1-lora-imu 已烧录，四镜像写后哈希均通过。

GPIO7 TX / GPIO9 RX 保持原接法，IMU 改为用户确认的 115200/8N1。控制状态与主循环回归 299+8066 条、网页回归及 ESP-IDF 构建通过。增加默认 OFF 的 ROBOTFISH_USB_DIAGNOSTICS 构建选项：只读最新快照的 CPU1 低优先级静态观察任务，100ms 输出 USBSTAT，静态栈 4096 字节；实际最小空余 1284 字节。运动控制仍由原 10/20ms owner 负责，不从 USB 接收运动指令。诊断构建支持主机显式请求 MCU 接收速率，仅 '1'（115200）被实际使用，未向 IMU 写配置或校准；正常 OFF 构建不含观察任务或该请求接口。

最初 IMU 无合法帧且接收错误持续增长，随后用户确认“刚才没上电”，并确认光耦支路已隔离。上电后 115200 下合法帧连续增长，新增 CRC/接收错误均为 0；初始静止姿态约 roll=-95.90°、pitch=0.01°、yaw=-23.28°。初次 45 秒采样收到 450 个新鲜姿态快照，静止范围 roll 0.104°、pitch 0.104°、yaw 0.834°，不可将此窗口当成定向转轴动作已完成。

用户随后转动传感器。相对最初静止姿态，实际读出变化范围为 roll 18.765°、pitch 19.990°、yaw 113.087°；最后新鲜读数约 roll=-77.964°、pitch=-19.715°、yaw=-135.022°。连续采样的 23 个快照中首个因显式重置接收速率而无效，剩余 22 个均有效且新鲜；新增 CRC/接收错误为 0，双桨始终为 0、written_mask=63。仅验证三轴有效输出和对手动转动的响应，未做量角精度标定。

LoRa 上电后发送停止帧到 0x0101 与 0x0102，鱼端暂未收到。用户确认两端 M0=3.3V/M1=GND，COM28 仍接 LoRa 发送模块。之后确认鱼端实际接成 TXD→PCB RX、RXD→PCB TX；与 PCB 网络和当前 GPIO 映射相反。已说明本板必须 TXD→PCB TX/GPIO2（ESP RX）、RXD→PCB RX/GPIO1（ESP TX），当前等待用户断电互换后重新上电，再做真实收包、去重、旧序号验证，尚不能宣称 LoRa 通过。

当前板上为带 USB 观察的 2.3.1-lora-imu，最新镜像 artifacts/RobotFish-ESP32S3-OTA-20261022-193156.bin，882400 字节，SHA256 717b336824f053b76fc4b69bfec1b145483f45b329f4a87fa2e009f87aa2e240；build 缓存 ROBOTFISH_USB_DIAGNOSTICS=ON。正式关闭观察可用 -DROBOTFISH_USB_DIAGNOSTICS=OFF 重新构建。实机原始样本和阶段结论保存在 validation/lora-imu-usb-20261022.json，后续应补写 LoRa 修线复测结果。

<!-- original-process-entry: 023 sha256: 904946c8368b7781b2f4750264a35a9772d98ef4aa8fbdba5b664426063bea11 -->
## 2026-10-22：2.3.2 软件适配现有 LoRa 接线，USB 实机复测通过

用户确认鱼端保持 LoRa TXD→PCB RX、LoRa RXD→PCB TX，并询问能否软件修改。按用户最新选择，改为 UART1 TX=GPIO2（PCB TX）、RX=GPIO1（PCB RX）；不要求用户互换信号线，取代上一节“等待换线”的临时方案。GPIO7 TX/GPIO9 RX 的 JY61P 保持 115200/8N1。对应实际主循环引脚回归同步更新，299 条控制状态及 8066 条主循环检查通过，2.3.2-lora-imu 构建通过。

已在用户重新连接 Type-C、确认模块上电后写入 COM26 的应用分区，目标 MAC e8:f6:0a:8b:38:e4，写后哈希校验通过。镜像 artifacts/RobotFish-ESP32S3-OTA-20261022-200659.bin，882400 字节，SHA256 b26f6b56e716a1e59fba610cafff7c9d612a6f96336be0384ae4f54b4bb44b1f。继续保留只读 USB 观察，未再次操作电脑 Wi-Fi。

COM28 向当前鱼端真实可达的定点地址 0x0102、信道 0x20 发送 10 条不同序号的 ASCII 停止命令；COM26 的主程序快照逐条确认接受，共 10/10。0x0101 的地址探测未到达，不能沿用历史独立测试板的旧地址归属假设。本次未重写两块模块的参数或密钥。定点地址 0x0102 与应用载荷内逻辑鱼节点 1 是不同字段。

重复最后一帧：接受计数不变、duplicates 增加；过期序号：接受计数不变、stale/rejected 增加；发到错误地址 0x017E：鱼端命令计数不变。LoRa UART 接收错误始终 0；rf_tx 始终 0，未启用 LoRa 回复。全部只发送命令 17，双桨始终为 0、六路 written_mask=63，没有测试非零推进或水池运动。主机观察到逐包回执约 199～240 ms，此值包含 USB 状态发布与主机调度，不等于纯空口延迟。

同一复测窗口 JY61P 合法帧持续增加，姿态 age=0 ms 的快照可读，新增 CRC/接收错误均 0；累计的一次接收中断为本窗口开始前已有记录。此前用户手动转动的 yaw/pitch/roll 三轴响应证据保留，不将初次静止采样或接收速率重置时的首个无效快照计作动态响应通过。验证范围为三轴有效输出、连续更新和手动转动响应，未进行角度精度或航向长期漂移标定。

最终数据更新到 validation/lora-imu-usb-20261022.json，status=passed、all_passed=true；包含上电前后阶段、三轴原始快照、最终逐包 RF 请求与 USB 接收结果。README 已更新现行接线：LoRa TXD→PCB RX/GPIO1、RXD→PCB TX/GPIO2；IMU TX→GPIO9、RX→GPIO7。初版 Git 备份和原始 Flash 扇区备份保留。本轮完成通信和姿态响应测试，板上保留启用 USBSTAT 的 2.3.2 测试构建。

<!-- original-process-entry: 024 sha256: e6aaec22d872253d5ab3412dfca5fb5899e2c501d7c0cc870a3d8aedce34b742 -->
## 2026-10-22：完整配置及操作归档

按用户要求，统一阅读入口为 docs/lora-imu-setup-and-test.md，README 顶部已添加链接。完整配置快照为 validation/lora-imu-config-snapshot-20261022.json，包含 19 份配置原文及摘要、43 个主控源文件摘要和 5 个镜像校验信息。实机数据为 validation/lora-imu-usb-20261022.json，最终 2.3.2 的 LoRa 10/10 与三轴姿态响应均已通过。复测脚本保存为 tests/lora_pair/firmware_usb_bench.py，固定测试序号，重跑前须重启主控；只发送停止命令，不操作电脑 Wi-Fi。已检查配置/镜像一致性、文档链接、UTF-8 与脚本语法。未因归档重新运行硬件测试，也未追加 Git 提交或推送。

