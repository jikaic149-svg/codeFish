# process

## 2026-08-14 三模式收紧转弯与HYBRID慢升油

- 修改原因：用户实测推进速度改善后转弯半径仍过大，并确认BIONIC、PROP、HYBRID三种模式都要提高转向能力；同时要求HYBRID螺旋桨更慢启动以减小瞬时反扭矩，且不增加IMU。
- 最终决策：BIONIC/HYBRID双尾舵转向偏置由12°/8°提高到18°/14°；纯PROP/HYBRID螺旋桨转向舵角由24°/8°提高到32°/16°；纯PROP网页直行动力保持100%，左右转动力单独降为80%，避免满油门顶舵扩大半径；ESC转向每20 ms渐变由4°提高到8°。HYBRID仅在油门离开中位、动力幅值增大时使用每20 ms 1°慢升油，减速、STOP和换向先以原3°速度回中；BIONIC和纯PROP油门渐变不变。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.cpp`、`src/web/ProductWebService.cpp`、`process.md`。
- 验证方式和结果：静态计算确认最大转向时BIONIC两尾舵范围30.5°～141.5°/34.25°～137.75°，HYBRID为28.5°～143.5°/32.85°～139.15°，均在20°～152°限位内；PROP/HYBRID转向目标分别58°～122°、74°～106°，均在40°～136°限位内。HYBRID正向从中位升至75%约0.62秒，STOP约0.22秒回中；PROP转弯正向油门目标由满动力124降为117。ESP-IDF 5.3.2增量构建、链接和应用分区尺寸检查通过，应用镜像937456字节，OTA分区剩余72%。生成 `RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260814-135524.bin`，SHA-256 为 `ebdc9e5b3c2ce7daa8746591b7cc66c90754c581e3b5f38a7f899e6222c4569c`。
- 已知限制：用户已报告当前实机在水下存在侧翻，具体发生模式、方向和稳态/瞬态特征仍待分离；本次新镜像尚未烧录和入水验证。增大转向力可能同时增大横向载荷，编译与静态限位通过不能证明侧翻得到解决。软件没有IMU或横滚闭环，本次仅通过慢升油和开环转向参数降低瞬态风险。
- 待确认事项：入水分别测试绿色BIONIC、红色PROP和紫色HYBRID的左右转弯半径、侧翻方向和停车时间；若直行仍固定向同一侧翻，优先检查舵机中位、尾鳍安装、左右配重和螺旋桨反扭矩，不继续盲目增加转向角。

## 2026-08-14 网页尾部动力增强与后舵真滞后

- 修改原因：用户入水实测 BIONIC/HYBRID 推进偏弱、双尾舵摆幅不足，并观察到前后尾舵配合仍有卸力感；当前网页尾部模式统一使用55%动力，且生成器采用 `sin(phase + offset)`，原0.1667/0.20周期会让后舵提前而非滞后。
- 最终决策：网页 BIONIC/HYBRID 动力由55%提高到75%，纯 PROP 保持100%；在不改生成器公式的前提下，将 BIONIC/HYBRID 相位偏移改为0.83333333/0.80周期，分别等效于后舵滞后60°/72°；每20 ms尾舵渐变上限由6°/10°配套提高到8°/11°并集中到 `sdk_config.h`。频率1.7/2.0 Hz、前舵幅值基数50°/58°、后舵0.9比例及转向偏置保持不变。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.cpp`、`process.md`。
- 验证方式和结果：静态计算确认75%下 BIONIC 前/后舵半摆幅为37.5°/33.75°、每周期最大变化7.996°/7.196°；HYBRID为43.5°/39.15°、每周期最大变化10.904°/9.814°，均不超过新渐变上限。含最大转向偏置时两舵目标仍在34.5°～137.5°内，未触及20°～152°软件限位。ESP-IDF 5.3.2增量构建、链接和应用分区尺寸检查通过，应用镜像937328字节，OTA分区剩余72%。生成 `RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260814-131937.bin`，SHA-256 为 `d935f1c26bc4a48ff06db893ed3222405de9d6d51ecca80a11da5c8cac7a4cff`。
- 已知限制：尚未烧录和入水验证；静态限位与编译通过不能证明舵机在水载荷下能跟随8°/11°每周期的变化，也不能替代前后舵物理安装方向确认。
- 待确认事项：先架空低时长点动，确认后舵实际跟随前舵且无机械干涉，再入水比较直行推力、尾端合成摆幅、电流、偏航和HYBRID螺旋桨动力；若后舵仍弱，再单独评估0.9幅值比例。

## 2026-08-14 HYBRID 尾舵频率柔化

- 修改原因：用户实测 HYBRID 尾部缺少柔性感；当前 2.45 Hz 在55%动力下每20 ms最大目标变化约9.783°，几乎贴住10°渐变上限。
- 最终决策：仅将 HYBRID 尾舵频率从2.45 Hz降至2.0 Hz，BIONIC保持1.7 Hz；两项频率集中到 `sdk_config.h`。HYBRID 的58°基准摆幅、55%网页动力、72°相位参数、10°渐变上限、螺旋桨动力和公式均不变。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.cpp`。
- 验证方式和结果：静态计算确认 HYBRID 前舵在55%动力下实际摆幅31.9°，每20 ms最大目标变化降至约7.996°；ESP-IDF 5.3.2增量构建、链接和应用分区尺寸检查通过，应用镜像937328字节，OTA分区剩余72%。生成 `RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260814-125606.bin`，SHA-256 为 `a9086685b905d80243a1077ceeb1c617cbf1074c4acf2f558275605c3a3e5048`。
- 已知限制：尚未烧录观察实际柔性和推进效率；当前公式仍使用 `phase + 72°`，若实机后舵表现为抢先而非跟随，下一步应优先验证物理方向后再调整相位参数。
- 待确认事项：实机对比2.45 Hz与2.0 Hz的尾部柔性、直行推力和偏航；若仍不自然，确认前后舵运动先后关系，再决定是否将HYBRID相位参数从0.20周期改为0.80周期。

## 2026-08-14 IDLE 启动模式与 RGB 状态区分

- 修改原因：用户要求连接成功后保持 IDLE，未下达模式指令时不得默认选择推进模式；同时要求新增启动、启动失败和连接颜色，但保留推进模式颜色。
- 最终决策：新增 `DriveMode::Idle=0` 并作为上电默认模式。连接事件只更新链路状态，不选择推进模式；第一次网页 MODE 指令执行 IDLE→BIONIC，随后按 BIONIC→PROP→HYBRID→BIONIC 循环。IDLE 下人工及 Partner 非停止运动命令返回拒绝，停止、俯仰、水泵仍可用；视觉控制、视觉 PID、Demo 和 Partner 模式命令均属于显式选择，继续进入各自指定模式。此前“连接颜色长期替代推进模式颜色”的方向被用户否决。
- RGB 映射：应用启动中为蓝色；任一核心启动环节失败为红色并保持；启动成功且 IDLE 未连接为蓝色、IDLE 已连接为青色；BIONIC 为绿色、PROP 为红色、HYBRID 为紫色；失联保护为橙色且优先于模式色。
- 涉及文件：`RoboFishPartnerDriverSDK/src/control/MotionController.h/.cpp`、`src/app/RoboFishRuntime.h/.cpp`、`RoboFishPartnerDriverSDK/README.md`。
- 验证方式和结果：ESP-IDF 5.3.2 增量构建、链接和应用分区尺寸检查通过；应用镜像 937296 字节，OTA 分区剩余 72%。生成 `RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260814-123343.bin`，SHA-256 为 `5ca3c92d23fca63cc66eb954361d3249b9a38271f076dd107ae7c8671a30bbec`。
- 已知限制：尚未烧录验证灯色和网页拒绝反馈；若 RGB 灯条自身初始化失败，则硬件无法显示启动失败红色。编译通过不能证明实际 GRB 色序与肉眼颜色正确。
- 待确认事项：实机依次验证蓝色启动、青色已连接 IDLE、首次 MODE 后绿色 BIONIC、后续红色 PROP/紫色 HYBRID、断联保护橙色，以及 IDLE 下运动按钮不会驱动执行器。

## 2026-08-14 网页人工动力按推进模式分流

- 修改原因：统一降到 55% 能避免高频尾舵 CPG 超过渐变跟踪能力，但用户实测纯 PROP 直行推力明显不足，要求恢复螺旋桨动力。
- 最终决策：网页 `servo_forward`、`servo_backward`、`servo_left`、`servo_right` 读取当前模式并显式传入动力；纯 PROP 使用 100%，BIONIC/HYBRID 使用 55%，转向力度仍为 100%。频率、相位、摆幅基值、油门渐变、停止渐变和运动公式不变。此前“所有推进模式统一使用 55%”的方向不再采用；视觉与 Partner 显式动力参数不受网页默认值影响。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.h`、`src/web/ProductWebService.cpp`。
- 验证方式和结果：ESP-IDF 5.3.2 增量构建、链接和应用分区尺寸检查通过；应用镜像 937328 字节，OTA 分区剩余 72%。生成 `RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260814-124556.bin`，SHA-256 为 `f7a450e4c2149a95577a0a2b5dc32061b2ba9a29a859d4bcf358cffbd9a0ec00`。
- 已知限制：尚未烧录和入水验证。PROP 的 100% 命令仍按现有每 20 ms 最大 3°的油门渐变输出，不是瞬时满油；左右转尾部偏置和物理方向问题未在本次修改。
- 待确认事项：架空确认 PROP 渐变升速和停止正常，再入水比较 100% 直行推力；BIONIC/HYBRID 继续验证 55% 下的尾舵波形、转向方向和推进效率。

## 2026-08-13 视觉端切换 MCU 双 PID 协议

- 修改原因：用户要求试运行 MCU X/Y PID，视觉端需要停止本地 PID 并发送标定误差。
- 最终决策：`VISION/yolo(PID).py` 保留 YOLO、标定、轨迹和 10 Hz 最新值队列；移除运行期电脑转向 PID，改发 `GET /vision_pid?seq=&x_error=&y_error=`，其中 X=`cross_m`、Y=`dist_m`，并校验 JSON ACK 的 `ok` 与回显序号。
- 安全行为：同步停止前先清空待发送队列，防止 stop 后旧 PID 命令出队；相机/目标丢失、清轨迹和退出仍发送 `servo_stop`。
- 验证方式和结果：视觉虚拟环境 Python 语法检查通过；本地模拟 MCU 验证实际请求为 `/vision_pid?seq=1&x_error=0.1250&y_error=1.7500` 且 ACK 校验通过。
- 已知限制：当前 `192.168.4.1/get_ip` 和 `/vision_pid` 均超时，未连接到可响应的机器鱼，因此没有进行实机推进测试。

## 2026-08-13 MCU 双轴视觉 PID

- 修改原因：用户确认执行器极性正确，并要求 X/Y 两个视觉 PID 全部放到单片机侧。
- 最终决策：新增 `GET /vision_pid?seq=&x_error=&y_error=`；`x_error` 是横向误差（米，正值保持现有左转极性），`y_error` 是到终点的剩余距离（米）。静态 `VisionControl` task 只在收到新视觉样本时计算 X 转向 PID 与 Y 推进 PID，输出分别限制为 -70..70 和 0..70；原 `MotionControl` task 继续独占硬件输出。
- 参数与保护：X PID 为 150/8/35，Y PID 初值为 180/2/10，积分限制 0.5，死区 0.015 m；保留序号防倒序和 500 ms 超时停车，旧 `/vision_control` 兼容接口仍保留。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.h/.cpp`、`src/web/ProductWebService.h/.cpp`。
- 验证方式和结果：ESP-IDF 5.3.2 编译、链接和应用分区尺寸检查通过；应用镜像 934592 字节，OTA 分区剩余 72%。
- 已知限制：尚未在真实水池验证 PID 增益和动态响应；当前电脑端仍发送旧离散动作，尚未改为调用 `/vision_pid`。
- 待确认事项：完成电脑端协议修改后，先架空螺旋桨确认正横向误差左转，再以低功率短轨迹整定 Y PID。
- 生成文件：`RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260813-161910.bin`。

## 2026-08-13 视觉 PID 鱼端控制接口

- 修改原因：用户要求电脑视觉与机器鱼形成闭环，先完成单片机侧接口，暂不修改视觉 Python。
- 最终决策：新增 `GET /vision_control?seq=&power=&steering=`，复用现有 `MotionController`，自动使用 PROP 模式；`power` 限制 0..70，`steering` 限制 -70..70，序号拒绝重复或倒序命令。视觉租约和 500 ms 超时停车由独立静态 `VisionControl` FreeRTOS task 负责，ESC 输出仍由原 `MotionControl` task 独占。
- 安全行为：视觉指令 500 ms 未刷新时自动停止推进并回中转向；网页人工控制、Partner、OTA 和现有 `/control` 接口保持原有路径。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`src/control/MotionController.h/.cpp`、`src/web/ProductWebService.h/.cpp`。
- 验证方式和结果：ESP-IDF 5.3.2 目标构建成功，应用镜像 926480 字节，OTA 分区剩余 72%；尚未在真实鱼上验证 ESC 左右方向、网络延迟和超时停车。
- 生成文件：`RoboFishPartnerDriverSDK/build/RoboFishPartnerDriverSDK-OTA-20260813-161047.bin`。

## 2026-08-13 OTA 实机验证标记

- 修改原因：用户要求执行一次可肉眼确认的无线 OTA 测试。
- 最终决策：仅在遥控网页顶部增加醒目的 `OTA TEST SUCCESS · 2026-08-13` 绿色标记，不改运动和保护逻辑。
- 涉及文件：`reverse_analysis/web_assets/html_01_0x0000100b.html`。
- 验证方式和结果：目标固件编译与无线 OTA 结果在本次测试结束后确认。
- 已知限制：旧 Service Worker 可能缓存 `/gpio`；验证时使用带查询参数的新地址绕过旧缓存。
- 待确认事项：OTA 重启后访问 `/gpio?ota-test=20260813` 确认标记和相机图传。

## 2026-08-13 无 PSRAM 摄像头启动修复

- 修改原因：实机启动日志确认 OV3660 已识别，但无 PSRAM 时以 UXGA 初始化会申请 384000 字节内部 RAM 帧缓冲并失败，导致端口 81 图传服务未启动。
- 最终决策：保留有 PSRAM 时的 UXGA 初始化；无 PSRAM 时直接以网页实际使用的 SVGA 初始化，避免无意义的大帧缓冲申请。
- 涉及文件：`RoboFishPartnerDriverSDK/src/camera/CameraService.cpp`。
- 验证方式和结果：实机日志已确认原故障为 `frame buffer malloc failed`；修复后完成 ESP-IDF 目标构建，实机图传结果待重新烧录确认。
- 已知限制：当前工程仍按原镜像配置关闭 PSRAM，仅使用单个内部 DRAM 帧缓冲。
- 待确认事项：重新有线烧录后确认 `Camera ready`、端口 81 MJPEG 和持续图传。

## 2026-08-04 SDK 内部重构

- 修改原因：按嵌入式固件工作流整理可编译 SDK，保持行为不变，降低执行器互斥锁路径和 OTA 任务参数维护风险。
- 最终决策：不修改公共 API、GPIO、执行器限幅、超时策略、OTA 认证策略、任务周期、优先级、栈大小或绑定核心；仅做内部结构重构。
- 涉及文件：`RoboFishPartnerDriverSDK/RoboFishDriver.h`、`RoboFishPartnerDriverSDK/RoboFishDriver.cpp`、`RoboFishPartnerDriverSDK/OtaService.cpp`、`RoboFishPartnerDriverSDK/sdk_config.h`。
- 验证方式和结果：已静态检查 SDK 源码中 FreeRTOS 等待路径和 OTA 任务参数引用；本机未安装 Arduino CLI、ESP32 Arduino 包或 Xtensa 编译器，未完成目标板编译。
- 已知限制：未进行硬件上电、执行器输出、Web OTA 上传或栈水位测量。
- 待确认事项：在 Arduino ESP32 Core 3.3.7 与 XIAO ESP32-S3 环境执行一次编译，并在硬件上确认 OTA 上传时执行器停机和锁定行为。

## 2026-08-04 SDK 框架分层

- 修改原因：用户要求进一步做“结构、框架”级重构，明确 Arduino 入口、app、config、device、service 的职责边界。
- 最终决策：实现文件迁入 `src/` 分层目录；根目录保留 `RoboFishDriver.h`、`OtaService.h`、`sdk_config.h` 兼容转发头；新增 `RoboFishRuntime` 负责启动编排、周期 update 和 OTA 停机回调。
- 涉及文件：`RoboFishPartnerDriverSDK.ino`、`ARCHITECTURE.md`、`README.md`、`src/app/RoboFishRuntime.h`、`src/app/RoboFishRuntime.cpp`、`src/config/sdk_config.h`、`src/device/RoboFishDriver.*`、`src/service/OtaService.*`、根目录兼容头。
- 验证方式和结果：已静态检查源码结构、include 路径、FreeRTOS 等待路径和 OTA 任务参数引用；未发现根目录残留实现 `.cpp`。
- 已知限制：本机仍未安装 Arduino CLI、ESP32 Arduino 包或 Xtensa 编译器，未完成目标板编译、硬件执行器输出、OTA 上传或栈水位测量。
- 待确认事项：在 Arduino ESP32 Core 3.3.7 与 XIAO ESP32-S3 环境确认 `src/` 递归编译、串口启动、Web OTA 和执行器停机锁定行为。

## 2026-08-04 Device 锁边界修正

- 修改原因：用户指出 device 包含灯条、两个尾舵、一个侧舵和一种螺旋桨，不应把所有执行器放在一个全局 mutex 下串行化。
- 最终决策：否决“全执行器单锁”方向；`RoboFishDriver` 改为兼容门面和 OTA 安全门，尾舵、侧舵、推进和水泵分别由独立 device owner 管理，每个 device 使用自己的静态互斥锁；OTA 的 `otaLocked` 只作为全局命令安全门。
- 涉及文件：`src/device/DeviceMutex.h`、`src/device/TailServos.*`、`src/device/SideRudder.*`、`src/device/Propulsion.*`、`src/device/PumpOutput.*`、`src/device/RoboFishDriver.*`、`ARCHITECTURE.md`、`README.md`、根 `README.md`。
- 验证方式和结果：已静态检查 device 目录无旧 `ScopedLock`/全局 `mutex_` 残留；未发现 `portMAX_DELAY` 或无限互斥等待；各等待仍使用有限超时或 0 tick 轮询。
- 已知限制：灯条未接入 SDK，当前缺少已确认的灯条 GPIO、协议、像素数和电气信息；本机仍未安装 Arduino CLI、ESP32 Arduino 包或 Xtensa 编译器，未完成目标板编译和硬件验证。
- 待确认事项：确认灯条硬件映射后新增 `LightStrip` device；在 Arduino ESP32 Core 3.3.7 与 XIAO ESP32-S3 环境确认 `src/device` 新增文件被递归编译，并验证 OTA 停机、舵机、推进和水泵输出。

## 2026-08-12 VS Code 编译与无线 OTA

- 修改原因：提供可重复的时间命名构建、电脑连接鱼热点、认证与 Web OTA 反馈入口，并防止误传非应用镜像。
- 最终决策：默认构建固定复用 XIAO ESP32-S3、Arduino ESP32 Core 3.3.7、8 MB Flash、`default_8MB` 双 OTA 分区和关闭 PSRAM；输出 `build/RoboFishPartnerDriverSDK-OTA-yyyyMMdd-HHmmss.bin`。无线拓扑固定为鱼 AP、电脑 STA，OTA 默认目标为鱼 AP 地址 `192.168.4.1`；工具只接受 `/status` 返回 `sdk=1.0.0` 且 AP 地址有效的目标。此前“电脑 AP、鱼 STA”的方向已被用户否决，不再生成或部署鱼 STA 凭据。
- 涉及文件：`.vscode/tasks.json`、`.vscode/launch.json`、`.vscode/settings.json`、`.vscode/robofish-ota.ps1`、`.gitignore`、`README.md`、`RoboFishPartnerDriverSDK/README.md`、`src/config/sdk_config.h`。
- 验证方式和结果：已核对固件使用 `WiFi.softAP` 创建鱼 AP；HTTP 路由为 Basic Auth `POST /update`、multipart 字段 `firmware`。已静态检查隐藏密码输入、目标状态校验、时间命名筛选、`0x330000` 大小门禁、上传前确认、设备反馈、资源释放和无自动重试；VS Code JSON 可解析，任务依赖完整，PowerShell AST 无语法错误，`git diff --check` 通过，FQBN 与已有 `build.options.json` 完全一致。
- 已知限制：本机未安装 Arduino CLI，未执行实际编译；未连接机器鱼热点、输入真实密码或上传固件。自动 Wi-Fi 连接使用 Windows `netsh`，并会为当前 Windows 用户保存机器鱼 Wi-Fi 配置。
- 待确认事项：安装构建环境后完成一次实际编译；在硬件上验证 AP 连接、认证失败反馈、上传成功反馈和重启后重新上线。

## 2026-08-12 VS Code 运行和调试入口

- 修改原因：用户要求从 VS Code“运行和调试”面板直接启动现有编译、联网和 OTA 流程。
- 最终决策：新增 `launch.json`，使用 `node-terminal` 启动器和 `preLaunchTask` 复用现有任务；不复制构建参数或密码，不宣称提供 ESP32 JTAG 断点调试。
- 涉及文件：`.vscode/launch.json`、`.gitignore`、`README.md`。
- 验证方式和结果：`launch.json` 可解析，6 个配置名称唯一，启动类型和请求类型有效，所有 `preLaunchTask` 均能在 `tasks.json` 中找到；`git diff --check` 通过。
- 已知限制：实际启动仍依赖 Arduino CLI/Arduino IDE 2.x、Windows Wi-Fi、机器鱼 AP 和硬件在线。
- 待确认事项：在 VS Code 中刷新工作区后确认运行配置列表可见，并完成一次硬件流程验证。


## 2026-08-12 完整固件摄像头与网页资源恢复

- 修改原因：用户要求依据 `CameraTest.ino.bin` 的静态逆向证据，在纯 ESP-IDF 工程中恢复摄像头接口、MJPEG 图传和原驾驶/Partner 网页资源。
- 最终决策：新增 `CameraService`，固定采用已确认的 XIAO ESP32-S3 Sense 摄像头引脚、JPEG、12 MHz XCLK 和 GPIO21 补光灯映射；主 HTTP 服务提供 `/status`、`/control`、`/capture`、`/bmp`，主端口加 1 的独立服务提供 `/stream`。`/control` 未识别参数通过回调交给运动应用层，保持相机和机器鱼命令共用原 URI。新增 `WebAssets`，用 `EMBED_TXTFILES` 直接链接四个恢复资源并提供 `/gpio`、`/partner`、`/manifest.json`、`/sw.js`，不改写前端业务协议。
- 涉及文件：`RoboFishPartnerDriverSDK/src/camera/CameraService.h`、`RoboFishPartnerDriverSDK/src/camera/CameraService.cpp`、`RoboFishPartnerDriverSDK/src/web/WebAssets.h`、`RoboFishPartnerDriverSDK/src/web/WebAssets.cpp`、`RoboFishPartnerDriverSDK/main/CMakeLists.txt`。
- 验证方式和结果：确认本机 ESP-IDF 5.3.2、工作区和常见 Arduino/PlatformIO 目录均没有 `esp32-camera`；缺组件路径会明确返回 `ESP_ERR_NOT_SUPPORTED`/HTTP 503。已在该真实缺组件环境完成 ESP32-S3 全量构建，并用 `xtensa-esp32s3-elf-nm` 核对四个嵌入资源的 `_binary_*_start/end` 符号与源码引用一致；`git diff --check` 通过。
- 已知限制：ESP-IDF 5.3.2 不自带 `esp32-camera`，当前构建只有可用的降级接口，未包含真实图像驱动；工程还关闭 PSRAM，后续启用相机时会使用单 DRAM 帧缓冲。恢复网页仍调用尚待整合的 Wi-Fi、运动、Partner 和旧 `/ota_upload` 协议；本条修改没有替换现有 OTA 服务。
- 待确认事项：将官方 `esp32-camera` 放入本地 `components/esp32-camera` 并登记依赖后，重新编译真实相机分支；在 XIAO ESP32-S3 Sense 上验证传感器型号、帧尺寸枚举、图像方向、GPIO21 亮度、DRAM 帧缓冲稳定性、端口 81 持续图传和并发控制响应。

## 2026-08-13 官方摄像头组件依赖闭环

- 修改原因：原工程只具备缺组件时的 HTTP 503 降级分支，尚未用真实 `esp32-camera` 编译相机初始化、抓拍、BMP 和 MJPEG 代码。
- 最终决策：在 `main/idf_component.yml` 声明官方 `espressif/esp32-camera ^2.0.0`，由 Component Manager 锁定 `esp32-camera 2.1.7`、传递依赖 `esp_jpeg 1.3.1` 和 ESP-IDF 5.3.2；提交清单与 `dependencies.lock`，忽略生成的 `managed_components/` 和 `build/*-idf/`。保持原固件证据对应的 PSRAM disabled、8 MB Flash、QIO 80 MHz，不为相机强行启用 PSRAM。
- 涉及文件：`RoboFishPartnerDriverSDK/main/idf_component.yml`、`RoboFishPartnerDriverSDK/dependencies.lock`、`RoboFishPartnerDriverSDK/components/README.md`、`.gitignore`。
- 验证方式和结果：在独立 `build/camera-idf` 使用 ESP-IDF 5.3.2 全量构建成功；编译定义包含 `RF_HAS_ESP32_CAMERA=1`，`CameraService.cpp` 对象存在 `esp_camera_init`、`esp_camera_fb_get`、`frame2bmp` 未定义引用，最终 ELF 链接出 `esp_camera_init`，证明真实分支参与编译和链接。应用镜像为 923808 字节，SHA-256 为 `a603272ce2c7efdbdf1b77dfbf6957a26e9bfcd84b669f5fdf6f65e1232a1b37`，OTA 分区剩余约 72%；构建日志未发现 warning 或 error。
- 已知限制：本次只验证源码、依赖解析和目标链接，未烧录硬件；PSRAM 关闭时相机采用单个 DRAM 帧缓冲，持续图传的帧率和内存余量尚无实机数据。ESP-IDF 5.3.2 在选择 QIO 时将镜像头模式字符串映射为 `dio`，但有效 Kconfig 选择仍为 `CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`。
- 待确认事项：在 XIAO ESP32-S3 Sense 上验证摄像头型号、图像方向、帧尺寸、GPIO21 补光灯、DRAM 稳定性、端口 81 MJPEG 与并发遥控；不得把编译通过当作整机图传已经实测通过。

## 2026-08-13 Partner HTTP 接口恢复

- 修改原因：按完整固件静态证据恢复 Partner Lab 脚本 API，并为网络 owner 提供受控的 `/get_ip` 与 `/save_wifi` 回调边界。
- 最终决策：新增无独立 Task、无动态分配的 `HttpService`；注册 `GET/POST /partner_script`、`POST /partner_run`、`POST /partner_stop`、`GET /partner_status`，可选注册 `GET /get_ip` 和 `GET /save_wifi`。脚本正文严格限制 1～3500 字节并循环接收；启动失败返回 409；保存成功、停止成功、状态 JSON 和 Wi-Fi 重启正文保持恢复证据中的协议。网络状态、凭据持久化和重启仍由网络 owner 回调完成。
- 涉及文件：`RoboFishPartnerDriverSDK/src/partner/PartnerHttpService.h`、`RoboFishPartnerDriverSDK/src/partner/PartnerHttpService.cpp`。
- 验证方式和结果：复用 ESP-IDF 5.3.2 `compile_commands.json` 的 Xtensa ESP32-S3 C++ 参数独立编译通过，生成 128320 字节对象文件；`nm -C` 确认路由与七个处理路径符号存在；`git diff --check` 通过。
- 已知限制：本条只完成可注册服务，未改 Runtime、OTA 服务或 CMake；尚未在真实 HTTP 服务、NVS 和硬件网络上进行请求测试。
- 待确认事项：主线接入时必须保证服务实例生命周期长于 HTTP server、提供网络回调，并为新增路由预留足够 `max_uri_handlers`。

## 2026-08-13 视觉坐标标定加固

- 修改原因：手动标定原先不校验点击顺序、区域或矩阵，重新标定期间仍保留旧矩阵；自动标定只统计四个 ArUco 同时出现的帧数，不能证明角点稳定。
- 最终决策：保持水池映射 `3.14 m × 1.6 m` 和 TL→TR→BR→BL 坐标方向不变；新增凸性、顺序、最小面积、有限值和矩阵退化检查；显示手动点击编号；自动标定要求四个角点连续 15 帧最大位移不超过 2 像素；按 `T` 重新标定时立即发送停止并清除旧矩阵与位置滤波。Windows WIN32 HighGUI 的鼠标回调直接使用原图坐标，禁止再次按窗口尺寸缩放，避免二次映射偏移。
- 涉及文件：`VISION/yolo(PID).py`。
- 验证方式和结果：Python 语法检查通过；合规四点映射为 `(0,0)`、`(3.14,0)`、`(3.14,1.6)`、`(0,1.6)`；交叉顺序和过小区域测试均被拒绝。
- 已知限制：未在真实水池画面验证 ArUco 稳定阈值；物理尺寸必须与实际有效区域一致。
- 待确认事项：实机标定后核对中心点约为 `(1.57,0.8)`；架空执行器确认画面横向误差与机器鱼左右转符号一致。

## 2026-08-13 本机绘图 PROP 轨迹追踪

- 修改原因：用户要求直接在电脑 YOLO 画面绘制轨迹，并暂时只使用螺旋桨推进系统执行 PID 追踪。
- 最终决策：左键拖动创建单条像素轨迹，右键清除，Enter 启动，Space 停止，Q 或窗口关闭时停止后退出；启动前要求完成标定、YOLO 已识别鱼、轨迹起点距鱼不超过 0.40 m。启动时根据固件 `/control?var=mode&val=1` 的响应循环确认进入 `PROP`，之后只发送 `servo_forward`、`servo_left`、`servo_right`、`servo_stop`；PROP 模式保持尾舵中位，只驱动 ESC 油门与 ESC 转向。轨迹误差按 10 Hz 输入现有 PID，到终点 0.12 m 内停止；摄像头中断或 YOLO 目标丢失时停止并退出/取消追踪。
- 涉及文件：`VISION/yolo(PID).py`。
- 验证方式和结果：Python 语法检查通过；直线路径测试确认在线横向误差为 0、路径两侧误差符号相反、末段剩余距离正确；静态核对固件 `ProductWebService`、`MotionController`，确认 PROP 模式仅驱动 ESC 推进与转向，尾舵保持中心。
- 已知限制：固件现有 `/control` 动作接口只消费 `var`，不接收电脑计算的 `power/strength`；当前 PID 只能决定直行、左转、右转三种动作，固件按默认 100% 动作参数执行。摄像头与机器鱼未连接，未做实机闭环验证。
- 待确认事项：架空执行器验证 PROP 模式与左右转方向；入水后从低风险短轨迹确认路径横向误差符号和终点停止距离。

## 2026-08-14 PROP 左右转向力度增强

- 修改原因：用户确认将螺旋桨模式最大转向偏角从 12° 增加到 24°，增强左右转向力度。
- 最终决策：新增配置 `RF_PROP_TURN_MAX_OFFSET_DEG=24` 并由 `MotionController` 消费；PROP 满转目标由中位 90° 的 ±12° 扩大为 ±24°，视觉 PID 在现有 70% 限幅下最大约 ±17°。HYBRID 的 8°、推进油门、20 ms 控制周期、转向每周期 4° 的渐变速度及保护逻辑保持不变。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`RoboFishPartnerDriverSDK/src/control/MotionController.cpp`。
- 验证方式和结果：ESP-IDF 5.3.2 全量构建成功；应用镜像 934592 字节，SHA-256 为 `f712a9b0846a359fd0d93d67ea71d5492d131faca3ff334a6bae027649e085d3`，OTA 分区剩余 72%。满转目标 66°/114° 位于现有 40°～136° 软件限位内。
- 已知限制：尚未烧录并进行水中转向测试；编译和数值边界检查不能证明实际转向角速度与水动力效果。
- 待确认事项：首次测试应架空螺旋桨并确认左右极性，再在低油门短时入水测试中确认转弯半径、执行器机械余量和电流。

## 2026-08-14 双尾鳍相位抵消调整

- 修改原因：用户观察到前后尾鳍舵机动力抵消，确认按原模式比例降低相位差。
- 最终决策：BIONIC 后舵机相位滞后由 0.35 周期（126°）降至 0.16666667 周期（60°），HYBRID 由 0.42 周期（约151°）按相同比例降至 0.20 周期（72°），保留两模式相位差 1:1.2 的比例。频率、前后摆幅 1:0.9、转向偏置、渐变速度和保护逻辑不变；相位参数集中到 `sdk_config.h`。
- 涉及文件：`RoboFishPartnerDriverSDK/src/config/sdk_config.h`、`RoboFishPartnerDriverSDK/src/control/MotionController.cpp`。
- 验证方式和结果：静态计算确认相位分别为约 60°、72°，比例约 1.2；随后与网页 STOP 修改一并通过 ESP-IDF 5.3.2 全量构建，应用镜像 937072 字节，OTA 分区剩余 72%。
- 已知限制：尚未确认两只舵机安装方向是否一致，也未验证尾端实际合成幅度、推进效率、电流和机械干涉；静态计算不能替代实机水动力验证。
- 待确认事项：编译烧录前先低幅度单独驱动两只舵机确认物理正方向；之后架空低动力观察波形，再入水比较前进速度、偏航和电流。

## 2026-08-14 网页 STOP 可靠停止

- 修改原因：网页用本地 `lastAction` 对 STOP 去抖，页面状态与设备不一致或前次请求失败时会直接跳过停止；运动请求失败也没有可见反馈，鼠标移出按钮后松开可能漏发。
- 最终决策：中央 STOP 改为按下即发送且永不受本地状态、节流或去抖阻止；发送前取消尚未完成的前后左右请求，并在鼠标松开、触摸取消、窗口失焦、页面隐藏或离开时走同一停止路径。中央按钮仅在 HTTP 200 后提示成功，失败显示红色错误。后端 `servo_stop` 继续通过原目标值和 `slew` 渐变归中，同时取消视觉 PID、Demo、Partner 并关闭水泵，胸鳍角度不变；用户否决了执行器瞬时跳回中位的硬停止方案。
- 涉及文件：`reverse_analysis/web_assets/html_01_0x0000100b.html`、`RoboFishPartnerDriverSDK/src/control/MotionController.cpp`。
- 验证方式和结果：Node.js 解析嵌入脚本成功；ESP-IDF 5.3.2 全量构建成功并确认网页资源重新生成、参与链接。应用镜像 937072 字节，SHA-256 为 `169a0c701ca1f37545f114ce26907e69bf56863414a84127c1554e5a5fffb6fc`，OTA 分区剩余 72%。
- 已知限制：尚未在手机浏览器、实际 Wi-Fi 丢包和执行器上验证；取消浏览器请求不能撤销已经到达后端的旧命令，但随后独立 STOP 请求会覆盖设备状态。
- 待确认事项：实机验证页面刷新后直接按 STOP、断续网络、按钮外松手、切后台及重复 STOP；观察尾鳍、油门和转向是否按原渐变速度归中且水泵关闭。
