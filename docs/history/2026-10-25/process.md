# 2026-10-25 开发记录

来源：归档前根目录 `process.md`。以下正文按原文保留；历史决策只适用于各自版本和试验条件。

<!-- original-process-entry: 054 sha256: e2588789a222d6193bbb05476c3eedb3e5f0e13d603dfb15a4c2e28fcecce1f2 -->
## 2026-10-25：相机自动检测、遥控响应与断线显示

范围／原因：恢复会话后继续处理用户要求的自动检测相机、画面卡顿、按钮禁用和LoRa迟钝。用户最终明确当前未接鱼，停止后续测试并直接交付。新上位机修订为workstation-25-responsive-control，重新打开启动器加载。

决策／文件：vision/camera_sources.py及upper/vision_service.py提供独立进程、25秒超时的实际读帧检测，网页改为自动选择可用输入。observer_runtime.py保留相机默认尺寸并请求MJPEG／30fps，取消强制1280×720；分辨率变化沿用原重标定处理。robot.js仅禁用不存在的板载相机，保留外部预览按钮；LoRa可用时Wi-Fi连接请求不再占遥控队列，配置提交后不等待遥测才解锁。lora_link.py普通帧间隔500→250ms，仍只保留最新待发运动目标，STOP优先。串口打开、Wi-Fi在线和鱼端近期收包分别显示，缓存身份不再冒充在线。不新增鱼端LoRa应答；现有S3航向算法未发现按航向偏差大小强制停机的分支，因此未删除其他停止条件或改固件。

已完成验证：15项LoRa独立控制、4项双链路及7项视觉服务测试通过；真实浏览器以模拟串口／鱼端验证未完成的Wi-Fi连接不阻塞LoRa、断线徽标、外部画面按钮和相机自动选择。指定视频／YOLO网页回归通过透视1000×500、原图1384×776、录像、采样17行和CSV／JSON下载，零浏览器错误、零实物控制。实际扫描识别相机0、640×480。此前强制1280×720路径实测回落到YUY2、约5fps，故最终取消强制分辨率；该最终采集调整按用户要求不再追加测试。

限制／待确认：250ms为软件发送间隔，未验证真实LoRa空口接收率、运动响应或水中效果；请求30fps不保证摄像头达到30fps。没有打开真实串口、发送实鱼指令、烧录、改网络或重启用户后台。普通相机默认尺寸变化需重新标定；用户重新运行upper/start.cmd即可加载新版服务。

<!-- original-process-entry: 055 sha256: 282ced7363809696570433c707fbfed139e8b94777a6b8e675d1fe7605fe5f6c -->
## 2026-10-25：统一视觉跟踪点 P

范围／原因：用户要求把机器鱼视为一个点，方便后续循迹。原观测位置在身体轮廓中心和 YOLO 框中心之间切换。本轮统一使用整鱼检测框中心，短时漏检沿用已有光流更新的同一点；经原场地 H 转换后供轨迹、速度、转弯拟合、LOS 预览、网页与 CSV 共用。身体轮廓继续只负责独立测角，未更改模型、跟踪时限、S3 固件或运动控制边界。

实现／接口：默认显示红色 P 点，调试开关保留框与特征点；冻结预览保存对应帧的点。position 保留 x_m/y_m/source，增加像素坐标、valid、track_id 和 sample_time；valid 表示米制坐标可用。未完成场地标定时保留像素点；缺测及原有图像过期条件下不输出有效米制点。涉及 vision/observer_runtime.py、upper/vision.js、现有 test_heading_observer.py 与 docs/vision-control.md。

验证／边界：8 项独立观测回归通过，包括中心来源一致、轨迹／引导／CSV 共点、冻结预览、缺测、过期和未标定；Python／JS 语法及 git diff --check 通过。实际 best.pt 与用户既有录像连续24帧均输出像素点，预览及逐帧记录在 build/host-vision/point-reference。该录像检查未套用实物场地标定，米制换算由合成标定回归覆盖；检测框中心不等同于物理重心，水中点位稳定性尚待实测。本轮未启动真实相机、串口、服务或实鱼运动，未烧录。


<!-- original-process-entry: 056 sha256: 91beb061ef71af678a438e2384d103d69cf82e60236d524f2e93db764e6016a5 -->
## 2026-10-25：Wi-Fi画线自动循迹、调试终端与角度环调整

范围／授权：用户明确要求数据页持续输出PC观测和鱼端回传，框选及鱼头操作还原完整全局画面，P从5降至2.5，删除0.36m/s速度输入上限，提高角度环转向力度，增加小死区，并明确选择接通画线自动循迹、优先Wi-Fi。此授权替代此前“视觉仅观测”的限制；默认视觉启动仍不发送运动，必须明确点击AUTO_START。按用户要求不做运行期／硬件测试，不发送实鱼指令、不烧录。

实现／文件：upper/workstation.js与index.html新增1000行PC／FISH终端，复用现有轮询，显示实际来源、缺失值、左右正号、PID分项、Wi-Fi控制往返及六路指令，支持暂停、筛选、清空和TXT导出。observer_runtime、vision.js把框选与鱼头标定保持在完整全局图，冻结帧附选择ID，鼠标事件按序提交并使用实际显示坐标；选鱼用冻结帧时标，不把旧图当成新采集。SPEED前后端均删除0.36上限，保留有限非负校验。

自动循迹／控制边界：复用PathGuidance、VisualSpeedLoop及WorkstationComm；PC使用实时相机P点位置和对齐IMU方向计算LOS目标，速度PID给推进百分比，最高10Hz发送最新目标，S3执行本地yaw PID。AUTO_START先全停并关闭本机LoRa发送端，连接Wi-Fi、同步零位，等待原3秒LoRa主控健康窗口释放。prepare与后续视觉目标明确via_wifi，不回退LoRa；初次准备再确认一次Wi-Fi STOP，原会话／序号和人工接管代次仍生效。终点、显式停止、退出、重标定或重新画线退出自动控制；关闭画面预览和停止采样不停止自动循迹。完全失联时全停可能未到达，原鱼端无运动超时语义保持，未增加新的鱼端保护／重连状态机。

固件／参数：2.7.1-trace-turn默认yaw Kp/Ki/Kd=2.5/0.02/0.1，±1°死区内清I/D及转向纠偏，保留航向目标和推进。闭环左右桨差动与尾舵转向分配增益1.5；去掉闭环下随转向量平方额外削减推进的处理，饱和、机械边界、原极性和周期保留。Wi-Fi遥测新增deadband、actuator_gain及turn_pct，旧固件未提供时不填造。

构建／验证：ESP-IDF5.3.2编译成功，BIN 910096字节，SHA256 9191e778f469ed2ad5323a42145bcb46350c60b54f72438dc752f3c88d670e4b；命名交付artifacts/RobotFish-ESP32S3-2.7.1-trace-turn.bin。只完成语法与构建检查；本次自动循迹、转向增益、死区和终端的运行效果未测试，不借用之前的网页回归作为本轮通过证据。上位机workstation-26-trace-turn，重新运行upper/start.cmd；新默认值和死区需用户手动升级BIN。未修改电脑网络、打开串口或烧录。

交付顺序：用户要求先交付再补全文档，已先提供2.7.1 BIN和上位机启动入口，再补齐README、循迹操作流程、调试终端字段、LoRa释放／Wi-Fi控制权、全停确认和本轮未运行测试的边界。文档补全期间未再修改交付代码或固件。

<!-- original-process-entry: 057 sha256: 04235ceae37fdaa7c4fb5470ff1094fc60fb31fb80795640a740aff54c83ef63 -->
## 2026-10-25：删除独立循迹预览

用户要求删除无用的循迹预览。已删除网页“启动／停止循迹预览”、旧 START/STOP 动作、运行分支及 preview_only 状态字段，速度与绘线提示统一指向自动循迹。仅保留 AUTO_START/AUTO_STOP 执行路线；相机画面开关、路径绘制、标定、独立角度、采样、录像和原自动循迹控制语义保留。涉及 observer_runtime、web_runtime、vision_service、网页、对应测试与现有说明；未修改固件。

验证：16项离线观测／视觉服务测试全部通过，覆盖旧动作拒绝、自动航向／速度目标仍下发、停止及丢失位置请求全停、点位／标定／CSV／录像；Python与JS语法、git diff --check通过。浏览器回归脚本已同步移除旧入口断言，本轮按用户要求加快交付，未执行完整浏览器录像回归、实机相机或运动测试。重新运行 upper/start.cmd --restart 后加载更新。

<!-- original-process-entry: 058 sha256: e202b519ed207ed15f0d0eb4118d3bdef312e4b69f630ce6351d335c3e5c89c4 -->
## 2026-10-25：持续循迹修复、界面减负与日志整定

用户要求鱼头标定保留当前视角，排查持续发送中断，提升整个上位机响应，并根据09:51:32导出的日志和轨迹图优化参数；明确要求先交付再补全文档。08:54前后uptime回退由用户确认为手动断电／重启，不归因为硬件故障。旧终端只在数据页记录，08:57日志缺少自动运行段；后台记录与视觉退出栈另外显示HTTP超时、历史IMU配对失效及满ROI距离变换权重溢出。

修复：angle_observation的距离变换增加零边界并以float64计算权重；observer_runtime使用新鲜当前IMU做控制，保留图像时刻历史配对用于测量，鱼头点选冻结当前视角并逆变换透视坐标。web_runtime本地代理超时覆盖鱼端1.5s超时，100ms发送周期包含HTTP耗时，只发最新目标；增加目标确认计数、应答年龄和退出原因，全停应答不清掉原退出原因。终端跨页面收集，新增Wi-Fi序号及航向有效状态。

性能：workstation.js增量追加终端行，避免每250ms重建约1MB文本；robot.js主状态合并为最高5Hz渲染，不改变控制／遥测节奏；backend、links、progress按页面或实际活动停止无关轮询，vision.js隐藏网页时关闭视频显示，详细JSON仅展开时刷新。API全部模拟的隔离Edge网页、同一1000行夹具12次刷新，中位30.4→0.4ms，P95为38.1→0.8ms，零浏览器错误、零横向溢出；build/host-ui/before.json与after.json保留结果。该数值只代表终端刷新，不外推整机倍数。

新日志：自动段累计确认235个目标，控制往返中位94.03ms；PC速度中位0.478m/s、目标中位0.083m/s、横向绝对误差中位0.274m。234条RUNNING样本中73.9%的转向达到±99%以上。结合轨迹图优先调整PC外环：前视0.50／0.35m、横向增益0.6、前视收紧距离0.40m、曲率预判0.50m、巡航0.18／最低0.04m/s；速度前馈参考0.50m/s、PID60／10／6、积分限幅±12%。所有参数来自control.json，运行入口和离线工具均消费新增字段，网页初始速度读取同一配置。前馈参考不作为速度输入限制。鱼端2.7.1、P2.5、±1°及转向增益1.5保持，本轮无新BIN。

检查与交付：全ROI权重、当前IMU控制、保留透视点头、含HTTP的发送周期4项内存回归通过；新参数配置加载、Python／JS语法及差异空白检查通过。未运行新水动力仿真、真实相机／串口或实鱼命令，未烧录；参数只是基于日志的初步整定。上位机修订workstation-28-tracking-tuned，已先给用户启动器和参数，再补齐现有README与循迹说明；交付后的文档阶段没有修改代码或固件。

<!-- original-process-entry: 059 sha256: 06893b127afb10d9fd40c58bb0755ab76fee8c93b9e084b7c8e0a5baf371e60d -->
## 2026-10-25：修复透视图鱼头标定点击锁住

用户反馈点鱼头方向后卡住，要求加快交付。定位为冻结帧仍使用MJPEG：合成1000×500图经生产流接口，浏览器等待1.2s仍未触发load、naturalWidth为0；相同图经现有单帧JPEG接口立即load、宽1000。网页依赖load解锁点击，因此冻结后无法继续。上轮仅覆盖透视坐标映射，遗漏了真实图像加载过程。

修复：所有冻结标定图改用现有frame接口；observer按成功编码递增preview_revision，用于更新角点／框选叠加；完成或取消后恢复stream，鱼头保持当前视角。启动器和服务修订为workstation-29-frozen-head，循迹参数和固件保持。新增合成画面真实网页检查脚本，修正旧检查中的全图坐标预期；完整网页检查先遇到夹具选择器和请求包装错误，修正后复验被自动权限审批超时拦下，沙箱内浏览器子进程也被拒绝，未宣称该检查通过。Python／JS语法和差异检查通过，未访问实鱼、相机或串口。按用户要求停止追加检查并直接交付。

<!-- original-process-entry: 060 sha256: b7ccd9ee86e81014d02f66e3a3839eb5948df90733db3906bfb05b319d6e7858 -->
## 2026-10-25：降低自动循迹目标速度

用户要求先降低游动速度。本轮仅将 vision/configs/control.json 的 cruise_speed_mps 从0.18降至0.10m/s，upper/index.html 输入默认值同步0.10；其余引导、速度PID、鱼端转向参数保持本轮修改前值。JSON差异核验确认只改变目标速度，网页默认值已核对。未确认运行中视觉已应用；重启视觉进程后读取新目标。本轮没有验证实测速度；0.10m/s是目标值，不等同于实际水中速度。


<!-- original-process-entry: 061 sha256: 529074492b86b1fba2d012dcd13cbc357390486e35759fcdfee83bb57d00078c -->
## 2026-10-25: dynamics calibration delivery

Delivered firmware 2.7.2-drive-calibration (912800 bytes; SHA256 a75e297c8ccfa94d433103911156c55f5efa8ac7de45c9654a691df8e0dc527a) and workstation-31-dynamics-vision. Automatic heading targets no longer add yaw-driven tail propulsion; calibration_target provides independent tail drive/bias and prop common/differential over authenticated Wi-Fi. Fixed output pauses feedback without changing PID gains; native response buffer grows from 4608 to 6144 bytes. No task, hardware mapping or control periods changed.

Independent test page records manual-duration segments, release/coast events, source timestamps and actuator/allocation observations to uniquely named Desktop/RobotFish-Dynamics folders. Confirmed request, actual output-zero telemetry and estimated motion onset are distinct. New segments clear memory, not saved files; save failures retain data for retry. User subsequently rejected repeated preparation: embedded live camera and calibration actions now live on the test page, pool dimensions fixed to 3 x 1.6 m, camera height corrected to 2.67 m, existing same-camera corners reused, head selection automatically establishes zero.

Delivered before extended checks/documentation as requested. ESP-IDF build and image validation passed; firmware native checks, 9 targeted passive-recording/ordered-control checks and UI syntax/mock checks passed. Existing yaw_full native test retains an incompatible old Kp assumption; PID was not changed. Final embedded-video simplification and saved-corner scaling have syntax validation only. No real camera, serial, flashing or fish motion was performed; measured propulsion recipes and bend tuning remain pending user data. See docs/dynamics-calibration.md.

<!-- original-process-entry: 062 sha256: 33fa18916d41a96c29f9f7c0778c51343f493351f1c96f1647119a1b0ee96ac0 -->
## 2026-10-25：默认关闭的实测弯道规划与循迹验证准备

范围与原因：用户要求先准备弯前减速、过小弯道自动圆角、执行路线显示、推进分配日志和约一秒米制速度复核，等实测数据后再启用。用户确认采用速度／滑行距离／左右最小转弯半径表，缺数据不启用；零前进量不追加鱼尾推进也先做成关闭的可选修正。

最终决策：curve_speed.enabled=false、profile_file=null、trace_validation.enabled=false；ROBOFISH_ENABLE_HEADING_FORWARD_DECOUPLING=0。PC默认关闭时路径与控制输出保持原逻辑。之前标定版本的无条件航向前向解耦已收进该固件开关，默认仍保留旧的转向摆尾分配；开关置1才使用前向量决定推进，固定输出标定的独立鱼尾／双桨命令仍有效。PID、任务周期与引脚映射未更改。

实现与文件：vision/curve_speed.py 读取可追溯的HYBRID实测表，按左右半径生成相切圆角，以实测滑行距离向前传播弯道／终点速度上限；path_guidance、observer_runtime 接入规划及实际路线显示。control.json／control_config 管理默认关闭配置；angle_observation、web_runtime、upper/station.py、upper/workstation.js 接入图像时刻之前的分配快照及可选CSV／终端诊断。ControlState.h／ControlConfig.h 增加固件开关。说明见 docs/vision-control.md 的“待实测启用”章节。

验证：37项Python回归通过，含8项新增规划／日志测试；关闭与开启固件选项各28项C++检查通过。ESP-IDF5.3.2构建成功；Python AST、JS语法与 git diff --check 通过。修正旧回归夹具的 control_heading 名称、通信统计字段及既有 STOP→MODE 顺序。结果和本次范围补丁保存在 build/host-vision/curve-speed-preparation。

限制与后续：未录入真实标定、未验证水中效果、未重启服务、未烧录或发送实鱼命令。圆角放不进路线时拒绝启动；不把急停、反推或不同混合配方的滑行数据混用。约一秒速度是净位移复核，弯道会低于弧长速度。待实测表验证后，按用户后续指令分别启用规划、日志和固件选项；当前没有待确认的实现问题。

<!-- original-process-entry: 063 sha256: 4f4c82f7599618be6071eb5b1044732d9222d7e935e37a2fd9908a316091a2e2 -->
## 2026-10-25：轻微减弱姿态角软件滤波

范围与原因：用户反馈当前陀螺仪滤波偏重，要求稍微减弱。最终将 main/config/AttitudeConfig.h 的 kAngleFilterStageHz 从3.0调为4.0Hz，保留两个一阶低通串联。该参数用于S3姿态角及其航向闭环、上位机角度显示；未修改传感器内部设置、原始角速度、PID、任务周期或先前默认关闭的实测规划开关。同步 docs/attitude-control.md 与 docs/attitude-goals.md。

验证与交付：ESP-IDF5.3.2构建成功，git diff --check通过。新固件 artifacts/RobotFish-ESP32S3-OTA-20261025-220344.bin，912800字节，SHA256 3beca9a7183aee022e81952063618cba2172ea9e1ca50ec4fd2470316c3187cd。

限制与后续：预期减少平滑延迟，同时可能略增抖动；尚未验证水中效果，未烧录或发送实鱼命令，升级该固件后生效。无待确认的实现问题。


<!-- original-process-entry: 064 sha256: 62be24f2ba7c08c098310673493974b738759db1ab975f06f93dfa0b1ddc9d71 -->
## 2026-10-25：S3 机载网页美化与遥控／调参／OTA交付

范围与原因：用户指定 RobotFish-ESP-IDF，要求美化 S3 上运行的上位机并提供固件更新、遥控、简单调参。直接更新 main/control.html，不改 PC 操作台业务、运动控制算法、硬件、任务、PID 默认值、现有4Hz滤波或保护策略。

最终决策：青绿色离线单页，遥控／调参／升级三页签，实时鱼体姿态与六路输出；显式 acquire，不自动接管。沿用认证 /api/v1/control 的有序会话、200ms按住目标与松开归零；全停丢弃待发送运动，模式切换先STOP，支持键盘／鼠标／原生双指。LoRa健康时方向与调参禁用、STOP仍可用。PID范围0～100，编辑不被轮询覆盖，写入后以设备回读确认，不自动启用辅助、不写NVS。失焦和离开页的全停为尽力而为，沿用 kLeaseMs=0，断网无法保证实鱼停止。

升级流程：沿用 Basic 网页授权与 /ota_upload 原始BIN接口；客户端检查大小、ESP32-S3与robotfish标识，后端继续做镜像校验及既有全停屏障。XHR进度只表示传输，确认提交后等待观测重启和版本回读；不自动重连遥控。页面内嵌CSS／SVG，无外部依赖。涉及 CMakeLists.txt 版本、main/control.html、tests/control/web_status.js、tests/wifi_http/browser.py、upper/tools/verify_browser.py 中机载页兼容断言及 README。

验证与交付：目标工程 Node 模拟回归和真实 Edge 检查通过；Basic认证、组合方向、松键、排队STOP、模式、PID边界／回读、LoRa主控、会话冲突、失联快照、错误BIN、模拟OTA／重启；手机原生双指和320／375／390／720／768／1024／1440像素三页签无横向溢出，浏览器错误0。Python语法与本轮文件 git diff --check通过。ESP-IDF5.3.2构建通过，最小应用分区仍有71%余量。交付 artifacts/RobotFish-ESP32S3-2.7.4-web-console.bin，961232字节，SHA256 cbb061c9b4466d3c84b411aca39f8ce94d73e2e139ab962fe3c4f2e1f08b720c；已有inspect_image校验通过，并确认BIN包含最终页面的逐字节内容。截图与报告 build-host/s3-console；结构化证据 validation/s3-console-20261025.json。

限制与后续：仅本机模拟与编译验证，未打开串口、切换网络、烧录、连接或驱动实鱼。实鱼Wi-Fi遥控、实际OTA与水中调参效果待上板确认；无待确认的实现事项。


<!-- original-process-entry: 065 sha256: 01f50718e8970ca5214ea08cab87a5c525530c3b8f5a3b85aeaf840f51516b12 -->
## 2026-10-25: measured dynamics integrated into tracking

Read all 32 Desktop/RobotFish-Dynamics segments and the supplied analysis/out report; original data unchanged. Actual metadata and actuator commands take precedence over directory names. Prop-only late-drive references use final two camera seconds: 20 percent -> 0.24122803162319684 m/s, 30 percent -> 0.3862307690767761 m/s, 19 samples each. Both are still accelerating, not verified equilibria. Retained provenance and quality in vision/configs/measured_dynamics_20261025.json. Default 0.10 m/s FF is approximately 8.29 percent; below-measured-range estimation and actual allocations are logged. PID gains remain unchanged.

HeadingTarget now separates common prop propulsion from static tail attack-angle steering. Periodic tail drive and prop differential default to zero; no yaw-driven extra propulsion, no mode changes to manual/calibration/baseline. Existing tail polarity, 1.5 gain, +/-24 degree geometry and local periods remain. Identity/status advertise heading_tail_guidance_supported and prop_surge_tail_bias. PC checks capability before preparation. Ordinary turn speed reduction remains; optional measured-radius/coast planner stays disabled because the new static-tail recipe lacks matching tests and recorded coast paths mostly end before stopping. Trace validation enabled. Corrected future calibration CSV imu_yaw_cw_deg to mapped body yaw; original sensor fields retained.

Version was initially 2.7.3-tail-guidance, then concurrent on-board-console work advanced the shared source/build to 2.7.4-web-console. Preserved that work and confirmed the allocation/capability survived. Final independently validated image snapshot: artifacts/RobotFish-ESP32S3-2.7.4-measured-tracking.bin, 961232 bytes, SHA256 cbb061c9b4466d3c84b411aca39f8ce94d73e2e139ab962fe3c4f2e1f08b720c. PC revision workstation-32-measured-tracking. ESP-IDF build and image validation passed; 54 allocation/calibration, 2140 LoRa-heading and 2161 Wi-Fi-heading owner checks passed; 14 targeted PC checks passed. Legacy yaw_full test still assumes the earlier P=5 saturation threshold, unrelated to the retained P=2.5. Delivered BIN/startup before documentation. No fish commands, camera, serial or flashing; new recipe water performance is unverified.


<!-- original-process-entry: 069 sha256: d95a6b3dbff2fb1ed7d765ec0b9d967609e8a726fb5ff626fe3c4614b0d76bef -->
## 补录：2026-10-25 Git 交付结果（时间按 UTC）

范围／原因：此前记录了实测动力接入，但遗漏后续上传结果。用户明确确认将源码、测试、说明与2.7.4固件共74个文件，推送到 `jikaic149-svg/codeFish` 的 `main` 分支、`RobotFish-ESP-IDF` 子目录；排除本机标定和原始实验。

最终结果：当时已按远端历史增量推送提交 `1c1e9c6947a4642f99b9428db18f7045d40787ed`，标题“融合实测动力循迹、独立标定与机载控制台”，74文件、4811行新增／326行删除；未强制覆盖历史。使用既有发布工作树 `build/git-publish-heading-observer`，保留远端其他目录。2.7.4测量循迹和机载控制台命名BIN为相同镜像，校验见前文。

验证／边界：原推送完成后曾核对远端提交；本次补录只读核验发布工作树HEAD、提交统计和 `origin/main` reflog，后者明确记录 `2026-10-25 16:06:25 +0100` 的 `update by push`，即UTC 15:06:25。本次没有重新推送，也没有查询远端当前HEAD；2.7.5～2.7.7及之后的本地修改不在该提交内。原自动审批拒绝后，用户已明确确认上述目标与范围，随后完成推送；此事不是当前阻碍。

<!-- original-process-entry: 074 sha256: 2238558c92d340c1f45a1cdd487fbcefbf75de250e0c210fcc281395b34e7950 -->
## 2026-10-25数据综合评估：运动性能与循迹问题报告

范围／原因：用户要求依据此前全部数据评估机器鱼运动性能和现存问题，并交付报告。本次核查桌面现存41段动力试验（早期32段498.785秒驱动、新静态9段167.039秒驱动）、14:52／16:09／16:53／17:45四份循迹TXT、已有分析脚本／报告及当前2.7.7源码。08:57／09:51原始TXT本轮桌面未找到，沿用既有Process作为历史背景，不声称重新计算该两份原始日志。

关键修正：旧kinematics的v_steady实际为运动开始后到松开前后60%时间窗口中位值，不是稳态检验；早期32段只有21段在线summary稳态候选非空。旧“混合80%”是尾偏40%加差动40%，不能当单独尾偏80%参与回归。尾角是命令，名义关节偏置不等于实测水动力攻角；滑行数据多为未停稳的有限记录窗口；运动超过1cm／3°的门限触发延迟不等于执行器延迟或动态时间常数。旧纯差动40%左右效能差异显著，不能概括成均为7～10°/s。运动轨迹曲率满足κ=(r+β̇)/U，只有侧滑变化可忽略时才可用R≈U/|r|，同时应统一时间窗口和角率来源。

统一日志口径：PC仅tracking=true；FISH按启动代次／uptime去重，并限制在各连续活动PC首末窗口。四轮横偏MAE/P95依次为11.76/21.20、9.94/19.44、10.38/25.79、10.85/20.01cm；对应16:09仍是静态尾偏配方。最新严格窗口FISH为558条、饱和22.22%；此前561条／22.1%含首PC前3条，不是结果矛盾。16:53统一窗口188条与此前187／210口径也不同。14:52明确退出到恢复4.397秒；恢复首帧误差107.61°，相邻帧环绕变化159.82°，两者不能混称。其余三份保留活动窗无新暂停，不能外推为截断窗口之外也已验收。新静态右转组同共同桨推20%时，偏置40/60/80/100%的等效半径0.787/0.536/0.365/0.303m；左80%、共同桨推10%约0.378m，支持低速具备转弯能力，但不适用于当前1.5倍摆尾包线。

最终评估：已具备低速持续循迹与有效转弯的实验能力，当前平均横偏约10～12cm，尚未完成精确曲线跟踪整定。2.7.7偏置优先已经在最新日志兑现；仍存在路径圆角未满足约束、尾偏与摆动共享±32°关节行程、收油同步降尾频、非饱和路段横向收敛不足，以及1000行导出不完整。最新满偏窗口航向误差中位42.22°／尾频0.29Hz／连续饱和8.052秒；另一个大横偏窗口没有转向饱和，不能把全部问题归因于尾力不足。补测顺序为完整记录、当前配方开环阶跃、低速局部转弯包线及同条件桨尾分配；报告中的验收目标和E1/E2/E3均为建议，未冒充已经完成的测试或新控制实现。

交付／文件：`docs/reports/2026-10-25-motion-performance/机器鱼运动性能与循迹问题评估报告.html` 为自包含中文报告，配套同名PDF共13页，5张自绘中文图提供SVG／PNG，`source_manifest.json`记录225个来源文件的路径、大小和SHA256。复算脚本、核验JSON、逐段／逐帧CSV与渲染检查保存在 `build/host-vision/motion-performance-report`。报告引用Fossen海洋航行器模型和控制分配综述作为分层建模依据，不套用未测水动力参数。

验证／边界：独立复核主要科学与统计结论；生成脚本检查11段直行／9段静态／4轮日志和5幅内嵌图；无界面浏览器检查无JS错误、无横向溢出、无缺失图片；PDF13页均有正文，已目视检查封面与分页缩略图。原沙箱阻止浏览器子进程，按仅本地报告转换的范围通过自动审批后完成PDF，不涉及设备。未修改原始实验、生产代码、PID或参数，未启动／停止用户服务、连接实鱼、烧录或推送Git；本次评估与产物已经同步写入本Process。

