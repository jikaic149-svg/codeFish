# 2026-10-26 开发记录

来源：归档前根目录 `process.md`。以下正文按原文保留；历史决策只适用于各自版本和试验条件。

<!-- original-process-entry: 066 sha256: 95db452084e050f2e320dba15f358a017e9d4b7b5e2b9a9c3a048665542287b5 -->
## 2026-10-26: continuous tracking and controlled hydrodynamic trials

14:52 trace review found a PC imu_unavailable STOP at 14:51:27.823 while fish IMU remained valid. The 107-degree restart spike came from path progress reset/local projection, not an IMU flip. Active-only PC zero-power count is 1/207; did not add minimum propulsion. WebBridge keeps original timestamps during transport gaps; a bounded 250ms zero-output observation pause retains authorization and path, while persistent invalidity/manual STOP/takeover/reboot cannot resume automatically. Camera receipt freshness is separate from the estimated 300ms exposure delay; sample timestamps still drive speed derivatives and logging. ACK >1s and pause history are explicit.

PathGuidance initializes from global path projection and initial heading, keeps valid progress, applies bounded curvature lookahead to the actual heading target, filters lateral error and limits heading-reference slew. Trial values: straight .12m/s, tight-curve base .05, minimum target .04; existing PID/firmware unchanged. Geometric target radius .60m is used only when it fits within .15m offset; constrained U turns keep the original path and report achieved curvature/unachievable, never silently use a tighter fillet. Actual velocity-vector projections/course/sideslip are logged; main speed feedback is unchanged.

The supplied yaw_map.py adds absolute prop differential to tail bias, so its 80-percent point and 67-percent extrapolation are not single-tail-bias measurements; pure-tail drive/bias are also confounded. Added an explicit static-tail test recipe: no oscillation/differential, 0/40/60/80/100 bias presets, fixed prop common. During manual hold X/Z apply +/-60/0 bias steps without changing common propulsion; release remains full STOP and coast recording. No automatic test duration, assumed tau, or fictitious hydrodynamic envelope.

Workstation revision33. 23 focused no-hardware tests plus Node tail-step invariants and syntax/diff checks passed. No camera/serial/fish commands or new firmware build. New water performance remains unverified; kinematic replay cannot establish a 10cm physical tracking error. Existing 2.7.4 image is retained.


<!-- original-process-entry: 067 sha256: cbf83032e3d75f5f62957194d7ceee38725fb06fb5f588b67460932bf97e7593 -->
## 2026-10-26: restore and strengthen tail gait

User explicitly corrected the static-tail interpretation: tracking must use oscillating tail propulsion as well as common prop thrust. Heading tail-drive ratio is now 1.0; yaw shifts its centre without increasing drive/frequency or restoring the 35-percent floor. Replaced pure-prop forward references with the measured 1:1 hybrid points (.05734369/.14829029/.23487840m/s at 10/15/20 percent). New nine angle-radius trials have zero tail drive, left common10/right20 and remain static references, not an oscillating-tail envelope.

After the 16:53 trace (about73s continuous to arrival, median tail drive11.44%) user requested stronger motion. Final 2.7.6 adds amplitude gain1.5 to heading and explicit non-baseline HYBRID calibration; unchanged nominal frequency, bias gains, periods and mechanical/rate limits. Old baseline and static comparison retained. Source forward references were measured at gain1 and are now marked initial estimates, not verified gain1.5 calibration.

Made two always-visible test entry buttons and direct #swim/#static links. Selecting them never sends motion; bias steps preserve each chosen gait/common output. PC revision35 requires the matching firmware. Final image artifacts/RobotFish-ESP32S3-2.7.6-strong-tail.bin: 961344 bytes, SHA256 94c4518c817d1c11e1e2bd5b4e0ee2179ed996dbd29fe27b004ca90880a3d110. ESP-IDF build and image validation,161 native checks,20 PC checks and JS step invariants passed. No physical commands, camera, serial or flashing; delivered first, then documented. New water response remains unverified.


<!-- original-process-entry: 068 sha256: 74babb7a09085be15e99b8fbfa6f83a597f31ee3003d26c391f293e4aea086d2 -->
## 2026-10-26: preserve steering bias with stronger gait

User confirmed the screenshot was captured after 2.7.6. Identified a deterministic joint-travel allocation conflict: front travel +/-32deg, requested full bias24deg plus gain1.5 gait at drive20% (16.77deg half-amplitude) previously scaled both, reducing bias to18.84deg. 2.7.7 gives heading and non-baseline HYBRID calibration bias priority, retaining24deg bias and8deg half-amplitude in that case. Straight gait gain1.5 remains; no new propulsion floor, PID, speed/path gains, mechanical or rate limits. Ordinary manual/pure-tail/pure-prop calibration and old baseline remain on their original allocation.

Motion request preserveTailBias and result effectiveTailBiasDegrees/effectiveTailAmplitudeDegrees make the allocation explicit. Identity/status advertise heading_bias_priority_supported. Allocation telemetry effective_tail_bias_deg/effective_tail_amplitude_deg represents nominal front-joint values after travel limiting but before final angle slew, not measured servo feedback. PC revision36 requires this capability and records these fields in terminal, host telemetry and observation CSV. Path progress/segment is also recorded to distinguish future self-crossing association problems; current screenshot alone does not establish a wrong branch.

ESP-IDF build and image validation passed. Image artifacts/RobotFish-ESP32S3-2.7.7-steering-priority.bin: 961600 bytes; SHA256 20f9f9b30a8958f76242afb09858262b198db965f78295bbe6da05498c612e59. Native checks:193 targeted calibration/allocation,2140 LoRa owner,2161 Wi-Fi owner,60561 existing allocation checks over51840 ticks.20 focused PC checks plus Python/JS syntax and diff whitespace checks passed. Did not start or stop services, open camera/serial, flash, or send fish commands. Existing water measurements remain reference only; new physical tracking response is unverified.



<!-- archived-from-root-process: 2026-11-20; 按原日期分组，同日保持原顺序 -->

<!-- process-archive-20261120-entry: 001 sha256: d2f9099b2605b54e08aaa2e2df9a55255cfd554a7930a2b7c479790749a236f0 -->
## 2026-10-26：目录整理与文档按时间归档

范围／原因：用户要求使用 tidy-skill 整理工程文件，并按时间顺序归档文档，为后续完整开发环境交付做准备。

最终决策／文件：保留固件、上位机和视觉的生产目录及运行入口；当前专题说明归入 `docs/guides/`。原 `process.md` 的 75 条正文按标题日期归档，保留原始序号与每条 SHA256；14 条未注明日期的记录单列。2026-10-22 LoRa／IMU 报告与2026-10-17验证清单归入对应日期目录。整理前 README 全文作为2026-10-26归档快照，日期指快照日期。录像测试迁入 `upper/tests/` 并适配导入位置，更新文档和测试入口。

验证：原75条历史正文可按原始序号逐字节还原，其中61条按明确日期归档、14条单列；迁移的5份说明与验证JSON以及README快照均通过内容还原校验。165个项目内文档链接有效，其余217个原有文件摘要不变。迁移后3项录像测试通过，覆盖真实MP4编码／解码、同步CSV、尺寸约束、失败编码和空录像；初次执行受沙箱临时目录权限限制，限定工程内临时目录并通过自动审批后在沙箱外执行通过。未因此修改生产录像逻辑或重跑实鱼测试。

边界／待办：保留既有历史快照、模型、原始实验和构建缓存；生产算法、固件参数、硬件、系统环境和运行服务保持原样。本次不构建便携包、不烧录、不发送实鱼指令、不提交或推送 Git。

<!-- end-process-archive-20261120-entry: 001 -->

<!-- process-archive-20261120-entry: 002 sha256: 09193e705b3626118d2c8a68869ce9915460c2954ae3852ba9edc448ba60125b -->
## 2026-10-26：按用户要求删除 archive 与 artifacts

范围／授权：用户明确要求删除先前讨论的两个目录 `archive/` 和 `artifacts/`，覆盖上一条目录整理时保留这两个目录的决定。

结果／验证：删除前核对两个绝对路径均为本工程直接子目录，且目录及其内部没有重解析点；已删除并确认路径不存在，包括旧代码快照、命名应用固件、USB安装包与硬件备份。更新README、文档索引和当前循迹说明，历史归档原文保持不变。两个既有Git跟踪固件随目录删除而显示为删除状态，未提交或推送。

当前固件／边界：`build/robotfish.bin` 保留，961600字节，SHA256仍为 `20f9f9b30a8958f76242afb09858262b198db965f78295bbe6da05498c612e59`，对应2.7.7-steering-priority。其相对2.7.6的主要变化是指定航向及非baseline混合标定保留尾偏优先，关节剩余行程用于摆尾，并回传分配后的名义偏置与摆幅；PID、机械限位和普通手动分配未改。本次仅清理目录与更新说明，不重新编译、烧录或操作设备。后续执行build.ps1会重新生成artifacts目录及新构建副本，不会恢复已删除历史备份。

<!-- end-process-archive-20261120-entry: 002 -->

<!-- process-archive-20261120-entry: 003 sha256: ef86cbdf1255609c391414df2c22871462419384a179c66a495f0af2c038a6d7 -->
## 2026-10-26：Windows 完整开发便携包与 launch.json

范围／授权：用户要求将当前源码和环境交付到其他Windows电脑，未知显卡、包含上位机／视觉／训练／固件编译烧录开发范围；初始化和项目输出限定包内。明确排除项目docs，交付文件夹放桌面；随后要求将多个CMD操作入口集中到VS Code启动配置。

交付／文件：桌面 `C:/Users/rlyoulo/Desktop/RobotFish-Dev`。包含当前main、upper、vision、tests源码及模型；Python3.13.14、PyTorch2.11.0+cpu、Ultralytics8.4.118；ESP-IDF5.3.2及配套GCC／GDB／OpenOCD、CMake／Ninja、原生C++工具、Git／Node与便携VS Code和Python／C++扩展。已审核数据按原划分复制263张训练／12张验证／13张测试图及标签，配置改为包内相对路径。原始素材、权重及源数据不回写；未复制个人密钥、原相机标定、原日志或原构建缓存。

入口／隔离：`.vscode/launch.json` 提供14个配置，覆盖初始化、普通／调试后台启动、停止、环境检查、编译、指定COM烧录、关键回归、CPU训练／验证、最小训练和交付检查、开发终端及编辑器；根目录只保留Editor.cmd用于打开便携编辑器。工作区打开包根目录，配置使用workspaceFolder；移植后初始化重建包内IDF虚拟环境和工具路径。PATH、APPDATA、LOCALAPPDATA、临时文件、pip／YOLO／Torch／Matplotlib缓存均在子进程中设置为包内位置，不修改系统变量、注册表、网络、驱动或用户配置。现有浏览器使用独立包内profile。未指定有效COM时拒绝烧录。

源码适配：新增tools/package_portable.py、portable.py、portable_training.py、test_portable_delivery.py与便携README资源。upper/dynamics_calibration.py支持显式包内实验输出目录；probe_upgrade.py和development.py支持包内Python／编辑器路径，若未设置环境变量则保留旧默认。相关分析／回放工具支持包内编译器和样例目录。生产固件算法、PID、机械限制和硬件映射均未改变。

固件／构建：原2.7.7交付BIN及匹配ELF保留，BIN SHA256为 `20f9f9b30a8958f76242afb09858262b198db965f78295bbe6da05498c612e59`。包内工具已完成全新编译，并核对原CMake缓存后保留ROBOTFISH_USB_DIAGNOSTICS=ON；新编译BIN、ELF、MAP独立命名为*-rebuilt，BIN961600字节，SHA256 `5c845eec26b4f9ae144de87d3a22a43c2a4654cb30e4c14b9ff9326e84c833b7`。构建路径和生成元数据不同，不宣称两镜像字节相同。未重建或覆盖源工程的原交付BIN。

验证：包内pip依赖检查、真实CPU模型推理、工具CLI和编辑器CLI通过；禁用外部DNS解析后，用3张训练图和64输入完成1轮最小训练及保存／重载，仅验证训练链路，不作为模型性能。原生运动60561项、标定／分配193项、Wi-Fi cJSON128项及BSP／LoRa回归通过；39项选定Python测试和两组JS回归通过。普通上位机及可断点调试的后台在随机本机端口启动、提供网页／模型配置并正常退出。独立验证目录重新初始化后，CPU依赖、模型、IDF Python、工具及便携编辑器CLI通过；未移动用户正在查看的交付目录。

已知限制／边界：旧tests/attitude/regression.cpp仍使用Kp=5及小于当前1°死区的收敛假设，保留原测试并在清单和README标注，未纳入默认关键回归集合，不宣称全部历史测试通过。目标电脑的摄像头、USB／串口／JTAG驱动及实时性能仍待现场确认，不自动安装驱动。没有打开真实摄像头或串口，没有发送实鱼命令、烧录、提交或推送Git。排除项目docs的要求已满足；项目文档归档留在源工程，不随便携包复制。

<!-- end-process-archive-20261120-entry: 003 -->

<!-- process-archive-20261120-entry: 004 sha256: 946ca89bf2f2c26df6fa027b4d7c91d0aed77c69416a3a32ed8abf455b2a4ed5 -->
## 2026-10-26：便携包收拢为四个操作并补全结果反馈

范围／最终入口：按用户后续要求覆盖上一条的14项入口，`.vscode/launch.json`和工作区任务菜单仅保留“启动上位机、编译固件、烧录固件、环境检查与补全”。不开放YOLO训练、验证、调试、测试和开发终端菜单；开发源码及运行依赖仍保留。根目录仅用`Editor.cmd`打开便携编辑器。

行为／结论：每次结束显示成功、失败、已取消或未确认完成，以及实际结论、输出位置，并保存`state/results/<操作>.json`。编译读取实际工程版本和产物摘要，实时日志存入`state/logs/build.log`；不自动烧录。烧录必须填写有效COM端口，留空取消；工具正常退出且日志确认全部镜像校验后才报告写入校验成功，不将其视为固件启动或实鱼运动验收。

环境补全：新增`offline/environment.zip`，包含Python运行环境、IDF Python依赖和字体，约386.4 MiB。补全前按manifest校验SHA256和归档路径，仅恢复缺失或空文件，不覆盖已有非空文件；随后更新包内配置并执行软件环境检查。SDK或编译工具缺失时报告失败，提示重新复制对应目录；不安装系统软件、驱动，不修改系统PATH或注册表。

验证：新增8项测试通过，覆盖缺失依赖恢复、保留现有文件、坏备份与越界路径拒绝、编译实际版本及摘要、烧录校验不全不报成功、无效串口拒绝、空串口取消和状态持久化。桌面交付包实际执行环境检查与补全通过，本次缺失文件为0；实际全新固件编译和分区大小检查通过，版本2.7.7-steering-priority，BIN961600字节。随机本机端口启动上位机并读取健康接口通过，随后正常停止；实际空串口输入返回取消且未启动烧录。未连接摄像头、串口或实鱼，未改变生产固件算法和参数。

<!-- end-process-archive-20261120-entry: 004 -->

<!-- process-archive-20261120-entry: 005 sha256: 1623417789da6fb5a2ac8141fb2a80a666627496c7054378513bec95eb5d7f37 -->
## 2026-10-26：USB自动串口、网络OTA与浏览器自动打开修复

原因／最终决策：用户要求将烧录拆成USB和OTA，并指出上位机只打印网址而没有打开页面。最终菜单覆盖上一条四项配置，保留启动上位机、编译固件、USB烧录（自动串口）、OTA升级（自动发现）、环境检查与补全五项。

涉及文件／行为：`tools/portable.py`的USB入口只枚举串口，不主动探测UART；优先Espressif原生USB，其次常见USB转串口，唯一候选自动使用，多个候选列出序号供选择，留空取消。写入继续使用既有ESP-IDF的ESP32-S3编译、烧录和镜像校验链路。OTA复用`upper/device_client.py`现有有限地址发现和`upper/robotfish.py`镜像检查、上传、重启换槽及健康确认；先编译当前源码，再自动上传，不扩展固件网络或OTA协议。配置、日志及结论仍在包内。同步README与交付验证器。

浏览器修复：Windows的`os.environ`转为普通字典后键名为大写；旧启动器使用混合大小写ProgramFiles查找失败。现使用大写键查找Edge并设置包内浏览器profile，无Edge时调用系统默认浏览器打开页面。按用户指令已立即发送打开当前8767页面的请求。

验证／边界：16项启动器测试通过，覆盖USB候选选择、无候选拒绝、取消、浏览器路径和默认浏览器、模拟OTA发现上传及确认；14项已有RobotFish HTTP／OTA测试通过，包含错误镜像、错误密码、相同版本换槽和启动未确认场景。没有触碰真实串口或上传到真实设备。当前固件为AP模式，OTA通常需电脑连接RobotFishLOAR1热点；不会自动更改电脑Wi-Fi连接。真实USB写入及OTA仍需现场验证，未将模拟结果作为实机通过。

<!-- end-process-archive-20261120-entry: 005 -->

<!-- process-archive-20261120-entry: 006 sha256: cb2efdf692abbac0b774c711011f10e1d3cbd46321e6bb0fc6a2526c0a682b0a -->
## 2026-10-26：便携上位机浏览器意外关闭

现象／证据：用户反馈鼠标移动即关闭浏览器，后台健康接口仍返回200。未发现浏览器崩溃转储，日志包含AMD图形合成错误；启动环境存在清理子进程的Windows作业。直接子进程启动和仅脱离作业的试验不足以确认窗口长期存活，不能将图形日志直接认定为唯一根因。

最终决策／涉及文件：`tools/portable.py`改由Windows桌面`Shell.Application.ShellExecute`打开包内profile的Edge，避免浏览器依附短时启动／调试进程；关闭该浏览器硬件加速和后台模式以适配未知显卡，日志保存在包内。无Edge时仍使用系统默认浏览器。同步便携说明和启动器回归断言。未修改上位机控制逻辑、固件或系统设置。

验证／边界：用户在通过Windows桌面打开浏览器后明确回复“已解决”。保留当前窗口和后台，同步修复时不再移动鼠标或重启窗口；真实设备及其他显卡环境未作验证。

<!-- end-process-archive-20261120-entry: 006 -->

<!-- process-archive-20261120-entry: 007 sha256: c7fc0c35dfb517dc8ea067ae63452b7aad2d85189356455ae6d61305279a6b5f -->
## 2026-10-26：同仓库便携环境版本交付

范围／授权：用户要求上传到当前工程的同一Git仓库，并明确选择“独立分支＋完整包附件”。仓库为`jikaic149-svg/codeFish`，发布分支`portable/windows-20261026`，发布标识`portable-win64-2.7.7-20261026`。本地与远端main已分叉，独立分支保留现有历史。

版本／文件：新增`tools/portable_environment.json`，记录Windows x64、固件2.7.7-steering-priority、ESP-IDF5.3.2、Python3.13.14、PyTorch2.11.0+cpu、Ultralytics8.4.118、OpenCV5.0.0、NumPy2.5.2、VS Code1.143.1，以及46项运行环境依赖、55项IDF依赖、工具链和扩展实际版本。更新打包脚本和README；便携包根目录使用`environment-versions.json`。

交付边界：Git保存当前工程源码、归档文档和便携打包脚本，完整工具环境通过同仓库Release附件分发。附件排除项目docs、运行state、浏览器profile、编辑器用户状态、日志、临时缓存和本机标定；用户当前运行目录及窗口不作停止或清理。USB实机写入和OTA实机升级仍未验收。

<!-- end-process-archive-20261120-entry: 007 -->

