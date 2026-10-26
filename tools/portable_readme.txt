RobotFish Windows 便携开发包（Windows 10/11 x64）

环境版本（2026-10-26）：固件2.7.7-steering-priority；ESP-IDF 5.3.2；Python 3.13.14；
PyTorch 2.11.0+cpu；Ultralytics 8.4.118；OpenCV 5.0.0；NumPy 2.5.2；VS Code 1.143.1。
完整Python依赖、工具链和扩展版本见environment-versions.json。

将整个文件夹复制到对方电脑，建议路径简短且不含空格。
双击 Editor.cmd 打开包内 VS Code；也可用现有 VS Code 打开 RobotFish.code-workspace。

所有操作已集中到 .vscode/launch.json：
在左侧“运行和调试”下拉菜单选择以下5个入口，按 F5 或 Ctrl+F5：

  启动上位机：页面地址 http://127.0.0.1:8767，退出请使用网页右上角“退出上位机”。
  编译固件：生成应用BIN、ELF和MAP，不自动烧录。
  USB 烧录（自动串口）：自动检测ESP32原生USB或常见USB串口；多个候选时选择序号，留空取消。
  OTA 升级（自动发现）：先连接机器鱼Wi-Fi，自动发现设备，编译当前源码并上传，等待换槽和健康启动确认。
  环境检查与补全：从包内离线备份补齐缺失Python依赖，更新配置和路径，再检查软件环境。

编译输出到 state/build/firmware 和 firmware/*-rebuilt.*。初始化不会烧录设备。
每项操作结束都会显示成功、失败、取消或未确认，以及实际结论和结果位置。
结论保存在 state/results/，编译及烧录原始日志位于 state/logs/。
USB烧录成功以工具日志确认全部镜像回读校验为准，不代表固件运行或实鱼动作已经验收。
OTA使用现有/get_ip和/ota_upload接口，不改变电脑网络配置；未发现设备、上传失败或重启未确认都会明确报告。
目前固件工作在AP模式，通常需要电脑连接RobotFishLOAR1热点，不能仅连接任意路由器。
OTA默认使用当前固件的出厂密码；若已修改，可在启动编辑器前设置进程环境变量ROBOFISH_OTA_PASSWORD。
不开放YOLO训练、调试、测试和开发终端的运行菜单。

环境补全仅恢复缺失或空的Python依赖文件，不覆盖已有非空文件，不安装系统软件或驱动。
若SDK、编译器、运行时启动文件或离线备份本身损坏，会明确失败并提示重新复制对应目录。

包含当前源码、best.pt/last.pt、263张训练图＋12张验证图＋13张测试图及标签，
CPU PyTorch、Python、ESP-IDF 5.3.2、编译器、GDB、OpenOCD和便携VS Code及Python/C++扩展。
不包含项目 docs/、个人密钥、原电脑场地标定、历史实验日志和旧构建缓存。

初始化不修改系统PATH、注册表、防火墙或驱动；配置、缓存及输出保存在本文件夹内。
包内Edge浏览器关闭硬件加速以兼容未知显卡，并独立于启动终端运行；浏览器错误日志位于state/logs/browser.log。
USB/串口/JTAG驱动需要对方电脑已有，本包不会安装驱动。CPU推理及实时速度取决于硬件。

firmware/ 保留原2.7.7交付镜像；重新编译的文件独立命名为 *-rebuilt.*。
编译保留原工程 ROBOTFISH_USB_DIAGNOSTICS=ON 的只读USB诊断开关。
旧 tests/attitude/regression.cpp 仍假定 Kp=5 和小于现有死区的收敛精度；
当前固件为 Kp=2.5、1°死区。该历史测试保留原文、在清单中标注，并排除于默认关键回归集合。
