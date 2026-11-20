# 2026-10-27 开发记录

来源：2026-11-20 从根目录 process.md 按标题日期迁入。以下原记录及验证边界保留，当前状态只以根目录 [process.md](../../../process.md) 为准；旧工程路径不代表文件仍存在。



<!-- archived-from-root-process: 2026-11-20; 按原日期分组，同日保持原顺序 -->

<!-- process-archive-20261120-entry: 008 sha256: 8accea97be3107c0cba02ca4ac00674df6c1391927f1601508ffcfcbf3d85da4 -->
## 2026-10-27：英文便携版与英文使用说明

范围／决定：按用户要求制作独立英文版，仍交付单个完整ZIP；翻译界面、运行入口和程序提示，不翻译已有文档。新增`tools/portable_english.py`、`english_js_strings.cjs`、`english_strings.json`、英文使用说明和浏览器验证脚本。使用Python AST和TypeScript语法树定位字符串，保留变量插值、按钮ID、通信字段和非字符串代码结构；测试中的中文提示匹配同步为英文。

交付：桌面`RobotFish-Dev-English-Win64-20261027.zip`，2,894,187,315字节，94,016个文件，SHA256为`1356af7a7d28f2e1b6edf9951ba9ee6dbcd37a4fdf7accb6650cbb147b464606`。翻译2,651项文字，包内新增`START_HERE_EN.txt`，覆盖初始化、五个运行入口、控制、视觉标定、USB／OTA、日志位置和排障。英文版默认本机端口8768，与中文版8767分开。复用原压缩依赖，原中文版保留。

验证：英文环境初始化、离线补全及CPU推理通过；五个页面无可见中文、无浏览器JS错误；12个前端JS及机载页面内联脚本语法通过；14项HTTP／OTA回归和机载网页控制回归通过。6,005份已有文档内容不变，46份非HTML固件源码、原固件BIN／ELF及模型权重保持不变。英文ZIP全部条目CRC通过，运行界面字符串扫描无遗漏中文，启动器摘要一致。

边界：既有文档、代码注释、报告生成文字和技术文件名保留；原固件二进制不因上位机翻译而改变，英文机载页面源文件须重新编译烧录后生效。未连接真实相机、串口或实鱼；未执行真实USB／OTA，英文版未上传GitHub。

<!-- end-process-archive-20261120-entry: 008 -->

<!-- process-archive-20261120-entry: 009 sha256: e6df0cb5b3eb35744ed973f19ec75abcf85e649b4d668ddfa05963d9dfc2e82b -->
## 2026-10-27：降低自动循迹游动速度

范围／决定：用户要求降低机器鱼游动速度，未指定数值；将当前源码工程的默认巡航目标由0.12降至0.08m/s（8cm/s，降低约三分之一），同步网页初始值和当前说明。最低速度目标仍为0.04m/s，速度PID、转向参数及鱼端固件保持原值。

涉及文件：vision/configs/control.json、upper/index.html、README.md、upper/README.md与docs/guides/vision-control.md。

验证／边界：配置加载、上位机配置接口和HTML默认值核对通过；相对修改前配置只更改巡航目标及对应说明，6项路径引导回归通过。运行中需在视觉页填0.08并点击“应用速度”，或重新启动视觉进程加载配置。本次未向实鱼发送指令，实际水中速度未验证；桌面便携目录和已交付ZIP未同步修改。

<!-- end-process-archive-20261120-entry: 009 -->
