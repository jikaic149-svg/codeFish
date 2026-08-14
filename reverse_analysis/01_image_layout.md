# 镜像与段布局

## 固件身份

以下字段均为二进制直接确认：

| 项目 | 值 |
| --- | --- |
| 文件 | `CameraTest.ino.bin` |
| 文件大小 | 1,111,664 字节 |
| SHA-256 | `bc55582a37e8cc3abce227b09bf2bc9f66d6d93fda4782f00bc25daecdb79009` |
| 芯片 | ESP32-S3，chip ID 9 |
| 镜像入口 | `0x40375c70` |
| Flash 配置 | 8 MB、QIO（镜像头/写入参数编码为 DIO）、80 MHz |
| 段数 | 6 |
| 镜像校验和 | `0x93`，重新计算一致 |
| 镜像验证哈希 | `cd9b3a745232cd85590a6b25e661bbabfdce1b6583988b100b732dab21a191bb`，重新计算一致 |

镜像头第 2 字节为 `0x02`，即 ROM 首次装载使用 DIO 编码；但固件内嵌的完整 Arduino FQBN 明确为 `FlashMode=qio`。这两项并不矛盾：ESP-IDF 5.3.2 的 `CONFIG_ESPTOOLPY_FLASHMODE_QIO` 会把镜像头/烧录参数映射为 `dio`，随后由二级 bootloader 切换到 QIO。工程配置因此应选 QIO，而不是仅凭镜像头字节写成 DIO。

镜像可直接解析，DROM 中的代码引用、HTML 和日志均为明文，因此可确认应用镜像没有启用 Flash 加密。文件在验证哈希后结束，没有发现附加的 Secure Boot 签名块；但仅凭应用文件不能判断目标设备 eFuse 的真实配置。

## 应用描述符

`esp_app_desc_t` 位于第一个 DROM 段开头：

| 字段 | 值 |
| --- | --- |
| Magic | `0xabcd5432` |
| Project | `arduino-lib-builder` |
| App version | `8cabf2c` |
| 编译时间 | `Feb 11 2026 19:50:46` |
| ESP-IDF | `v5.5.2-729-g87912cd291` |
| ELF SHA-256 | `9016d45f936c080d9d7334478071723ae773ff1867b22b481fbc17134a91f236` |
| Secure version | 0 |

产品版本 `4.0.0-partner` 来自网页、JSON 状态和启动日志；它与应用描述符中的 Git/构建版本 `8cabf2c` 是两个不同版本字段。

## 加载段

| 段 | 类型 | 加载地址 | 长度 | SHA-256 前缀 |
| ---: | --- | --- | ---: | --- |
| 0 | DROM | `0x3c0c0020` | `0x3da90` | `55d5bb66ef6de656` |
| 1 | DRAM | `0x3fc97100` | `0x2560` | `0b03cdd512904110` |
| 2 | IROM | `0x42000020` | `0xb701c` | `5844709ec494a6a` |
| 3 | DRAM | `0x3fc99660` | `0x5554` | `9189847bfd625fbc` |
| 4 | IRAM | `0x40374000` | `0x13084` | `c4051ea91622e195` |
| 5 | RTC_DATA | `0x50000200` | `0x20` | `66687aadf862bd77` |

完整十进制字段、文件偏移和段哈希见 [image_manifest.json](metadata/image_manifest.json)。`segments/` 中每个文件都直接复制自镜像相应数据段，可用其哈希验证提取过程。

## 已恢复的静态内容

| 内容 | 数量/大小 |
| --- | ---: |
| ASCII 字符串 | 9,737 条 |
| UTF-8 C 字符串 | 4,240 条 |
| 重点字符串 | 799 条 |
| 有效段内 32 位指针 | 9,535 个 |
| 重点字符串指针 | 328 个 |
| 完整 Web 资源 | 4 个 |

Web 资源包括 38,523 字节的驾驶页、5,523 字节的 Partner Lab、PWA manifest 和 service worker。资源在 [web_assets](web_assets/) 中保持为 UTF-8 文本；各自原始偏移和 SHA-256 记录在镜像清单中。

## 镜像边界

这个文件是单一 ESP32 应用镜像，不是包含 bootloader、分区表和应用的 merged image。因此下列信息不能从该文件单独恢复：

- 实机分区表的完整布局和每个 OTA 槽的偏移；
- bootloader 编译选项与 Secure Boot/eFuse 实际状态；
- 设备 NVS 中已经保存的 Wi-Fi、Partner 脚本或 OTA 运行状态；
- 原始 ELF 符号表、源文件名和调试行号。
