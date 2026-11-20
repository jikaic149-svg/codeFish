# 文档入口

当前源码版本为 `2.7.7-steering-priority`，上位机为 `workstation-37-layout`（目录迁移修订）。历史版本、参数和验证结论只适用于各自记录的条件。

## 当前操作与开发说明

| 入口 | 内容 |
| --- | --- |
| [工程入口](../README.md) | 代码目录、当前版本、启动、编译与验证入口 |
| [当前进度](../process.md) | 后续 AI 优先阅读；只保留有效结论、最近验证与待办 |
| [上位机说明](../upper/README.md) | 控制、视觉、数据记录和退出方式 |
| [视觉循迹](guides/vision-control.md) | 路径、视觉观测、推进配方和已知限制 |
| [动力标定](guides/dynamics-calibration.md) | 摆尾联合推进、静态尾角对照和采样 |
| [姿态控制](guides/attitude-control.md) | yaw 闭环、坐标、PID、执行器分配和论文依据 |
| [控制目标](guides/attitude-goals.md) | 阶段验收目标；待验证指标不代表实测效果 |
| [模型说明](../vision/models/fish_body_yolo/README.md) | 当前 best.pt、训练来源、评估及适用范围 |

## 按时间查阅

| 日期 | 归档入口 |
| --- | --- |
| 2026-10-17 | [开发记录](history/2026-10-17/process.md)、[早期验证清单](history/2026-10-17/validation.json) |
| 2026-10-22 | [开发记录](history/2026-10-22/process.md)、[LoRa 与 IMU 实机记录](history/2026-10-22/lora-imu-setup-and-test.md) |
| 2026-10-23 | [开发记录](history/2026-10-23/process.md) |
| 2026-10-24 | [开发记录](history/2026-10-24/process.md) |
| 2026-10-25 | [开发记录](history/2026-10-25/process.md)、[运动性能报告 HTML](reports/2026-10-25-motion-performance/机器鱼运动性能与循迹问题评估报告.html)、[PDF](reports/2026-10-25-motion-performance/机器鱼运动性能与循迹问题评估报告.pdf) |
| 2026-10-26 | [开发记录](history/2026-10-26/process.md)、[整理前工程说明快照](history/2026-10-26/project-readme.md) |
| 2026-10-27 | [英文便携版与默认降速](history/2026-10-27/process.md) |
| 2026-11-18 | [循迹审查、仿真与接入](history/2026-11-18/process.md) |
| 2026-11-19 | [实测退出诊断与推进分工讨论](history/2026-11-19/process.md) |
| 2026-11-20 | [逻辑核对、目录整理与历史验证摘要](history/2026-11-20/process.md) |
| 日期未确认 | [原始未注明日期的记录](history/undated/process.md) |

日期取自原记录标题，保留其时区说明；不通过文件修改时间、版本号或前后条目推算事件日期。README 快照以归档日期命名，不表示其中所有事件发生在当天。完整规则及原始条目顺序见[历史索引](history/README.md)。根目录 [process.md](../process.md) 只保留当前进度，后续详细记录直接写入当天 `history/YYYY-MM-DD/process.md`。

## 文件维护

- `guides/` 保存持续维护的当前说明；历史开发记录放入 `history/YYYY-MM-DD/`。
- 无明确记录日期的内容放入 `history/undated/`，确认日期后再归档。
- 正式报告保留在 `reports/YYYY-MM-DD-topic/`，正文、图表和来源清单共同保存。
- 根目录旧 `tests/`、`materials/`、`build-host/`、`artifacts/`、`validation/` 已于2026-11-20按授权移入回收站；保留 `upper/tests/`。validation 的可复用结论见当日过程记录，不复制密钥与个人网络快照。
- 当前应用在 `build/firmware/robotfish.bin`；`build.ps1` 成功编译后将副本导出至 `build/releases/`，不重建根 `artifacts/`。旧缓存保留在 `build/archive/`。历史记录中的旧路径保持原语境，文件可能已不存在。
- 报告来源清单中的路径和摘要代表报告生成时的证据，不随目录整理改写。
