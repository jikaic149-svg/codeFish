# RoboFish Partner Lab 使用说明


## 进入页面

1. 连接机器鱼热点 `RoboFish-ND1`，默认热点密码为 `12345678`。
2. 浏览器访问 ``http://192.168.4.1/gpio 进入驾驶界面。
3. 点击 `PARTNER LAB` 进入动作脚本页面，也可以直接访问 `http://192.168.4.1/partner`。

## 示例脚本

```json
{
  "name": "Water tank route",
  "repeat": 2,
  "steps": [
    {"action": "mode_bionic"},
    {"action": "forward", "power": 55, "duration_ms": 3000},
    {"action": "stop", "duration_ms": 800},
    {"action": "left", "power": 45, "strength": 40, "duration_ms": 1200},
    {"action": "pitch", "angle": 110, "duration_ms": 1000},
    {"action": "stop", "duration_ms": 500}
  ]
}
```

先点击“校验并保存”，通过后再点击“运行”。保存后的脚本断电后仍会保留，但开机不会自动运行。

## 允许的动作

- `mode_bionic`：仿生尾鳍模式
- `mode_prop`：螺旋桨模式
- `mode_hybrid`：混合模式
- `forward`、`backward`、`left`、`right`、`stop`
- `pitch`：胸鳍俯仰，需要 `angle`
- `pump_on`、`pump_off`

纯仿生模式没有主动倒车能力，`backward` 应配合 `mode_prop` 或 `mode_hybrid` 使用。

## 固件强制限制

- `power`：20–70
- `strength`：10–70
- `angle`：45–135°
- 普通动作最长30秒
- 水泵单次开启100毫秒–10秒
- 最多32个步骤
- `repeat`：1–10次
- 脚本总执行时间最长5分钟

每个步骤开始前，固件都会先停止推进并关闭水泵。因此每一步是独立动作，`pitch` 不会自动保持上一步的前进状态。

任何手动驾驶操作都会中止脚本。网络失联触发安全保护时，脚本会被无条件终止，机器鱼执行原有的停止推进、关闭水泵和上浮保护。


