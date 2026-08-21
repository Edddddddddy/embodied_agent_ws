# 06 LLM 动作解析与安全边界

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_online_agent/embodied_online_agent/command_fallback.py:29` | `parse_fallback_action` | 确定性语义优先级 |
| `src/embodied_online_agent/embodied_online_agent/command_fallback.py:90` | `should_block_model_actions` | 否定/疑问/危险阻断 |
| `src/embodied_agent_cpp/src/action_validator.cpp:10` | `ActionValidator::validate` | C++ schema 与白名单 |
| `src/embodied_agent_cpp/src/action_validator.cpp:105` | `require_exact_keys` | 精确字段集合 |
| `src/embodied_agent_cpp/src/robot_command_adapter.cpp:10` | `RobotCommandAdapter::convert` | 规范 JSON 到 typed command |
| `src/embodied_agent_cpp/src/action_guard_node.cpp:94` | `ActionGuardNode::on_candidate` | Lifecycle 信任出口 |

## 1. 威胁模型

LLM 输出是不可信文本。即使 system prompt 规定了 JSON、速度范围和动作白名单，也可能发生：

- 格式不完整或额外字段；
- 幻觉出不存在动作；
- 数值越界、NaN 或错误类型；
- 用户在疑问句中提到动作，模型误执行；
- prompt injection 要求绕过限制；
- Python 层 bug 或进程被替换。

所以安全不能只靠 prompt，也不能只靠 Python parser。最终接近执行器的一侧必须独立验证。

## 2. 多层决策链

```text
用户文本
  -> WakeWordGate
  -> LLM tagged protocol
  -> deterministic fallback / semantic block
  -> /agent/action_candidate (不可信 JSON)
  -> C++ ActionGuard
     -> JSON parse
     -> exact schema
     -> whitelist/type/range
     -> RobotCommand typed message
  -> Action server 再检查支持的 goal
  -> controller speed/safety clamp
```

这是 defense in depth：上层负责语义，下层负责可执行安全，控制器负责动态环境安全。

## 3. 确定性 fallback 仲裁

`parse_fallback_action()` 对有限明确命令使用规则：前进、后退、左右转、停止、模式、灯和挥手。

优先顺序很重要：

1. 退出自动/手动模式；
2. 自动避障/沿墙；
3. “别动/不要动”；
4. 语义阻断；
5. stop；
6. move/turn 等动作。

例如“不要向前走”先命中 `should_block_model_actions()`，不会因为包含“向前”而执行 move。

规则 parser 的优势是可预测、可测试；缺点是覆盖有限、中文表达变化多。它被定义为有限机器人指令的确定性兜底，不是通用 NLU。

## 4. 模型动作阻断

`should_block_model_actions()` 对以下文本阻断模型 action：

- 疑问标记：“为什么”“如何”“吗”等；
- 否定：“不要”“别”“禁止”；
- 一边/同时/高速旋转等复合危险表达。

仲裁逻辑：

```python
fallback = parse_fallback_action(user_text)
if fallback:
    publish(fallback)              # 明确命令由规则优先
elif should_block_model_actions(user_text):
    discard(model_actions)         # 语义危险时不信模型
else:
    publish(model_actions)
```

这降低误动作，但不是完整自然语言安全模型。未知危险表达仍可能漏过，最终还要依赖 Guard 白名单和控制层急停。

## 5. C++ ActionValidator

`ActionValidator::validate()` 是主要信任边界。

### 5.1 结构验证

- 整体必须是 JSON object；
- `name` 必须是 string；
- 缺少 `arguments` 时补空 object；
- arguments 必须是 object。

### 5.2 精确 schema

`require_exact_keys()` 比较 expected set 与 actual set。move 必须恰好有：

```text
{"linear_x", "duration_s"}
```

缺字段拒绝，额外字段也拒绝。这避免攻击者用未被下游理解的字段产生语义分歧。

### 5.3 白名单与限幅

| 动作 | 校验 | 处理 |
|---|---|---|
| stop | 无参数 | 接受 |
| move | 数字 linear_x/duration | clamp 到 `[-0.5,0.5]`、`[0,10]` |
| turn | 数字 angular_z/duration | clamp 到 `[-1.5,1.5]`、`[0,10]` |
| wave | 数字 count | clamp 1..5 后四舍五入 |
| set_led | string color | 固定颜色集合 |
| set_mode | string mode | manual/avoidance/wall-following |

这里对越界运动参数选择 clamp，而不是 reject。优点是系统仍能安全执行；缺点是用户请求与实际执行值不同。对未知动作和枚举则拒绝，因为无法安全降级。

### 5.4 非有限数

JSON parser 对标准 JSON 不接受裸 `NaN`，Action server 的 `supported_action_goal()` 还用 `std::isfinite()` 检查 typed 数值。控制器再次 clamp，形成额外保护。

## 6. ActionGuard 生命周期

Guard 只有 active 时处理候选：

```cpp
if (!is_active()) return;
auto result = adapter_.convert(message->data, command_id, "agent");
if (!result.valid) publish rejection;
else publish legacy and typed trusted command;
```

每条命令得到 `guard-N` ID 和 source，并在 typed message 上加时间戳。command ID 用于跨 feedback/result 关联。

Guard 同时发布 legacy 和 typed 是迁移兼容。默认 typed 仿真链不会消费 legacy Topic；硬件控制器仍消费 legacy trusted JSON，并再次执行 `ActionValidator`。

## 7. 为什么不能直接发布 `/cmd_vel`

若 LLM 直接发布速度：

- 没有动作白名单和 schema；
- 没有任务 ID、进度、取消和结果；
- 没有统一超时；
- 无法把新障碍映射成 blocked result；
- 模型持续输出可能覆盖急停；
- 仿真和硬件需要重复安全逻辑。

正确分层是“模型表达意图，Guard 形成可信命令，执行层决定如何安全实现”。

## 8. 动态安全不属于 Guard

Guard 只能判断“0.2 m/s、1 秒”在静态 schema 上合法，无法知道机器人前方此刻是否有障碍。LaserScan freshness、emergency distance 和 collision stop 属于执行层。把所有安全塞进 Guard 会让它依赖传感器和底盘状态，破坏可复用边界。

## 9. 测试证据

- `test_command_fallback.py`：明确动作、否定、疑问、复合危险文本。
- `test_protocol.py`：标签跨 token、非法 JSON、句子切块。
- `test_action_validator.cpp`：越界 clamp、未知动作、缺/多字段、颜色和模式。
- `test_robot_command_adapter.cpp`：JSON 到 typed 字段映射。

这些证明规则和结构安全；不证明任意自然语言都不会误触发，真实语音场景仍需安全 case 集和物理急停。

## 10. 面试回答模板

**问题：模型为什么不能直接控制机器人？**

模型输出本质上是不可信文本，prompt 不是安全边界。项目先用标签 parser 保证 action 必须是完整 JSON，再用确定性规则处理有限明确命令，并阻断否定、疑问和复合危险表达。候选动作只发布到 `/agent/action_candidate`，C++ Guard 独立完成 JSON 结构、精确字段、类型、白名单和速度/时长限幅，再转成强类型 `RobotCommand`。Action server 还会检查 typed goal，控制器根据实时 LaserScan 做传感器超时和前方急停。这样模型只表达意图，不能绕过任务状态机直接写 `/cmd_vel`。边界是语义规则不是完整安全 NLU，所以仍需物理急停和真实场景测试。

## 11. 自测

1. “机器人能不能向前走”为什么应被阻断？
2. 精确 key 校验比只检查必需字段多防住什么？
3. 为什么速度越界选择 clamp，而未知 mode 选择 reject？
4. Guard 能否判断前方 15 cm 有障碍？为什么？
5. 硬件控制器为什么在可信 Topic 后仍再次 validate？
