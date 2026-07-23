# 证据索引

证据按能力分区。Markdown 是摘要，JSON 是机器事实源；同名文件必须成对。运行时大文件位于 `logs/`，默认不提交 Git。

| 分区 | 内容 | 入口 |
| --- | --- | --- |
| Voice | 麦克风、VAD、ASR、连续会话现场报告 | [voice/README.md](voice/README.md) |
| Offline | Q8/LoRA 对照与离线运行时报告 | [offline/README.md](offline/README.md) |
| SLAM | OpenLORIS、回环、后端消融与新地图报告 | [slam/README.md](slam/README.md) |
| Navigation | AMCL/Nav2、动态障碍与最终停车证据 | [navigation/README.md](navigation/README.md) |

[architecture_facts.md](architecture_facts.md) / `.json` 是由仓库源码生成的结构快照，留在索引根目录，不归入运行能力分区。

## 证据规则

1. 同一配置只保留最新成功摘要；失败报告仅在用于回归时保留。
2. Markdown 与 JSON 是一组，不把相同表格复制到 README。
3. README 只链接入口；阈值在 `docs/TESTING.md`，原理在学习笔记。
4. mock、dry-run、Gazebo、真人语音、公开 bag、实机使用不同 `evidence_kind`。
5. OpenLORIS 的不同 sequence、前端阶段和后端消融不是重复快照；删除任何一组前必须确认没有报告引用它。
6. 运行时地图、音频、模型和大日志不提交 Git，只记录生成命令、哈希和报告路径。
