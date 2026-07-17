# Voice 证据

本分区记录真实麦克风、VAD/ASR endpoint、连续会话和命令队列证据。现场产物默认留在：

- `logs/audio_calibration.json`
- `logs/voice_calibration_report.json`
- `logs/continuous_voice/`

它们只证明运行时声卡、环境和模型版本。音频样本可能包含个人声音，不默认提交仓库；验收命令与判定见 [测试手册](../../TESTING.md)。
