# 评估与实验工具

这里集中放置不属于日常部署入口的离线评估工具：

- OpenLORIS 下载、回放、轨迹与回环分析；
- Ceres/GTSAM、鲁棒核、scan-overlap 和多序列消融；
- LiDAR loop candidate/shadow matching 评估；
- 动态障碍模型对比；
- 指令解析、LoRA/Q8、离线模型与现场语音报告。

稳定演示只需要 `scripts/acceptance_test.sh` 的公开模式。评估模式由
`tools/acceptance/catalog.py` 注册并在 `--help-all` 中展示；底层 handler 在
`acceptance_handlers.sh`。评估脚本可以直接运行，例如：

```bash
python3 tools/evaluation/evaluate_slam_trajectory.py --help
python3 tools/evaluation/evaluate_instruction_parser.py --help
```

这些工具生成的是实验/证据报告，不应被误写成真实麦克风、Gazebo 或实机验收结果。
