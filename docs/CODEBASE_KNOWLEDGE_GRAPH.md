# 项目代码知识图谱使用说明

本项目使用 `codebase-memory-mcp` 为 ROS2/C++/Python 混合代码建立本地知识图谱，目标是在后续开发中减少反复 `rg/read` 全仓搜索带来的 token 消耗，并更快定位“某个功能在哪些文件、哪些函数互相调用、改动会影响哪些模块”。

## 当前安装状态

- 工具位置：`/home/ubuntu/.local/bin/codebase-memory-mcp`
- 当前版本：`0.8.1`
- 本项目索引名：`home-ubuntu-embodied_agent_ws`
- 缓存位置：`~/.cache/codebase-memory-mcp/`
- 当前索引规模：约 `2773` 个节点、`8167` 条边

> 说明：索引缓存不提交到 Git。它是本地开发辅助数据，业务代码仍以仓库为准。

## 常用命令

在 WSL 中执行：

```bash
cd /home/ubuntu/embodied_agent_ws

# 重建/增量更新本项目图谱
/home/ubuntu/.local/bin/codebase-memory-mcp cli index_repository \
  '{"repo_path":"/home/ubuntu/embodied_agent_ws"}'

# 查看已经索引的项目
/home/ubuntu/.local/bin/codebase-memory-mcp cli list_projects

# 查询整体架构热点
/home/ubuntu/.local/bin/codebase-memory-mcp cli get_architecture \
  '{"project":"home-ubuntu-embodied_agent_ws","aspects":["overview","hotspots"]}'

# 按语义/关键词查相关代码节点
/home/ubuntu/.local/bin/codebase-memory-mcp cli search_graph \
  '{"project":"home-ubuntu-embodied_agent_ws","query":"continuous voice command queue nav2","limit":8}'

# 查询某个符号的调用关系
/home/ubuntu/.local/bin/codebase-memory-mcp cli trace_path \
  '{"project":"home-ubuntu-embodied_agent_ws","symbol":"SequentialActionPublisher.publish","direction":"inbound","max_depth":2}'
```

## Codex 开发约定

后续进行结构性问题分析时，优先使用代码知识图谱：

1. 先用 `get_architecture` 或 `search_graph` 快速定位模块。
2. 再用 `trace_path` 查调用链和影响范围。
3. 只有在需要确认具体实现细节、修改代码、或图谱结果不足时，再用 `rg` / `sed` / 直接打开文件。
4. 每次较大开发阶段结束后，重新运行 `index_repository`，让图谱跟上代码变化。

## 适合用图谱回答的问题

- “连续语音队列相关逻辑在哪些文件？”
- “`/agent/action_candidate` 是从哪里发布的？”
- “Nav2 目标点导航链路从脚本到 ROS Action 经过哪些模块？”
- “修改 `SequentialActionPublisher` 会影响哪些测试？”
- “哪些函数是高 fan-in 的架构热点？”

## 不适合完全依赖图谱的问题

- 真实麦克风、Gazebo、Nav2 的运行时问题：仍需要看日志和实际 topic。
- 参数调优和性能瓶颈：仍需要验收脚本、`ros2 topic echo`、`ros2 action`、`colcon test` 验证。
- 未保存到文件的临时状态：图谱只看文件系统中的代码。
