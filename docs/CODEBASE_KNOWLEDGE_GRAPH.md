# 项目代码知识图谱使用说明

本项目使用第三方工具 `codebase-memory-mcp` 作为轻量代码知识图谱/代码记忆工具。它会把仓库中的文件、类、函数、调用关系、测试关系、ROS topic 等信息索引成本地图谱，方便后续开发时快速定位代码，减少反复全仓阅读带来的 token 消耗。

## 当前安装状态

- WSL 路径：`/home/ubuntu/.local/bin/codebase-memory-mcp`
- 当前版本：`0.8.1`
- 当前项目名：`home-ubuntu-embodied_agent_ws`
- 当前仓库路径：`/home/ubuntu/embodied_agent_ws`

## 更新索引

每次完成一轮较大的代码修改后，建议重新索引：

```bash
wsl -d Ubuntu-24.04 --% bash -lc "cd /home/ubuntu/embodied_agent_ws && /home/ubuntu/.local/bin/codebase-memory-mcp cli index_repository '{\"repo_path\":\"/home/ubuntu/embodied_agent_ws\"}'"
```

索引成功时会输出类似：

```text
{"project":"home-ubuntu-embodied_agent_ws","status":"indexed","nodes":2629,"edges":7739}
```

## 常用查询

列出已索引项目：

```bash
wsl -d Ubuntu-24.04 --% bash -lc "/home/ubuntu/.local/bin/codebase-memory-mcp cli list_projects '{}'"
```

搜索某个技术点对应的代码：

```bash
wsl -d Ubuntu-24.04 --% bash -lc "/home/ubuntu/.local/bin/codebase-memory-mcp cli search_graph '{\"project\":\"home-ubuntu-embodied_agent_ws\",\"query\":\"voice navigation action queue nav2 executor\",\"limit\":8}'"
```

查看整体架构摘要：

```bash
wsl -d Ubuntu-24.04 --% bash -lc "/home/ubuntu/.local/bin/codebase-memory-mcp cli get_architecture '{\"project\":\"home-ubuntu-embodied_agent_ws\",\"aspects\":[\"all\"]}'"
```

## 开发时怎么用

建议在这些场景先查知识图谱，再读源码：

1. 想知道“某个功能在哪里实现”：用 `search_graph` 搜中文/英文关键词。
2. 想梳理“入口节点、热点函数、模块边界”：用 `get_architecture`。
3. 想避免误改重复逻辑：先搜相似关键词，例如 `command queue`、`ActionGuard`、`Nav2RobotExecutor`。
4. 做阶段总结或汇报材料：用图谱输出的 entry points、routes、hotspots 辅助整理代码讲解路径。

## PowerShell 调 WSL 注意事项

涉及 JSON 参数时，优先使用 `--%`，否则 PowerShell 容易提前解析引号、冒号、逗号：

```bash
wsl -d Ubuntu-24.04 --% bash -lc "..."
```

如果命令仍然复杂，优先改成单行命令，避免 here-doc 被 PowerShell 改写。
