# Docker 与交付流程

本页定义项目的干净环境容器门禁和版本镜像发布契约。容器目标是证明一个 Ubuntu/ROS 环境可以重复执行
依赖解析、全部自有 ROS 2 包构建和核心测试，并生成不携带工作区 checkout/build/test 树的运行镜像。
apt、rosdep 和部分 Python 依赖会随上游仓库演进，因此“严格复现”专指已经发布的不可变镜像摘要。

## 镜像分层

```text
ros:jazzy-ros-base-noble
  └─ dependencies  apt/rosdep/Python provider 依赖
       ├─ build    全工作区 colcon build
       │    └─ test  仓库、Agent、C++ 与仿真核心测试
       └─ runtime  只复制 build 的 install 产物，以非 root 用户运行
```

`runtime` 不携带工作区 checkout、`build/`、测试、模型、日志或 `.env`。实际 API key 和模型路径必须在部署时注入，
不得使用 Docker `ARG` 或 `COPY` 写入镜像。

基础镜像通过 OCI digest 固定，并直接使用该镜像随附的 rosdep cache。日常构建不依赖 GitHub Raw
实时可用；确需刷新依赖索引时可执行 `docker build --build-arg REFRESH_ROSDEP=true ...`，刷新失败会在
三次有界重试后严格终止。

## 本地构建与验收

前置条件仅为 Docker Engine 和 Compose 插件。第一次构建需要联网下载 ROS/apt/pip 依赖：

```bash
docker compose config --quiet
docker compose build test
docker compose run --rm test
docker compose build runtime-smoke
docker compose run --rm runtime-smoke
```

`test` 服务先在镜像构建阶段完成整个工作区的 Colcon build，再在容器运行阶段调用
`scripts/run_core_tests.sh`。最后一个命令应输出：

```text
PASS: embodied agent runtime image is loadable
```

需要单独构建发布形态时执行：

```bash
VCS_REF="$(git rev-parse HEAD)" VERSION=dev docker compose build runtime-smoke
docker image inspect embodied-agent:runtime \
  --format '{{ index .Config.Labels "org.opencontainers.image.revision" }}'
```

## GitHub Actions 发布

`.github/workflows/container.yml` 的权限和触发边界如下：

1. PR 以及 `dev/main` push 只构建测试镜像，并在容器内运行 Colcon 和核心测试。
2. 只有指向 `main` 历史、格式为 `vMAJOR.MINOR.PATCH` 的稳定 SemVer 标签能通过发布校验并实际推送镜像。
3. 发布 job 先按 digest 推送候选镜像并运行 smoke；通过后才把同一 digest 提升为版本、完整提交 SHA 和 `latest` 标签。
4. GitHub Actions 记录不可变镜像摘要、源码 revision、工作流链接以及
   `container-release-manifest` artifact；OCI provenance 和 SBOM 随镜像发布。

里程碑合并到 `main` 后，可由维护者创建类似 `v0.5.0` 的版本标签。标签发布完成后，
使用镜像摘要而不是可变标签进行严格复现：

```bash
docker pull ghcr.io/eddddddddddy/embodied_agent_ws@sha256:<镜像摘要>
docker inspect ghcr.io/eddddddddddy/embodied_agent_ws@sha256:<镜像摘要>
```

可追溯关系为：Git 标签 → Git commit → Actions run → GHCR 镜像摘要 → OCI source/revision/version labels。

## 能力边界

- 容器门禁覆盖 headless ROS 2 全工作区构建、核心 pytest/GTest 和安装产物 smoke。
- WSLg 麦克风、RViz、Gazebo GUI、GPU 以及真实大模型文件仍按本机验收流程运行，不由无设备 CI 容器代替。
- `runtime` 是组件运行基础镜像，并不默认启动一个固定 Agent 场景；部署者应通过 Compose override、
  Kubernetes 或 `docker run ... ros2 launch ...` 选择在线/离线和仿真拓扑。
