# syntax=docker/dockerfile:1.7@sha256:a57df69d0ea827fb7266491f2813635de6f17269be881f696fbfdf2d83dda33e

FROM ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f AS dependencies

ARG REFRESH_ROSDEP=false
ENV ROS_DISTRO=jazzy \
    DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONUNBUFFERED=1
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN printf '%s\n' \
      'Acquire::Retries "5";' \
      'Acquire::http::Timeout "30";' \
      'Acquire::https::Timeout "30";' \
      'Acquire::http::Pipeline-Depth "0";' \
      > /etc/apt/apt.conf.d/80-embodied-network \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      python3-pytest \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/context
COPY requirements.txt ./requirements.txt
# 依赖层只接收 package manifest，业务源码变化不会让数十分钟的 rosdep/pip 层失效。
COPY src/embodied_agent_interfaces/package.xml ./src/embodied_agent_interfaces/package.xml
COPY src/embodied_agent_core/package.xml ./src/embodied_agent_core/package.xml
COPY src/embodied_agent_middleware/package.xml ./src/embodied_agent_middleware/package.xml
COPY src/embodied_agent_bringup/package.xml ./src/embodied_agent_bringup/package.xml
COPY src/embodied_voice_frontend/package.xml ./src/embodied_voice_frontend/package.xml
COPY src/embodied_online_agent/package.xml ./src/embodied_online_agent/package.xml
COPY src/embodied_offline_agent/package.xml ./src/embodied_offline_agent/package.xml
COPY src/embodied_agent_cpp/package.xml ./src/embodied_agent_cpp/package.xml
COPY src/embodied_navigation/package.xml ./src/embodied_navigation/package.xml
COPY src/embodied_slam/package.xml ./src/embodied_slam/package.xml
COPY src/embodied_slam_tools/package.xml ./src/embodied_slam_tools/package.xml
COPY src/embodied_simulation/package.xml ./src/embodied_simulation/package.xml
# 固定的官方 ROS 镜像已经包含 rosdep cache，默认直接使用，避免新环境因 GitHub Raw
# 短暂断流而无法构建。需要刷新时可传 REFRESH_ROSDEP=true，并保留有界重试与严格失败。
# Jazzy 基础镜像已提供 ament_python，而 rosdep 在 Noble 上没有同名 key；只跳过该已知 buildtool。
RUN if [[ "${REFRESH_ROSDEP}" == true ]]; then \
      for attempt in 1 2 3; do \
        if rosdep update; then break; fi; \
        if [[ "${attempt}" == 3 ]]; then exit 1; fi; \
        rm -rf /root/.ros/rosdep/sources.cache; \
        sleep "$((attempt * 5))"; \
      done; \
    else \
      test -s /root/.ros/rosdep/sources.cache/index; \
    fi \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" \
         --skip-keys ament_python -y \
    && python3 -m pip install --break-system-packages --no-cache-dir -r requirements.txt \
    && rm -rf /var/lib/apt/lists/* /root/.cache/pip /tmp/context/src

FROM dependencies AS build
ARG BUILD_PARALLEL_LEVEL=2
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_PARALLEL_LEVEL} \
    MAKEFLAGS=-j${BUILD_PARALLEL_LEVEL}
WORKDIR /workspace
COPY src ./src
# Colcon 顺序构建只限制“包级并发”；CMake 仍可能在单包内占满 8 GB WSL。
# 双层限流使本机和 CI 的峰值内存可预测；运行镜像使用普通 install，不能复制 symlink install。
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

FROM build AS test
WORKDIR /workspace
COPY . .
ENV WORKSPACE=/workspace \
    EMBODIED_INSTALL_PREFIX=/workspace/install \
    EMBODIED_CORE_REBUILD=false
RUN install -m 0755 docker/entrypoint /opt/embodied-entrypoint \
    && install -D -m 0755 docker/run-container-tests /opt/embodied/bin/run-container-tests
ENTRYPOINT ["/opt/embodied-entrypoint"]
CMD ["/opt/embodied/bin/run-container-tests"]

FROM dependencies AS runtime
ARG BUILD_DATE=1970-01-01T00:00:00Z
ARG VCS_REF=unknown
ARG VERSION=dev
ARG SOURCE_URL=https://github.com/Edddddddddy/embodied_agent_ws
ARG RUNTIME_UID=10001
ARG RUNTIME_GID=10001
LABEL org.opencontainers.image.title="Embodied Voice Agent for ROS 2" \
      org.opencontainers.image.description="ROS 2 Jazzy voice agent, typed control and navigation runtime" \
      org.opencontainers.image.source="${SOURCE_URL}" \
      org.opencontainers.image.revision="${VCS_REF}" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.created="${BUILD_DATE}" \
      org.opencontainers.image.licenses="Apache-2.0"

# Noble 基础镜像通常已有 ubuntu:1000；独立服务 UID 避免与基础用户冲突。
RUN groupadd --gid "${RUNTIME_GID}" embodied \
    && useradd --uid "${RUNTIME_UID}" --gid "${RUNTIME_GID}" --create-home --shell /bin/bash embodied \
    && install -d -o embodied -g embodied /opt/embodied /opt/embodied/bin /home/embodied/.ros
COPY --from=build --chown=embodied:embodied /workspace/install /opt/embodied/install
COPY docker/entrypoint /opt/embodied/bin/container-entrypoint
COPY docker/runtime-smoke /opt/embodied/bin/runtime-smoke
COPY LICENSE /licenses/LICENSE
RUN chmod 0755 /opt/embodied/bin/container-entrypoint /opt/embodied/bin/runtime-smoke

ENV WORKSPACE=/opt/embodied \
    EMBODIED_INSTALL_PREFIX=/opt/embodied/install
USER embodied
WORKDIR /home/embodied
ENTRYPOINT ["/opt/embodied/bin/container-entrypoint"]
CMD ["/opt/embodied/bin/runtime-smoke"]
