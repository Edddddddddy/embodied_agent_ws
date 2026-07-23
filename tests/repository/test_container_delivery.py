"""Docker 镜像、Compose 验收与 GHCR 发布流程的仓库合同。"""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml

from repository_test_support import ROOT


DOCKERFILE = ROOT / "Dockerfile"
COMPOSE_FILE = ROOT / "compose.yaml"
CONTAINER_WORKFLOW = ROOT / ".github" / "workflows" / "container.yml"


def _read(path: Path) -> str:
    assert path.is_file(), f"缺少容器交付文件: {path.relative_to(ROOT)}"
    return path.read_text(encoding="utf-8")


def test_bringup_manifest_closes_launch_runtime_dependencies():
    """干净镜像只能依赖清单安装；launch 直接启动的包不能依赖开发机偶然存在。"""

    manifest = ROOT / "src" / "embodied_agent_bringup" / "package.xml"
    root = ET.fromstring(_read(manifest))
    runtime_dependencies = {
        element.text for element in root.findall("exec_depend") if element.text
    }
    assert {
        "embodied_agent_core",
        "embodied_agent_cpp",
        "embodied_voice_frontend",
        "launch",
        "launch_ros",
        "nav2_lifecycle_manager",
    } <= runtime_dependencies


def test_dockerfile_is_multistage_and_separates_test_from_runtime():
    """测试镜像保留源码/构建树，运行镜像只能继承安装层。"""

    text = _read(DOCKERFILE)
    assert text.startswith("# syntax=docker/dockerfile:1.7@sha256:")
    for stage in ("dependencies", "build", "test", "runtime"):
        assert f" AS {stage}" in text

    assert "ros:jazzy-ros-base-noble" in text
    assert "rosdep install --from-paths src --ignore-src" in text
    dependencies_stage = text.split("FROM dependencies AS build", 1)[0]
    build_and_test_stages = text.split("FROM dependencies AS build", 1)[1]
    assert "COPY src ./src" not in dependencies_stage
    assert "COPY src ./src" in build_and_test_stages
    assert "FROM build AS test" in text and "COPY . ." in text.split("FROM build AS test", 1)[1]
    manifests = sorted((ROOT / "src").glob("*/package.xml"))
    assert manifests
    for manifest in manifests:
        relative = manifest.relative_to(ROOT).as_posix()
        assert f"COPY {relative} " in text, f"Docker rosdep layer misses {relative}"
    assert "CMAKE_BUILD_PARALLEL_LEVEL" in text
    assert "colcon build" in text
    assert "FROM build AS test" in text
    assert 'CMD ["/opt/embodied/bin/run-container-tests"]' in text
    assert "--from=build --chown=embodied:embodied /workspace/install /opt/embodied/install" in text
    assert "RUNTIME_UID=10001" in text
    assert "COPY LICENSE /licenses/LICENSE" in text
    assert "USER embodied" in text


def test_container_scripts_expose_reproducible_test_and_runtime_entrypoints():
    test_gate = _read(ROOT / "docker" / "run-container-tests")
    entrypoint = _read(ROOT / "docker" / "entrypoint")
    runtime_smoke = _read(ROOT / "docker" / "runtime-smoke")
    core_gate = _read(ROOT / "scripts" / "run_core_tests.sh")

    assert "scripts/run_core_tests.sh" in test_gate
    assert "set -eo pipefail" in test_gate
    assert "set -eo pipefail" in entrypoint
    assert "set -euo pipefail" not in test_gate
    assert "set -euo pipefail" not in entrypoint
    assert "/opt/ros/${ROS_DISTRO}/setup.bash" in entrypoint
    assert "/opt/embodied/install/setup.bash" in entrypoint
    assert "ros2 pkg prefix embodied_agent_interfaces" in runtime_smoke
    # 运行镜像必须证明自己是非 root，且没有把 checkout/build/test 树带入交付物。
    assert "id -u" in runtime_smoke
    for forbidden_tree in ("/workspace/src", "/workspace/build", "/tmp/context/src"):
        assert forbidden_tree in runtime_smoke
    assert "/licenses/LICENSE" in runtime_smoke
    assert "COLCON_OVERRIDE_ARGS" in core_gate
    assert "colcon build --help" in core_gate
    assert "EMBODIED_CORE_REBUILD" in core_gate
    assert "EMBODIED_CORE_REBUILD=false" in _read(DOCKERFILE)


def test_compose_has_test_and_runtime_smoke_services():
    text = _read(COMPOSE_FILE)
    model = yaml.load(text, Loader=yaml.BaseLoader)

    assert "test:" in text
    assert "target: test" in text
    assert "runtime-smoke:" in text
    assert "target: runtime" in text
    assert text.count("init: true") >= 2
    assert set(model["services"]) == {"test", "runtime-smoke"}
    for service in model["services"].values():
        assert "privileged" not in service
        assert "/var/run/docker.sock" not in str(service)


def test_dockerignore_excludes_local_state_and_credentials():
    ignored = set(_read(ROOT / ".dockerignore").splitlines())

    for required in (
        ".env",
        ".venv",
        "build",
        "install",
        "log",
        "logs",
        "models",
        "third_party",
    ):
        assert required in ignored


def test_container_workflow_tests_every_change_and_only_publishes_version_tags():
    """PR/分支只构建并测试；只有 v* 标签拥有向 GHCR 推送的权限。"""

    text = _read(CONTAINER_WORKFLOW)
    workflow = yaml.load(text, Loader=yaml.BaseLoader)

    assert "pull_request:" in text
    assert "branches: [main, dev]" in text
    assert "tags: [\"v*\"]" in text
    assert "container-test:" in text
    assert "target: test" in text
    assert "push: false" in text
    assert "load: true" in text
    assert "cache-to: type=gha,mode=max,scope=embodied-agent" in text
    assert "docker compose run --rm --no-deps test" in text
    assert "--no-build" not in text
    assert "publish:" in text
    assert "needs: container-test" in text
    assert "startsWith(github.ref, 'refs/tags/v')" in text
    assert "packages: write" in text
    assert "ghcr.io/" in text
    assert "docker/metadata-action@" in text
    assert "target: runtime" in text
    assert "push=true" in text
    assert "push-by-digest=true" in text
    assert 'docker run --rm --init "${IMAGE}@${DIGEST}"' in text
    assert "docker buildx imagetools create" in text
    assert "refusing to overwrite immutable tag" in text
    assert "unable to inspect immutable tag" in text
    assert "promoted immutable tag does not resolve to candidate digest" in text
    assert "Release tags must be stable SemVer" in text
    assert "provenance: mode=max" in text
    assert "sbom: true" in text
    assert "container-release-manifest" in text
    assert workflow["permissions"] == {"contents": "read"}
    jobs = workflow["jobs"]
    assert "permissions" not in jobs["container-test"]
    assert jobs["publish"]["permissions"]["packages"] == "write"
    # publish job 持有 packages:write；第三方 Action 必须锁定不可变提交，而非可移动 major tag。
    uses_lines = [line.split("#", 1)[0].strip() for line in text.splitlines() if "uses:" in line]
    assert uses_lines
    assert all(re.search(r"@[0-9a-f]{40}$", line) for line in uses_lines)


def test_image_build_never_receives_project_credentials():
    dockerfile = _read(DOCKERFILE)
    workflow = _read(CONTAINER_WORKFLOW)

    for forbidden in ("API_KEY", "PASSWORD", "PRIVATE_KEY", "GITHUB_TOKEN"):
        assert forbidden not in dockerfile
    # GITHUB_TOKEN 只允许交给 registry login，不能成为 Docker build secret/arg。
    assert "password: ${{ secrets.GITHUB_TOKEN }}" in workflow
    assert "secret-files:" not in workflow
    assert "secrets:" not in workflow


def test_delivery_document_contains_build_test_and_traceability_commands():
    text = _read(ROOT / "docs" / "deployment" / "CONTAINER_DELIVERY.md")

    assert "docker compose build test" in text
    assert "docker compose run --rm test" in text
    assert "docker compose run --rm runtime-smoke" in text
    assert "v0." in text
    assert "GHCR" in text
    assert "镜像摘要" in text
