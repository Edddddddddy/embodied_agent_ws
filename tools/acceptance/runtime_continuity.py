"""持久 Gazebo 会话的进程选择、阶段快照与连续性判定。"""

from __future__ import annotations

import os
import re
import shlex
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping
from uuid import UUID

from tools.acceptance.runtime_identity import RuntimeCheckpoint


RUNTIME_CONTINUITY_SCHEMA_VERSION = 1
_CHECKPOINT_FIELDS = frozenset({"label", "wall_ns", "roles"})
_IDENTITY_FIELDS = (
    "pid",
    "start_ticks",
    "boot_id",
    "executable",
    "argv_sha256",
)
_IDENTITY_FIELD_SET = frozenset(_IDENTITY_FIELDS)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_SUPPORTED_AGENT_MODES = frozenset({"offline", "online"})


class RuntimeRoleSelectionError(RuntimeError):
    """运行时角色缺失或不唯一，无法形成可信的连续性证据。"""


@dataclass(frozen=True, slots=True)
class ProcessCandidate:
    """从 procfs 读取的最小进程视图；只保留角色选择所需字段。"""

    pid: int
    executable: str
    argv: tuple[str, ...]
    environment: Mapping[str, str]


def _read_environment(path: Path) -> dict[str, str]:
    """仅读取隔离标识，避免把模型密钥等环境变量带入证据。"""

    selected: dict[str, str] = {}
    for raw_entry in path.read_bytes().split(b"\0"):
        if b"=" not in raw_entry:
            continue
        raw_key, raw_value = raw_entry.split(b"=", 1)
        key = raw_key.decode("utf-8", errors="replace")
        if key in {"ROS_DOMAIN_ID", "GZ_PARTITION"}:
            selected[key] = raw_value.decode("utf-8", errors="replace")
    return selected


def scan_process_candidates(
    *,
    proc_root: Path = Path("/proc"),
) -> tuple[ProcessCandidate, ...]:
    """扫描当前用户可审计的活进程。

    验收只从相同 UID 的进程中选择角色，避免主机上其他用户的仿真被误判为
    本次会话。读取失败通常表示进程正退出，直接跳过并由唯一性门禁处理。
    """

    candidates: list[ProcessCandidate] = []
    current_uid = os.getuid()
    for process_dir in sorted(
        (path for path in proc_root.iterdir() if path.name.isdigit()),
        key=lambda path: int(path.name),
    ):
        try:
            if process_dir.stat().st_uid != current_uid:
                continue
            argv = tuple(
                item.decode("utf-8", errors="replace")
                for item in (process_dir / "cmdline").read_bytes().split(b"\0")
                if item
            )
            if not argv:
                continue
            executable = str((process_dir / "exe").resolve(strict=True))
            environment = _read_environment(process_dir / "environ")
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            OSError,
        ):
            continue
        candidates.append(
            ProcessCandidate(
                pid=int(process_dir.name),
                executable=executable,
                argv=argv,
                environment=environment,
            )
        )
    return tuple(candidates)


def _executable_name(candidate: ProcessCandidate) -> str:
    return Path(candidate.executable).name


def _is_robot_state_publisher(candidate: ProcessCandidate) -> bool:
    return _executable_name(candidate) == "robot_state_publisher"


def _is_rviz(candidate: ProcessCandidate) -> bool:
    return _executable_name(candidate) == "rviz2"


def _is_agent(candidate: ProcessCandidate, agent_mode: str) -> bool:
    """只匹配真正的 Agent 可执行入口，不扫描父进程的任意参数文本。

    Python console script 可能以两种形式出现在 procfs：setuptools 入口直接
    位于 argv[0]，或 Linux shebang 把 ``python3`` 放在 argv[0]、把入口脚本
    放在 argv[1]。只允许解释器后的第一个脚本位匹配，不能在完整参数列表中
    搜索；否则 launch 父进程携带 ``mode:=offline_agent`` 时会制造假证据。
    """

    expected_name = f"{agent_mode}_agent"
    argv0_name = Path(candidate.argv[0]).name if candidate.argv else ""
    executable_name = _executable_name(candidate)
    if executable_name == expected_name or argv0_name == expected_name:
        return True
    if not executable_name.startswith("python") or len(candidate.argv) < 2:
        return False
    return Path(candidate.argv[1]).name == expected_name


def _is_gazebo_server(candidate: ProcessCandidate) -> bool:
    executable = _executable_name(candidate)
    raw_arguments = candidate.argv
    if executable.startswith("ruby") and len(raw_arguments) == 1:
        # Harmonic 的 Ruby `gz` wrapper 会在启动后把整个进程标题压成一个
        # argv 项（例如 ``gz sim -r -s world.sdf``）。只为该已知 wrapper
        # 拆词，避免把普通进程中合法的含空格参数误解释成多个参数。
        try:
            raw_arguments = tuple(shlex.split(raw_arguments[0]))
        except ValueError:
            return False
    argument_names = tuple(Path(argument).name for argument in raw_arguments)
    gz_pair = any(
        argument_names[index] in {"gz", "ign"}
        and argument_names[index + 1] in {"sim", "gazebo"}
        for index in range(len(argument_names) - 1)
    )
    server_only = any(
        argument in {"-s", "--server-only"}
        for argument in raw_arguments
    )
    # ros_gz_sim 在 Jazzy/Harmonic 中以 `ruby /usr/bin/gz sim` 运行。
    # 排除携带同样文本的 shell/launch 父进程，确保绑定真正的 Gazebo server。
    if executable.startswith("ruby"):
        return gz_pair and server_only
    if executable == "gzserver":
        return True
    return executable == "gz" and server_only and (
        gz_pair or "sim" in argument_names
    )


def _unique_role(
    role: str,
    candidates: Iterable[ProcessCandidate],
) -> int:
    matches = sorted(candidate.pid for candidate in candidates)
    if len(matches) != 1:
        raise RuntimeRoleSelectionError(
            f"{role} requires exactly one process; found pids={matches}"
        )
    return matches[0]


def select_runtime_role_pids(
    candidates: Iterable[ProcessCandidate],
    *,
    ros_domain_id: str,
    gz_partition: str,
    require_rviz: bool,
    agent_mode: str,
) -> dict[str, int]:
    """按本次会话隔离键唯一选择持久角色。

    进程名只能说明“是什么”，ROS domain/Gazebo partition 才说明“属于哪次
    验收”。两层条件缺一不可，否则残留仿真可能制造连续运行的假象。
    """

    if not ros_domain_id or not gz_partition:
        raise RuntimeRoleSelectionError(
            "ROS_DOMAIN_ID and GZ_PARTITION are required for runtime selection"
        )
    if agent_mode not in _SUPPORTED_AGENT_MODES:
        raise RuntimeRoleSelectionError(
            f"unsupported persistent Agent mode: {agent_mode!r}"
        )
    materialized = tuple(candidates)
    agent_role = f"{agent_mode}_agent"
    selected = {
        agent_role: _unique_role(
            agent_role,
            (
                candidate
                for candidate in materialized
                if _is_agent(candidate, agent_mode)
                and candidate.environment.get("ROS_DOMAIN_ID") == ros_domain_id
            ),
        ),
        "gazebo_server": _unique_role(
            "gazebo_server",
            (
                candidate
                for candidate in materialized
                if _is_gazebo_server(candidate)
                and candidate.environment.get("GZ_PARTITION") == gz_partition
            ),
        ),
        "robot_state_publisher": _unique_role(
            "robot_state_publisher",
            (
                candidate
                for candidate in materialized
                if _is_robot_state_publisher(candidate)
                and candidate.environment.get("ROS_DOMAIN_ID") == ros_domain_id
            ),
        ),
    }
    if require_rviz:
        selected["rviz"] = _unique_role(
            "rviz",
            (
                candidate
                for candidate in materialized
                if _is_rviz(candidate)
                and candidate.environment.get("ROS_DOMAIN_ID") == ros_domain_id
            ),
        )
    return dict(sorted(selected.items()))


def capture_persistent_runtime_checkpoint(
    label: str,
    *,
    ros_domain_id: str,
    gz_partition: str,
    require_rviz: bool,
    agent_mode: str,
) -> RuntimeCheckpoint:
    """定位会话角色后，立即绑定 PID/starttime/boot-id/argv 摘要。"""

    role_pids = select_runtime_role_pids(
        scan_process_candidates(),
        ros_domain_id=ros_domain_id,
        gz_partition=gz_partition,
        require_rviz=require_rviz,
        agent_mode=agent_mode,
    )
    return RuntimeCheckpoint.capture(label, role_pids)


def _checkpoint_evidence(
    checkpoint: RuntimeCheckpoint | Mapping[str, object],
) -> Mapping[str, object]:
    return (
        checkpoint.as_evidence()
        if isinstance(checkpoint, RuntimeCheckpoint)
        else checkpoint
    )


def _normalize_required_roles(required_roles: Iterable[str]) -> tuple[str, ...]:
    """把 verifier 配置变成唯一且稳定的角色集合。"""

    materialized = tuple(required_roles)
    if (
        not materialized
        or any(
            not isinstance(role, str) or not role or role.strip() != role
            for role in materialized
        )
        or len(set(materialized)) != len(materialized)
    ):
        raise ValueError("required runtime roles must be unique non-empty names")
    return tuple(sorted(materialized))


def _plain_positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _checkpoint_schema_valid(checkpoint: Mapping[str, object]) -> bool:
    """验证阶段快照外层 schema；标签语义和时间顺序由独立 gate 判断。"""

    if set(checkpoint) != _CHECKPOINT_FIELDS:
        return False
    label = checkpoint.get("label")
    wall_ns = checkpoint.get("wall_ns")
    roles = checkpoint.get("roles")
    return (
        isinstance(label, str)
        and bool(label)
        and _plain_positive_int(wall_ns)
        and isinstance(roles, Mapping)
        and bool(roles)
        and all(
            isinstance(role, str)
            and bool(role)
            and isinstance(identity, Mapping)
            for role, identity in roles.items()
        )
    )


def _identity_schema_valid(identity: object) -> bool:
    """拒绝字段缺失、类型混淆和额外字段制造的“相等但不可信”身份。"""

    if not isinstance(identity, Mapping) or set(identity) != _IDENTITY_FIELD_SET:
        return False
    pid = identity.get("pid")
    start_ticks = identity.get("start_ticks")
    boot_id = identity.get("boot_id")
    executable = identity.get("executable")
    argv_sha256 = identity.get("argv_sha256")
    if not (
        _plain_positive_int(pid)
        and _plain_positive_int(start_ticks)
        and isinstance(boot_id, str)
        and isinstance(executable, str)
        and Path(executable).is_absolute()
        and isinstance(argv_sha256, str)
        and _SHA256_PATTERN.fullmatch(argv_sha256) is not None
    ):
        return False
    try:
        return str(UUID(boot_id)) == boot_id.lower()
    except ValueError:
        return False


def _same_process_identity(
    mapping_identity: object,
    navigation_identity: object,
) -> bool:
    """逐字段比较已验证身份，明确绑定 PID 复用防护字段。"""

    if not (
        _identity_schema_valid(mapping_identity)
        and _identity_schema_valid(navigation_identity)
    ):
        return False
    assert isinstance(mapping_identity, Mapping)
    assert isinstance(navigation_identity, Mapping)
    return all(
        mapping_identity[field] == navigation_identity[field]
        for field in _IDENTITY_FIELDS
    )


def build_runtime_continuity_evidence(
    mapping_ready: RuntimeCheckpoint | Mapping[str, object],
    navigation_ready: RuntimeCheckpoint | Mapping[str, object],
    *,
    required_roles: Iterable[str],
) -> dict[str, object]:
    """严格验证两个阶段边界，并生成可被二次重算的附加证据。

    两份字典相等并不足以证明连续性：错误标签、倒序时间或同步漂移后的字段
    也可能完全相等。因此先验证 schema/角色/时序，再逐字段比较内核身份。
    """

    mapping = _checkpoint_evidence(mapping_ready)
    navigation = _checkpoint_evidence(navigation_ready)
    required = _normalize_required_roles(required_roles)
    mapping_schema_valid = (
        isinstance(mapping, Mapping) and _checkpoint_schema_valid(mapping)
    )
    navigation_schema_valid = (
        isinstance(navigation, Mapping) and _checkpoint_schema_valid(navigation)
    )
    mapping_roles = mapping.get("roles")
    navigation_roles = navigation.get("roles")
    roles_present = (
        mapping_schema_valid
        and navigation_schema_valid
        and isinstance(mapping_roles, Mapping)
        and isinstance(navigation_roles, Mapping)
        and all(
            role in mapping_roles and role in navigation_roles
            for role in required
        )
    )
    role_sets_exact = bool(
        roles_present
        and set(mapping_roles) == set(required)
        and set(navigation_roles) == set(required)
    )
    checks: dict[str, bool] = {
        "checkpoint_schema_valid": bool(
            mapping_schema_valid and navigation_schema_valid
        ),
        "checkpoint_labels_valid": bool(
            mapping_schema_valid
            and navigation_schema_valid
            and mapping.get("label") == "mapping_ready"
            and navigation.get("label") == "navigation_ready"
        ),
        "checkpoint_time_ordered": bool(
            mapping_schema_valid
            and navigation_schema_valid
            and mapping.get("wall_ns") < navigation.get("wall_ns")
        ),
        "required_roles_present": bool(roles_present),
        "role_sets_exact": role_sets_exact,
    }
    for role in required:
        identity_schema_valid = bool(
            roles_present
            and _identity_schema_valid(mapping_roles[role])
            and _identity_schema_valid(navigation_roles[role])
        )
        checks[f"{role}_identity_schema_valid"] = identity_schema_valid
        checks[f"{role}_unchanged"] = bool(
            identity_schema_valid
            and _same_process_identity(
                mapping_roles[role],
                navigation_roles[role],
            )
        )
    return {
        "schema_version": RUNTIME_CONTINUITY_SCHEMA_VERSION,
        "passed": all(checks.values()),
        "required_roles": list(required),
        "checks": checks,
        "checkpoints": {
            "mapping_ready": dict(mapping),
            "navigation_ready": dict(navigation),
        },
    }
