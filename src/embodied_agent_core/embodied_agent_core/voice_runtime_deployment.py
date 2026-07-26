"""VAD→ASR→LLM/RAG→TTS 部署契约与只读健康预检。

该模块不启动模型、不下载权重，也不会调用付费推理 API。它只解释声明式 profile，
选择第一个可用 provider，并把缺依赖、降级和未执行的 endpoint probe 变成结构化证据。
"""

from __future__ import annotations

import importlib.util
import os
import string
import urllib.request
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Mapping


@dataclass(frozen=True)
class ComponentHealth:
    name: str
    required: bool
    status: str
    selected_provider: str = ""
    evidence: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.blockers


@dataclass(frozen=True)
class VoiceRuntimeHealthReport:
    profile: str
    status: str
    components: tuple[ComponentHealth, ...]
    contract_only: bool = False

    @property
    def ok(self) -> bool:
        return not self.blockers

    @property
    def blockers(self) -> tuple[str, ...]:
        return tuple(
            blocker
            for component in self.components
            for blocker in component.blockers
        )

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple(
            warning
            for component in self.components
            for warning in component.warnings
        )

    def as_dict(self) -> dict:
        return {
            "profile": self.profile,
            "status": self.status,
            "contract_only": self.contract_only,
            "components": [asdict(component) for component in self.components],
            "blockers": list(self.blockers),
            "warnings": list(self.warnings),
        }


class VoiceRuntimeProfileError(ValueError):
    pass


class VoiceRuntimeDeploymentChecker:
    """检查 profile 的模块、路径、环境变量和可选只读 endpoint。"""

    def __init__(
        self,
        *,
        module_finder: Callable[[str], object | None] = importlib.util.find_spec,
        path_exists: Callable[[Path], bool] = Path.exists,
        path_is_file: Callable[[Path], bool] = Path.is_file,
        path_is_dir: Callable[[Path], bool] = Path.is_dir,
        path_is_executable: Callable[[Path], bool] | None = None,
        endpoint_probe: Callable[[str, float], bool] | None = None,
    ):
        self._module_finder = module_finder
        self._path_exists = path_exists
        self._path_is_file = path_is_file
        self._path_is_dir = path_is_dir
        self._path_is_executable = path_is_executable or (
            lambda path: path.is_file() and os.access(path, os.X_OK)
        )
        self._endpoint_probe = endpoint_probe or _http_probe

    def check(
        self,
        profile_name: str,
        profile: Mapping[str, object],
        *,
        variables: Mapping[str, str],
        environment: Mapping[str, str] | None = None,
        probe_endpoints: bool = False,
        contract_only: bool = False,
    ) -> VoiceRuntimeHealthReport:
        components = profile.get("components")
        if not isinstance(components, Mapping) or not components:
            raise VoiceRuntimeProfileError(
                f"profile {profile_name!r} must declare components"
            )
        environment = environment if environment is not None else os.environ
        reports = tuple(
            self._check_component(
                str(name),
                raw,
                variables=variables,
                environment=environment,
                probe_endpoints=probe_endpoints,
                contract_only=contract_only,
            )
            for name, raw in components.items()
        )
        if any(not report.ok for report in reports):
            status = "blocked"
        elif any(
            report.status in {"degraded", "unverified"} for report in reports
        ):
            status = "degraded"
        else:
            status = "contract_valid" if contract_only else "ready"
        return VoiceRuntimeHealthReport(
            profile_name,
            status,
            reports,
            contract_only=contract_only,
        )

    def _check_component(
        self,
        name: str,
        raw: object,
        *,
        variables: Mapping[str, str],
        environment: Mapping[str, str],
        probe_endpoints: bool,
        contract_only: bool,
    ) -> ComponentHealth:
        if not isinstance(raw, Mapping):
            raise VoiceRuntimeProfileError(f"component {name!r} must be a mapping")
        required = bool(raw.get("required", True))
        candidates = raw.get("candidates")
        if not isinstance(candidates, list) or not candidates:
            raise VoiceRuntimeProfileError(
                f"component {name!r} must declare at least one candidate"
            )
        validated = [
            self._validate_candidate_contract(name, candidate)
            for candidate in candidates
        ]
        if contract_only:
            providers = tuple(candidate["provider"] for candidate in validated)
            return ComponentHealth(
                name,
                required,
                "contract",
                selected_provider=providers[0],
                evidence=tuple(f"candidate:{provider}" for provider in providers),
            )

        failures: list[tuple[str, tuple[str, ...]]] = []
        for index, candidate in enumerate(validated):
            provider = candidate["provider"]
            evidence, problems, endpoint_warnings = self._check_candidate(
                name,
                candidate,
                variables=variables,
                environment=environment,
                probe_endpoints=probe_endpoints,
            )
            if not problems:
                warnings = list(endpoint_warnings)
                # 配置了 endpoint 但未显式探测时不能宣称健康，只能说明依赖契约完整。
                status = "unverified" if endpoint_warnings else "ready"
                if index:
                    status = "degraded"
                    skipped = ",".join(
                        f"{item_provider}({';'.join(item_problems)})"
                        for item_provider, item_problems in failures
                    )
                    warnings.append(
                        f"{name}:fallback_selected:{provider}:skipped={skipped}"
                    )
                return ComponentHealth(
                    name,
                    required,
                    status,
                    selected_provider=provider,
                    evidence=tuple(evidence),
                    warnings=tuple(warnings),
                )
            failures.append((provider, tuple(problems)))

        flattened = tuple(
            f"{name}:{provider}:{problem}"
            for provider, problems in failures
            for problem in problems
        )
        if required:
            return ComponentHealth(
                name,
                required,
                "blocked",
                blockers=flattened,
            )
        return ComponentHealth(
            name,
            required,
            "disabled",
            warnings=flattened,
        )

    @staticmethod
    def _validate_candidate_contract(
        component: str, raw: object
    ) -> Mapping[str, object]:
        if not isinstance(raw, Mapping):
            raise VoiceRuntimeProfileError(
                f"component {component!r} candidate must be a mapping"
            )
        provider = raw.get("provider")
        if not isinstance(provider, str) or not provider.strip():
            raise VoiceRuntimeProfileError(
                f"component {component!r} candidate requires provider"
            )
        for key in ("python_modules", "environment", "paths", "endpoints"):
            value = raw.get(key, [])
            if not isinstance(value, list):
                raise VoiceRuntimeProfileError(
                    f"component {component!r} candidate field {key!r} must be a list"
                )
        verification = str(raw.get("verification", "local"))
        if verification not in {"local", "credential_only"}:
            raise VoiceRuntimeProfileError(
                f"component {component!r} has unknown verification "
                f"{verification!r}"
            )
        for path_check in raw.get("paths", []):
            if not isinstance(path_check, Mapping):
                raise VoiceRuntimeProfileError(
                    f"component {component!r} path check must be a mapping"
                )
            kind = str(path_check.get("kind", "any"))
            if kind not in {"any", "file", "dir", "executable"}:
                raise VoiceRuntimeProfileError(
                    f"component {component!r} has unknown path kind {kind!r}"
                )
            if not str(path_check.get("value", "")).strip():
                raise VoiceRuntimeProfileError(
                    f"component {component!r} path check requires value"
                )
        for key in ("python_modules", "environment", "endpoints"):
            if any(not str(item).strip() for item in raw.get(key, [])):
                raise VoiceRuntimeProfileError(
                    f"component {component!r} candidate field {key!r} "
                    "cannot contain empty values"
                )
        return raw

    def _check_candidate(
        self,
        component: str,
        candidate: Mapping[str, object],
        *,
        variables: Mapping[str, str],
        environment: Mapping[str, str],
        probe_endpoints: bool,
    ) -> tuple[list[str], list[str], list[str]]:
        evidence: list[str] = []
        problems: list[str] = []
        warnings: list[str] = []

        for module in candidate.get("python_modules", []):
            module_name = str(module)
            if _module_available(module_name, self._module_finder):
                evidence.append(f"python:{module_name}")
            else:
                problems.append(f"python_module_missing:{module_name}")

        for variable in candidate.get("environment", []):
            name = str(variable)
            if str(environment.get(name, "")).strip():
                evidence.append(f"env:{name}")
            else:
                problems.append(f"environment_missing:{name}")

        for raw_path in candidate.get("paths", []):
            if not isinstance(raw_path, Mapping):
                raise VoiceRuntimeProfileError(
                    f"component {component!r} path check must be a mapping"
                )
            expanded = _expand(str(raw_path.get("value", "")), variables)
            kind = str(raw_path.get("kind", "any"))
            path = Path(expanded).expanduser()
            if _path_matches(
                path,
                kind,
                exists=self._path_exists,
                is_file=self._path_is_file,
                is_dir=self._path_is_dir,
                is_executable=self._path_is_executable,
            ):
                evidence.append(f"path:{kind}:{path}")
            else:
                problems.append(f"path_{kind}_missing:{path}")

        for endpoint in candidate.get("endpoints", []):
            url = _expand(str(endpoint), variables)
            if not probe_endpoints:
                evidence.append(f"endpoint_configured:{url}")
                warnings.append(f"{component}:endpoint_not_probed:{url}")
            elif self._endpoint_probe(url, 2.0):
                evidence.append(f"endpoint_healthy:{url}")
            else:
                problems.append(f"endpoint_unhealthy:{url}")
        if candidate.get("verification", "local") == "credential_only":
            # 模块存在且 Key 非空，只能证明“具备发起请求的条件”，不能证明网络、
            # 账号权限或密钥有效；真正 ready 必须由一次受控在线验收给出。
            provider = str(candidate.get("provider", "unknown"))
            warnings.append(
                f"{component}:cloud_credentials_not_verified:{provider}"
            )
        return evidence, problems, warnings


def _expand(value: str, variables: Mapping[str, str]) -> str:
    return string.Template(value).safe_substitute(
        {key: str(item) for key, item in variables.items()}
    )


def _module_available(
    module: str, finder: Callable[[str], object | None]
) -> bool:
    try:
        return finder(module) is not None
    except (ImportError, ModuleNotFoundError, ValueError):
        return False


def _path_matches(
    path: Path,
    kind: str,
    *,
    exists: Callable[[Path], bool],
    is_file: Callable[[Path], bool],
    is_dir: Callable[[Path], bool],
    is_executable: Callable[[Path], bool],
) -> bool:
    if kind == "file":
        return is_file(path)
    if kind == "dir":
        return is_dir(path)
    if kind == "executable":
        return is_executable(path)
    if kind == "any":
        return exists(path)
    raise VoiceRuntimeProfileError(f"unknown path kind: {kind!r}")


def _http_probe(url: str, timeout_s: float) -> bool:
    try:
        request = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return 200 <= int(response.status) < 300
    except (OSError, ValueError):
        return False
