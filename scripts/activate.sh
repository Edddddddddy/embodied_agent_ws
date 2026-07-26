#!/usr/bin/env bash

_embodied_activate_main() {
  local activate_source="${BASH_SOURCE[0]}"
  local had_errexit=false
  local had_nounset=false
  local had_allexport=false
  [[ $- == *e* ]] && had_errexit=true
  [[ $- == *u* ]] && had_nounset=true
  [[ $- == *a* ]] && had_allexport=true

  # ROS/colcon 生成的 setup 可能读取未定义变量。激活期间临时关闭这些选项，
  # 结束时精确恢复，避免 `source activate.sh` 改变开发者终端的控制流语义。
  set +e
  set +u
  set +a

  local status=0
  local script_dir=""
  script_dir="$(cd -- "$(dirname -- "$activate_source")" 2>/dev/null && pwd -P)" || status=2
  if [[ $status -ne 0 ]]; then
    echo "ERROR: 无法定位 activate.sh：$activate_source" >&2
  fi

  if [[ $status -eq 0 ]]; then
    if [[ ! -f "$script_dir/lifecycle_utils.sh" ]]; then
      echo "ERROR: 缺少共享 shell 工具：$script_dir/lifecycle_utils.sh" >&2
      status=2
    elif ! source "$script_dir/lifecycle_utils.sh"; then
      echo "ERROR: 无法加载共享 shell 工具：$script_dir/lifecycle_utils.sh" >&2
      status=2
    elif ! embodied_resolve_workspace "${BASH_SOURCE[0]}"; then
      status=2
    elif ! embodied_resolve_runtime_root; then
      status=2
    fi
  fi

  # 验收隔离层会保留当前 worktree 的 prefix、删除其它 worktree。若同一子进程
  # 再次 source colcon setup，旧 underlay 会因“新加入”反而排到当前 prefix 前面。
  # 只有 marker 与 COLCON_PREFIX_PATH 同时证明当前 worktree 已激活时才短路；
  # 切换 worktree 或环境不完整时仍执行下面的完整激活流程。
  if [[ $status -eq 0 && \
        "${EMBODIED_ACTIVE_WORKSPACE:-}" == "$WORKSPACE" ]]; then
    case ":${COLCON_PREFIX_PATH:-}:" in
      *":$WORKSPACE/install:"*)
        if [[ "$had_allexport" == true ]]; then set -a; else set +a; fi
        if [[ "$had_nounset" == true ]]; then set -u; else set +u; fi
        if [[ "$had_errexit" == true ]]; then set -e; else set +e; fi
        return 0
        ;;
    esac
  fi

  local ros_setup="${EMBODIED_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
  if [[ $status -eq 0 && ! -f "$ros_setup" ]]; then
    cat >&2 <<EOF
ERROR: 未找到 ROS 2 环境：$ros_setup
请先安装 ROS 2 Jazzy，再执行：
  cd "$WORKSPACE"
  bash scripts/bootstrap.sh
EOF
    status=2
  fi

  if [[ $status -eq 0 && ! -f "$WORKSPACE/install/setup.bash" ]]; then
    cat >&2 <<EOF
ERROR: 未找到工作区安装层：$WORKSPACE/install/setup.bash
请先构建当前工作区：
  cd "$WORKSPACE"
  source "$ros_setup"
  colcon build --symlink-install
EOF
    status=2
  fi

  if [[ $status -eq 0 && -f "$WORKSPACE/scripts/ros_dds_env.sh" ]]; then
    source "$WORKSPACE/scripts/ros_dds_env.sh" || status=2
  fi
  if [[ $status -eq 0 ]]; then
    source "$ros_setup" || {
      echo "ERROR: ROS 2 环境加载失败：$ros_setup" >&2
      status=2
    }
  fi
  local python_runtime_root="$WORKSPACE"
  if [[ ! -f "$python_runtime_root/.venv/bin/activate" &&
        -f "$EMBODIED_RUNTIME_ROOT/.venv/bin/activate" ]]; then
    python_runtime_root="$EMBODIED_RUNTIME_ROOT"
  fi
  if [[ $status -eq 0 && -f "$python_runtime_root/.venv/bin/activate" ]]; then
    # linked worktree 保留自己的源码/install，但复用主 worktree 的未跟踪 venv；
    # 否则真实 Sherpa/LLM provider 只会在 main 可运行，功能分支无法独立验收。
    source "$python_runtime_root/.venv/bin/activate" || {
      echo "ERROR: Python 虚拟环境加载失败：$python_runtime_root/.venv" >&2
      status=2
    }
  fi
  if [[ $status -eq 0 && -n "${VIRTUAL_ENV:-}" ]]; then
    # ROS console script 固定使用系统 Python shebang，因此把当前 venv 依赖显式
    # 暴露给 ROS 子进程；这不是切换解释器，而是补齐 provider Python 包路径。
    local venv_site_packages=""
    venv_site_packages="$(python -c 'import site; print(site.getsitepackages()[0])')" || status=2
    if [[ $status -eq 0 ]]; then
      export PYTHONPATH="$venv_site_packages${PYTHONPATH:+:$PYTHONPATH}"
    else
      echo "ERROR: 无法解析 Python 虚拟环境 site-packages。" >&2
    fi
  fi
  if [[ $status -eq 0 ]]; then
    source "$WORKSPACE/install/setup.bash" || {
      echo "ERROR: 工作区安装层加载失败；请重新执行 colcon build --symlink-install。" >&2
      status=2
    }
  fi

  if [[ $status -eq 0 && -f "$WORKSPACE/.env" ]]; then
    # .env 中的 provider key/模型路径必须传给 launch 子进程；仅在 source 期间
    # 开启 allexport，随后恢复调用者原状态。
    set -a
    source "$WORKSPACE/.env" || {
      echo "ERROR: 项目环境文件加载失败：$WORKSPACE/.env" >&2
      status=2
    }
    set +a
  fi

  if [[ $status -eq 0 ]]; then
    export EMBODIED_ACTIVE_WORKSPACE="$WORKSPACE"
  fi
  if [[ "$had_allexport" == true ]]; then set -a; else set +a; fi
  if [[ "$had_nounset" == true ]]; then set -u; else set +u; fi
  # errexit 最后恢复，避免清理过程中的非关键命令提前终止调用者 shell。
  if [[ "$had_errexit" == true ]]; then set -e; else set +e; fi
  return "$status"
}

if _embodied_activate_main; then
  unset -f _embodied_activate_main
else
  unset -f _embodied_activate_main
  return 2 2>/dev/null || exit 2
fi
