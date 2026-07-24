"""把 SLAM session 状态压缩成操作者可读的验收进度。"""

from embodied_agent_interfaces.msg import SlamSessionState
from tools.acceptance.progress import AcceptanceProgress


def publish_state_progress(
    progress: AcceptanceProgress,
    message: SlamSessionState,
) -> None:
    """将细粒度状态机映射为稳定的六阶段里程碑。"""

    milestone = {
        SlamSessionState.STARTING_MAPPING: (1, "runtime_startup"),
        SlamSessionState.MAPPING: (1, "runtime_startup"),
        SlamSessionState.AUTOMATIC_MAPPING: (2, "frontier_slam"),
        SlamSessionState.SAVING_MAP: (3, "map_save"),
        SlamSessionState.MAP_SAVED: (3, "map_save"),
        SlamSessionState.SWITCHING_TO_NAVIGATION: (
            4,
            "localization_and_semantic_nav",
        ),
        SlamSessionState.STARTING_NAVIGATION: (
            4,
            "localization_and_semantic_nav",
        ),
        SlamSessionState.NAVIGATING: (4, "localization_and_semantic_nav"),
        SlamSessionState.AUTOMATIC_NAVIGATING: (
            4,
            "localization_and_semantic_nav",
        ),
    }.get(int(message.phase))
    if milestone is not None:
        # AcceptanceProgress 自带去重，transient-local 重投递不会刷屏。
        progress.stage(*milestone, detail=str(message.detail))
