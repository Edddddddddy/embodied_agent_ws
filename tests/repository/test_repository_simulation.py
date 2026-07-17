"""仿真执行层的模块所有权与安全契约护栏。"""

from repository_test_support import ROOT


def test_simulation_command_policy_is_shared_by_action_and_bt_paths():
    """BT 开关不能改变 typed command 的可执行集合。"""
    package = ROOT / "src" / "embodied_simulation"
    header = (
        package
        / "include"
        / "embodied_simulation"
        / "robot_command_policy.hpp"
    )
    source = package / "src" / "robot_command_policy.cpp"
    node = (package / "src" / "simulation_control_node.cpp").read_text(
        encoding="utf-8"
    )
    tree = (package / "src" / "command_behavior_tree.cpp").read_text(
        encoding="utf-8"
    )

    assert header.is_file()
    assert source.is_file()
    assert "src/robot_command_policy.cpp" in (
        package / "CMakeLists.txt"
    ).read_text(encoding="utf-8")
    for caller in (node, tree):
        assert "is_executable_robot_command(command)" in caller or (
            "is_executable_robot_command(goal->command)" in caller
        )
    # 防止未来在两个 Adapter 中再次复制 action_type 分支和数值边界。
    assert "static bool supported_action_goal" not in node
    assert "bool valid_command" not in tree
    assert "std::isfinite" not in node
    assert "std::isfinite" not in tree
