builtin_interfaces/Time stamp

uint8 STOPPED=0
uint8 STARTING_MAPPING=1
uint8 MAPPING=2
uint8 SAVING_MAP=3
uint8 MAP_SAVED=4
uint8 SWITCHING_TO_NAVIGATION=5
uint8 STARTING_NAVIGATION=6
uint8 NAVIGATING=7
uint8 FAILED=8
uint8 STOPPING=9
uint8 AUTOMATIC_MAPPING=10
uint8 AUTOMATIC_NAVIGATING=11
uint8 MISSION_COMPLETED=12
# 撤销自治后保持业务互斥，直到 Nav2/Explore/STOP/零速证据完成同代 ACK。
uint8 QUIESCING=13

uint8 phase
bool map_saved
string map_yaml_path
string detail

uint8 PROFILE_UNSPECIFIED=0
uint8 PROFILE_KNOWN_WORLD=1
uint8 PROFILE_UNKNOWN_WORLD=2

# mission_sequence 区分同一进程内的多次任务；目标 sequence 只在本任务内递增。
uint8 mission_profile
uint64 mission_sequence

# phase 表示当前 ROS 运行阶段；mission_outcome 独立表达自动任务终态。
# 导航失败后可以保留 NAVIGATING 运行能力，监控端也无需解析 detail 文本。
uint8 MISSION_IDLE=0
uint8 MISSION_RUNNING=1
uint8 MISSION_SUCCEEDED=2
uint8 MISSION_FAILED=3
uint8 MISSION_CANCELED=4
uint8 mission_outcome
string mission_message

embodied_agent_interfaces/FrontierExplorationEvidence frontier
embodied_agent_interfaces/SlamMappingCompletionEvidence mapping_completion
embodied_agent_interfaces/SlamNavigationGoalEvidence[] sampled_navigation_goals
