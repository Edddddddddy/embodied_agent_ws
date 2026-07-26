builtin_interfaces/Time stamp

# HOLD 表示系统保持零速但尚未授权自动控制；解除急停后也必须先回到该状态。
uint8 HOLD=0
uint8 AUTONOMY=1
uint8 KEYBOARD=2
uint8 ESTOP=3

uint8 authority
bool estop_latched

# manager 每次进程启动生成新 epoch；仅靠递增序号无法区分“重启后的 0”和迟到旧消息。
uint64 manager_epoch

# 只在实际状态迁移时递增。监控端可用它丢弃迟到的旧状态。
uint64 transition_sequence

# 最近一次离开 AUTONOMY 的状态迁移序号；0 表示首次启动、尚未撤销过自动任务。
# HOLD/KEYBOARD/ESTOP 之间继续切换时必须保留它，终止确认才能绑定到同一代任务。
uint64 pending_autonomy_revocation_sequence

# 上游已确认该次撤销对应的 Nav2/SLAM 任务终止且末端速度为零。
# RESUME_AUTONOMY 会消费此确认，避免旧任务在恢复后“幽灵复活”。
bool autonomy_quiescence_acknowledged

# 当前有权产生运动意图的来源；HOLD 时为空字符串。
string active_source
string reason
