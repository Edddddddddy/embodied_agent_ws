std_msgs/Header header
string command_id
string source
# priority=true 仅用于用户急停、会话退出和取消导航；组合动作末尾的计划 STOP 保持 false。
bool priority

uint8 UNKNOWN=0
uint8 STOP=1
uint8 MOVE=2
uint8 TURN=3
uint8 WAVE=4
uint8 SET_LED=5
uint8 SET_MODE=6
uint8 NAVIGATE_TO=7
uint8 FOLLOW_WAYPOINTS=8
uint8 CANCEL_NAVIGATION=9
uint8 ARC=10
uint8 action_type

float64 linear_x
float64 angular_z
float64 duration_s
int32 count
string color
string mode
string target
string[] waypoints
# 路线总遍历轮数：1 表示每个 waypoint 只访问一遍；执行器会转换成后端的循环语义。
uint32 number_of_loops
