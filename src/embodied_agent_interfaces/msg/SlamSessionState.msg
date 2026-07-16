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

uint8 phase
bool map_saved
string map_yaml_path
string detail
