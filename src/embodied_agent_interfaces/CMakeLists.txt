cmake_minimum_required(VERSION 3.16)
project(embodied_agent_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandContext.msg"
  "msg/CommandQueueEvent.msg"
  "msg/CommandExecutionEvent.msg"
  "msg/WakeEvent.msg"
  "msg/TextRewrite.msg"
  "msg/RecognitionFeedback.msg"
  "msg/CommandSlot.msg"
  "msg/NluCommand.msg"
  "msg/NluParseEvent.msg"
  "msg/RobotCommand.msg"
  "msg/RobotCommandFeedback.msg"
  "msg/RobotCommandResult.msg"
  "srv/SynthesizeSpeech.srv"
  "action/ExecuteRobotCommand.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
