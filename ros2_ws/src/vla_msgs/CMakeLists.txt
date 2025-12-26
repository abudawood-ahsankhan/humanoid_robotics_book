cmake_minimum_required(VERSION 3.8)
project(vla_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define message files
set(msg_files
  "msg/ActionPlan.msg"
  "msg/ObjectList.msg"
  "msg/ExecutionStatus.msg"
  "msg/SceneDescription.msg"
  "msg/ObjectQuery.msg"
  "msg/RecognitionRequest.msg"
  "msg/RecognitionResult.msg"
  "msg/GraspRequest.msg"
  "msg/GraspResult.msg"
  "msg/Transcription.msg"
  "msg/PlanRequest.msg"
  "msg/PlanResponse.msg"
)

# Define service files
set(srv_files
  "srv/ProcessVoice.srv"
  "srv/GeneratePlan.srv"
  "srv/DetectObjects.srv"
  "srv/CheckSafety.srv"
)

# Define action files
set(action_files
  "action/ExecuteAction.action"
  "action/GraspAction.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()