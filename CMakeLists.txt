cmake_minimum_required(VERSION 3.8)
project(voice_alert_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# 실행 파일
add_executable(voice_alert_node
  src/main.cpp
  src/voice_alert_node.cpp
)

ament_target_dependencies(voice_alert_node
  rclcpp
  std_msgs
)

# 설치
install(TARGETS voice_alert_node
  DESTINATION lib/${PROJECT_NAME}
)

# 음성 파일 설치
install(DIRECTORY audio/
  DESTINATION share/${PROJECT_NAME}/audio
)

ament_package()
