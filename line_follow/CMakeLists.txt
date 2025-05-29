cmake_minimum_required(VERSION 3.5)
project(line_follow)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일 옵션 추가 (옵션)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 탐색
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 실행 파일 생성
add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(line_tracer src/line_tracer.cpp)
ament_target_dependencies(line_tracer
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

# 실행 파일 설치
install(TARGETS video_publisher line_tracer
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 설정 (옵션)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 패키지 설정 (마지막에 위치)
ament_package()
