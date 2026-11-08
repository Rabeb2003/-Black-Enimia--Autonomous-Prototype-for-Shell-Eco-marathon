cmake_minimum_required(VERSION 3.8)
project(lane_assist_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/lane_assist_node.py
  scripts/lane_assist_wrapper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  src/
  DESTINATION share/${PROJECT_NAME}/src
)

install(FILES
  config.ini
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
