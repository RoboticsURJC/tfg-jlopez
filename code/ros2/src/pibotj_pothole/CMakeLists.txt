cmake_minimum_required(VERSION 3.5)
project(pibotj_pothole)

# Default to C++14 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ament_cmake package
find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(urdf REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Include directories
include_directories(
  ${ament_INCLUDE_DIRS}
)

# Declare a C++ executable (if needed)
# Uncomment and adapt if you have executables to compile
# add_executable(${PROJECT_NAME}_node src/mrm_description_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   urdf
#   controller_manager
#   joint_state_broadcaster
#   robot_state_publisher
# )

# Install targets
# Uncomment and adapt if you have libraries or executables to install
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# Install headers
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

# Install other files (e.g., launch files)
# install(FILES
#   launch/view_robot.launch.py
#   DESTINATION share/${PROJECT_NAME}/launch
# )

ament_package()
