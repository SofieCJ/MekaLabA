
cmake_minimum_required(VERSION 3.8)
project(lab1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---- legg til programmet ditt her ----
add_executable(lab1
        src/joint_simulator_node.cpp
        )
target_include_directories(lab1 PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/INCLUDE>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lab1 rclcpp std_msgs)
install(
        TARGETS lab1
        DESTINATION lib/${PROJECT_NAME}
)
ament_package()

