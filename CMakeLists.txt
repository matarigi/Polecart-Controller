cmake_minimum_required(VERSION 3.8)
project(polecart_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tell CMake to add this directory to the target including path
include_directories(include/${PROJECT_NAME})

set(HEADER_FILES include/${PROJECT_NAME}/PID_Functions.hpp)
add_library(pid_function_library src/PID_Functions.cpp ${HEADER_FILES})
add_library(pid_controller_library src/PID_Controller.cpp include/${PROJECT_NAME}/PID_Controller.hpp)

target_link_libraries(pid_controller_library pid_function_library)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(polecart_controller src/Polecart_Controller.cpp)
ament_target_dependencies(polecart_controller rclcpp sensor_msgs)

target_link_libraries(polecart_controller pid_controller_library)

install(TARGETS
  polecart_controller
  DESTINATION lib/${PROJECT_NAME})

# Tests part
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  set(TESTFILES
    test/main.cpp
    test/PID_Functions_Test.cpp
    test/PID_Controller_Test.cpp)

    ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
    target_link_libraries(${PROJECT_NAME}_test pid_function_library)
    target_link_libraries(${PROJECT_NAME}_test pid_controller_library)

    install(TARGETS
      ${PROJECT_NAME}_test
      DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
