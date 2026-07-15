cmake_minimum_required(VERSION 3.11)
project(manus_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manus_ros2_msgs REQUIRED)

add_executable(manus_data_publisher 
  src/manus_data_publisher.cpp
  src/ClientPlatformSpecific.cpp
  src/ManusDataPublisher.cpp)
target_include_directories(manus_data_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ./../ManusSDK/include)
set(MANUS_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/../ManusSDK/lib/libManusSDK_Integrated.so)
if(NOT EXISTS ${MANUS_SDK_LIBRARY})
  message(FATAL_ERROR "MANUS SDK is missing. Run ./scripts/fetch_manus_sdk.sh first.")
endif()
target_link_libraries(manus_data_publisher ${MANUS_SDK_LIBRARY} ncurses)

target_compile_features(manus_data_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS manus_data_publisher
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${MANUS_SDK_LIBRARY} DESTINATION lib/${PROJECT_NAME})

set_target_properties(manus_data_publisher PROPERTIES INSTALL_RPATH "$ORIGIN")

ament_target_dependencies(manus_data_publisher rclcpp std_msgs geometry_msgs manus_ros2_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
