cmake_minimum_required(VERSION 3.8)
project(ekf_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dv_msgs REQUIRED)

add_library(ekf src/ekf.cpp src/ekf_localization.cpp src/msgs_bridge.cpp)
add_library(ekf_localization::ekf ALIAS ekf)
target_compile_features(ekf PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ekf
  "rclcpp"
  "std_msgs"
  "Eigen3"
  "dv_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ekf PRIVATE "EKF_LOCALIZATION_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ekf
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(entry src/entry.cpp)
target_include_directories(entry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(entry ekf)

install(TARGETS entry
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # include test resources
  install(
    DIRECTORY test/resource/
    DESTINATION share/${PROJECT_NAME}
  )
  # test test
  ament_add_gtest(${PROJECT_NAME}_test_test test/test_test.cpp)
  target_include_directories(${PROJECT_NAME}_test_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # ekf test
  ament_add_gtest(${PROJECT_NAME}_ekf_test test/ekf_test.cpp)
  target_include_directories(${PROJECT_NAME}_ekf_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_ekf_test
    Eigen3
  )
  target_link_libraries(${PROJECT_NAME}_ekf_test ekf)
  # ekf test
  ament_add_gtest(${PROJECT_NAME}_ekf_localization_test test/ekf_localization_test.cpp)
  target_include_directories(${PROJECT_NAME}_ekf_localization_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_ekf_localization_test
    Eigen3
  )
  target_link_libraries(${PROJECT_NAME}_ekf_localization_test ekf)
  # msgs_bridge test
  ament_add_gtest(${PROJECT_NAME}_msgs_bridge_test test/msgs_bridge_test.cpp)
  target_include_directories(${PROJECT_NAME}_msgs_bridge_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_msgs_bridge_test
    Eigen3
    rclcpp
    std_msgs
    dv_msgs
  )
  target_link_libraries(${PROJECT_NAME}_msgs_bridge_test ekf)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ekf
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
