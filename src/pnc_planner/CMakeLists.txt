cmake_minimum_required(VERSION 3.10)
project(pnc_planner)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pnc_planner_core
    src/visualizer.cpp
    src/ego_vehicle.cpp
    src/reference_line.cpp
    src/math/spline1d.cpp
    src/math/cartesian_frenet.cpp
    src/math/quintic_polynomial.cpp
    src/pnc_planner_node.cpp
    src/lattice_planner.cpp
    src/controller/pure_pursuit_controller.cpp
    src/controller/pid_controller.cpp
)

ament_target_dependencies(
    pnc_planner_core
    "rclcpp"
    "visualization_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_ros"
    "nav_msgs"
)

target_include_directories(pnc_planner_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(pnc_planner_core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_library(pnc_scenario_loader STATIC
  src/scenario/scenario_loader.cpp
)

target_include_directories(pnc_scenario_loader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(pnc_scenario_loader PUBLIC cxx_std_17)  # Require  C++17

target_link_libraries(pnc_scenario_loader PRIVATE
    yaml-cpp::yaml-cpp
)


add_executable(pnc_planner
    src/main.cpp
)

ament_target_dependencies(
    pnc_planner
    "rclcpp"
)

target_include_directories(pnc_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(pnc_planner PUBLIC c_std_99 cxx_std_17)

target_link_libraries(pnc_planner
    pnc_planner_core
)

add_executable(scenario_publisher
    src/scenario/scenario_publisher.cpp
)

ament_target_dependencies(
    scenario_publisher
    "rclcpp"
    "nav_msgs"
    "geometry_msgs"
)

target_link_libraries(scenario_publisher
    pnc_scenario_loader
)

target_compile_features(scenario_publisher PUBLIC
    cxx_std_17
)

install(TARGETS
    pnc_planner
    scenario_publisher
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS pnc_planner_core
    EXPORT export_pnc_planner_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_pnc_planner_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
    rclcpp
    visualization_msgs
    geometry_msgs
    tf2
    tf2_ros
    nav_msgs
)

install(DIRECTORY
    launch
    meshes
    rviz
    urdf
    config
    scenarios
    DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_quintic_polynomial
      test/test_quintic_polynomial.cpp
  )

  target_link_libraries(test_quintic_polynomial
      pnc_planner_core
  )

  ament_add_gtest(test_cartesian_frenet
      test/test_cartesian_frenet.cpp
  )

  target_link_libraries(test_cartesian_frenet
      pnc_planner_core
  )

  ament_add_gtest(test_reference_line
      test/test_reference_line.cpp
  )
  target_link_libraries(test_reference_line
      pnc_planner_core
  )

  ament_add_gtest(test_lattice_planner
     test/test_lattice_planner.cpp
  )
  target_link_libraries(test_lattice_planner
      pnc_planner_core
  )

  ament_add_gtest(test_scenario_loader
      test/test_scenario_loader.cpp
  )
  target_link_libraries(test_scenario_loader
      pnc_scenario_loader
  )
  target_compile_definitions(test_scenario_loader PRIVATE
      PNC_PLANNER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
