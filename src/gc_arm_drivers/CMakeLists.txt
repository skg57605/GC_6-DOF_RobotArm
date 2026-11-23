cmake_minimum_required(VERSION 3.16)
project(gc_arm_drivers LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 환경(colcon)에서는 ament_cmake, 그 외에는 순수 CMake로 빌드 가능
find_package(ament_cmake QUIET)

add_library(${PROJECT_NAME} SHARED
  src/socketcan.cpp
  src/ak_servo_protocol.cpp
  src/ak_mit_protocol.cpp
  src/mks_protocol.cpp
  src/joint_config.cpp
  src/actuator.cpp
  src/simulated_bus.cpp
  src/arm_driver.cpp
)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(gc_motor_cli tools/gc_motor_cli.cpp)
target_link_libraries(gc_motor_cli ${PROJECT_NAME})

set(TEST_SOURCES
  test/test_ak_servo.cpp
  test/test_ak_mit.cpp
  test/test_mks.cpp
  test/test_joint.cpp
  test/test_arm_driver.cpp
)

if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS gc_motor_cli DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_protocols ${TEST_SOURCES})
    target_link_libraries(test_protocols ${PROJECT_NAME})
  endif()

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(Threads)
  ament_package()
else()
  message(STATUS "ament_cmake 없음 → 순수 CMake 빌드")
  include(CTest)
  if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    add_executable(test_protocols ${TEST_SOURCES})
    target_link_libraries(test_protocols ${PROJECT_NAME} GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(test_protocols)
  endif()
endif()
