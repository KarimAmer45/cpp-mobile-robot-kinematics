cmake_minimum_required(VERSION 3.16)

project(
  cpp_mobile_robot_kinematics
  VERSION 0.1.0
  DESCRIPTION "Modern C++ mobile robot kinematics models"
  LANGUAGES CXX)

option(CMRK_BUILD_EXAMPLES "Build example programs" ON)
option(CMRK_BUILD_TESTS "Build tests" ON)

add_library(
  cpp_mobile_robot_kinematics
  src/ackermann_drive.cpp
  src/differential_drive.cpp
  src/geometry.cpp
  src/mecanum_drive.cpp)

add_library(cpp_mobile_robot_kinematics::cpp_mobile_robot_kinematics ALIAS
            cpp_mobile_robot_kinematics)

target_compile_features(cpp_mobile_robot_kinematics PUBLIC cxx_std_17)

target_include_directories(
  cpp_mobile_robot_kinematics
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

if(MSVC)
  target_compile_options(cpp_mobile_robot_kinematics PRIVATE /W4 /permissive-)
else()
  target_compile_options(cpp_mobile_robot_kinematics PRIVATE -Wall -Wextra
                                                             -Wpedantic)
endif()

if(CMRK_BUILD_EXAMPLES)
  add_executable(cmrk_differential_drive_demo
                 examples/differential_drive_demo.cpp)
  target_link_libraries(cmrk_differential_drive_demo
                        PRIVATE cpp_mobile_robot_kinematics)

  add_executable(cmrk_mecanum_drive_demo examples/mecanum_drive_demo.cpp)
  target_link_libraries(cmrk_mecanum_drive_demo
                        PRIVATE cpp_mobile_robot_kinematics)

  add_executable(cmrk_ackermann_drive_demo examples/ackermann_drive_demo.cpp)
  target_link_libraries(cmrk_ackermann_drive_demo
                        PRIVATE cpp_mobile_robot_kinematics)
endif()

include(CTest)

if(CMRK_BUILD_TESTS)
  enable_testing()

  add_executable(cmrk_kinematics_tests tests/kinematics_tests.cpp)
  target_link_libraries(cmrk_kinematics_tests
                        PRIVATE cpp_mobile_robot_kinematics)

  add_test(NAME kinematics COMMAND cmrk_kinematics_tests)
endif()

include(GNUInstallDirs)

install(
  TARGETS cpp_mobile_robot_kinematics
  EXPORT cpp_mobile_robot_kinematicsTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT cpp_mobile_robot_kinematicsTargets
  NAMESPACE cpp_mobile_robot_kinematics::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cpp_mobile_robot_kinematics)

