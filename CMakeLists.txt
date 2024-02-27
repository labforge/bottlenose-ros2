##############################################################################
#  Copyright 2023 Labforge Inc.                                              #
#                                                                            #
# Licensed under the Apache License, Version 2.0 (the "License");            #
# you may not use this project except in compliance with the License.        #
# You may obtain a copy of the License at                                    #
#                                                                            #
#     http://www.apache.org/licenses/LICENSE-2.0                             #
#                                                                            #
# Unless required by applicable law or agreed to in writing, software        #
# distributed under the License is distributed on an "AS IS" BASIS,          #
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   #
# See the License for the specific language governing permissions and        #
# limitations under the License.                                             #
##############################################################################
cmake_minimum_required(VERSION 3.9.0)
include(GoogleTest)

project(bottlenose_camera_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/cmake/eBusConfig.cmake")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(CURL REQUIRED)

# Special handling for legacy ROS messages (i.e. Foxy)
add_compile_definitions(ROS_VERSION_MAJOR=${std_msgs_VERSION_MAJOR})
add_compile_definitions(ROS_VERSION_MINOR=${std_msgs_VERSION_MINOR})


add_library(bottlenose_camera_driver SHARED
        src/bottlenose_camera_driver.cpp
        src/bottlenose_chunk_parser.cpp)
target_compile_definitions(bottlenose_camera_driver
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(bottlenose_camera_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $eBUS_INCLUDE_DIRS)
target_link_libraries(bottlenose_camera_driver ${eBUS_LIBRARIES})

ament_target_dependencies(bottlenose_camera_driver
    "rclcpp"
    "rclcpp_components"
    "sensor_msgs"
    "std_msgs"
    "vision_msgs"
    "visualization_msgs"
    "camera_info_manager"
    "image_transport"
    "camera_calibration_parsers"
    "OpenCV"
    "CURL"
)

rclcpp_components_register_nodes(bottlenose_camera_driver "bottlenose_camera_driver::CameraDriver")
set(node_plugins "${node_plugins}bottlenose_camera_driver::CameraDriver;$<TARGET_FILE:bottlenose_camera_driver>\n")

add_executable(bottlenose_camera_driver_node
        src/bottlenose_camera_driver_node.cpp
)
target_link_libraries(bottlenose_camera_driver_node
        bottlenose_camera_driver
)
add_executable(test_integration
        test/test.cpp
        )
target_link_libraries(test_integration gtest gtest_main bottlenose_camera_driver)
target_include_directories(test_integration
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#gtest_discover_tests(test_integration)

ament_target_dependencies(bottlenose_camera_driver_node
  "rclcpp"
)

install(TARGETS
  bottlenose_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bottlenose_camera_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Lint
find_program(CPPLINT cpplint)
if(CPPLINT)
    add_test(NAME cpplint_test COMMAND ${CPPLINT} --verbose=0 --recursive src include)
    set_tests_properties(cpplint_test PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  set(ament_cmake_copyright_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#
#  find_package(ament_cmake_gtest REQUIRED)
#endif()

ament_package()
