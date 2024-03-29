/******************************************************************************
 *  Copyright 2023 Labforge Inc.                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this project except in compliance with the License.        *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 ******************************************************************************

@file bottlenose_camera_driver_node.cpp Implementation of Bottlenose Camera Driver
@author Thomas Reidemeister <thomas@labforge.ca>
*/

#include "bottlenose_camera_driver.hpp"

int main(int argc, char * argv[])
{

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    rclcpp::NodeOptions options;
    auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);

    exec.add_node(bottlenose_camera_driver);


    exec.spin();
    
    rclcpp::shutdown();
    return 0;
}