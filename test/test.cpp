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

@file test.cpp Integration Test suite for this driver
@author Thomas Reidemeister <thomas@labforge.ca>
*/
#include <PvConfigurationReader.h>
#include "rclcpp/rclcpp.hpp"
#include "bottlenose_camera_driver.hpp"
#include <gtest/gtest.h>

namespace {
  TEST(CameraDriverTests, SmokeTest) {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
  }
}