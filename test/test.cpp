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
  using namespace std;
  using namespace std::literals::chrono_literals;

  /**
   * Test that the driver can be initiated against a sensor
   * (set BOTTLENOSE environment variable to the MAC address of a sensor).
   * or without a sensor (smoke test).
   */
  TEST(CameraDriverTests, TestInitiate) {
    static atomic<bool> done(false);
    const char*mac = getenv("BOTTLENOSE");
    if(mac) {
      // Start Bottlenose Camera Driver
      rclcpp::init(0, nullptr);
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
      rclcpp::Parameter mac_param("mac_address", mac);
      bottlenose_camera_driver->set_parameter(mac_param);

      // Fire up ROS driver
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(bottlenose_camera_driver);
      std::thread spin_thread([&exec]() {
        while(!done) {
          exec.spin_once(100ms);
        }
      });
      sleep(3);
      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());
      done = true;
      spin_thread.join();

    } else {
      // No sensor to test against, smoke test
      rclcpp::init(0, nullptr);
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
    }
  }
}