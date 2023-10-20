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
#include <ament_index_cpp/get_package_share_directory.hpp>

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
      done = true;
      spin_thread.join();
      sleep(1);
      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());

    } else {
      // No sensor to test against, smoke test
//      rclcpp::init(0, nullptr);
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
    }
  }

  /**
   * Test that the driver can be initialized with a custom CCM.
   */
  TEST(CameraDriverTests, TestCCMSwitch) {
    static atomic<bool> done(false);
    const char*mac = getenv("BOTTLENOSE");
    if(mac) {
      // Start Bottlenose Camera Driver
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
      rclcpp::Parameter mac_param("mac_address", mac);
      bottlenose_camera_driver->set_parameter(mac_param);

      // Set CCM parameters
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("CCMColorProfile", "IndoorWarmLightCurtainClose"));

      // Fire up ROS driver
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(bottlenose_camera_driver);
      std::thread spin_thread([&exec, &bottlenose_camera_driver]() {
        while(!done) {
          exec.spin_once(100ms);
        }
      });
      sleep(3);
      done = true;
      spin_thread.join();
      sleep(1);
      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());
    } else {
      GTEST_SKIP() << "No sensor to test against, skipping test";
    }
  }

  /**
   * Test that the driver can be initialized with a custom CCM.
   */
  TEST(CameraDriverTests, TestAEXPEnable) {
    static atomic<bool> done(false);
    const char*mac = getenv("BOTTLENOSE");
    if(mac) {
      // Start Bottlenose Camera Driver
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
      rclcpp::Parameter mac_param("mac_address", mac);
      bottlenose_camera_driver->set_parameter(mac_param);

      // Set AEXP
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("autoExposureEnable", true));

      // Fire up ROS driver
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(bottlenose_camera_driver);
      std::thread spin_thread([&exec, &bottlenose_camera_driver]() {
        while(!done) {
          exec.spin_once(100ms);
        }
      });
      sleep(3);
      done = true;
      spin_thread.join();
      sleep(1);
      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());
    } else {
      GTEST_SKIP() << "No sensor to test against, skipping test";
    }
  }

  /**
   * Test auto white balance control.
   */
  TEST(CameraDriverTests, TestWbEnable) {
    static atomic<bool> done(false);
    const char*mac = getenv("BOTTLENOSE");
    if(mac) {
      // Start Bottlenose Camera Driver
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
      rclcpp::Parameter mac_param("mac_address", mac);
      bottlenose_camera_driver->set_parameter(mac_param);

      // Set wbAuto
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("wbAuto", true));

      // Fire up ROS driver
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(bottlenose_camera_driver);
      std::thread spin_thread([&exec, &bottlenose_camera_driver]() {
        while(!done) {
          exec.spin_once(100ms);
        }
      });
      sleep(3);
      done = true;
      spin_thread.join();
      sleep(1);
      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());
    } else {
      GTEST_SKIP() << "No sensor to test against, skipping test";
    }
  }

  TEST(CameraDrivcerTests, SetCalibration){
    static atomic<bool> done(false);
    const char*mac = getenv("BOTTLENOSE");
    if(mac) {
      // Start Bottlenose Camera Driver
      rclcpp::NodeOptions options;
      auto bottlenose_camera_driver = std::make_shared<bottlenose_camera_driver::CameraDriver>(options);
      rclcpp::Parameter mac_param("mac_address", mac);
      bottlenose_camera_driver->set_parameter(mac_param);

      // Set calibration parameters
      string prefix = ament_index_cpp::get_package_share_directory(bottlenose_camera_driver->get_name()) + "/config/";
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("camera_calibration_file", prefix+"camera.yaml"));
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("left_camera_calibration_file", prefix+"left_camera.yaml"));
      bottlenose_camera_driver->set_parameter(rclcpp::Parameter("right_camera_calibration_file", prefix+"right_camera.yaml"));
      
      // Fire up ROS driver
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(bottlenose_camera_driver);
      std::thread spin_thread([&exec]() {
        while(!done) {
          exec.spin_once(100ms);
        }
      });
      sleep(3);
      sleep(3);
      done = true;
      spin_thread.join();
      sleep(1);

      ASSERT_TRUE(bottlenose_camera_driver->is_streaming());
      ASSERT_TRUE(bottlenose_camera_driver->isCalibrated());

    } else {
      GTEST_SKIP() << "No sensor to test against, skipping test";
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);
  return RUN_ALL_TESTS();
}