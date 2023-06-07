//
// Created by treideme on 01/06/23.
//
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