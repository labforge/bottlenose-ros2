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

@file bottlenose_camera_driver.hpp Definition of Bottlenose Camera Driver
@author Thomas Reidemeister <thomas@labforge.ca>
*/
#ifndef __BOTTLENOSE_PARAMETERS_HPP__
#define __BOTTLENOSE_PARAMETERS_HPP__

#include "rclcpp/rclcpp.hpp"

typedef struct {
  const std::string name;
  const rclcpp::ParameterValue & default_value;
} parameter_t;

typedef enum {
  MODE_MONO,
  MODE_STEREO,
  MODE_DEPTH,
  /* MODE_RGBD: FIXME: as soon as the FW update rolls in for this */
} camera_mode_t;

const parameter_t bottlenose_parameters[] = {
  {"frame_id", rclcpp::ParameterValue("camera")},
  {"camera_calibration_file", rclcpp::ParameterValue("package://bottlenose_camera_driver/config/camera.yaml")},
  {"mac_address", rclcpp::ParameterValue("00:00:00:00:00:00")},
  {"keep_partial", rclcpp::ParameterValue(false)},
  {"mode", rclcpp::ParameterValue(0)},
  /* Device parameters */
  {"fps", rclcpp::ParameterValue(20.0)},
  {"exposure", rclcpp::ParameterValue(20.0)},
  {"gain", rclcpp::ParameterValue(1.0)},
  {"gamma", rclcpp::ParameterValue(2.2)},
  {"dgainBlue", rclcpp::ParameterValue(1024)},
  {"dgainGB", rclcpp::ParameterValue(1024)},
  {"dgainGR", rclcpp::ParameterValue(1024)},
  {"dgainRed", rclcpp::ParameterValue(1024)},
  /* ISP parameters */
  {"blackBlue", rclcpp::ParameterValue(0xFF)},
  {"blackGB", rclcpp::ParameterValue(0xFF)},
  {"blackGR", rclcpp::ParameterValue(0xFF)},
  {"blackRed", rclcpp::ParameterValue(0xFF)},
  {"brightness", rclcpp::ParameterValue(-13107)},
  {"linearContrast", rclcpp::ParameterValue(136)},
  {"blackGainBlue", rclcpp::ParameterValue(0.9375)},
  {"blackGainGB", rclcpp::ParameterValue(0.9375)},
  {"blackGainGR", rclcpp::ParameterValue(0.9375)},
  {"blackGainRed", rclcpp::ParameterValue(0.9375)},
  {"wbBlue", rclcpp::ParameterValue(1.0)},
  {"wbGreen", rclcpp::ParameterValue(1.0)},
  {"wbRed", rclcpp::ParameterValue(1.0)},
  {"custom_ccm", rclcpp::ParameterValue(false)},
  {"CCMValue00", rclcpp::ParameterValue(1.0)},
  {"CCMValue01", rclcpp::ParameterValue(1.0)},
  {"CCMValue02", rclcpp::ParameterValue(1.0)},
  {"CCMValue10", rclcpp::ParameterValue(1.0)},
  {"CCMValue11", rclcpp::ParameterValue(1.0)},
  {"CCMValue12", rclcpp::ParameterValue(1.0)},
  {"CCMValue20", rclcpp::ParameterValue(1.0)},
  {"CCMValue21", rclcpp::ParameterValue(1.0)},
  {"CCMValue22", rclcpp::ParameterValue(1.0)},

  /* Lens and extrinsic parameters -> not used FIXME: figure out ROS2 calibration files */
  {"Rectification", rclcpp::ParameterValue(false)},
  {"Undistortion", rclcpp::ParameterValue(false)},
  {"cx0", rclcpp::ParameterValue(1928.27)},
  {"cy0", rclcpp::ParameterValue(1089.25)},
  {"fx0", rclcpp::ParameterValue(1199.58)},
  {"fy0", rclcpp::ParameterValue(1199.58)},
  {"k10", rclcpp::ParameterValue(-0.14)},
  {"k20", rclcpp::ParameterValue(0.01)},
  {"k30", rclcpp::ParameterValue(0.0)},
  {"p10", rclcpp::ParameterValue(0.0)},
  {"p20", rclcpp::ParameterValue(0.0)},
  {"rx0", rclcpp::ParameterValue(0.0)},
  {"ry0", rclcpp::ParameterValue(0.0)},
  {"rz0", rclcpp::ParameterValue(0.0)},
  {"tx0", rclcpp::ParameterValue(0.0)},
  {"ty0", rclcpp::ParameterValue(0.0)},
  {"tz0", rclcpp::ParameterValue(0.0)},
  {"cx1", rclcpp::ParameterValue(2124.38)},
  {"cy1", rclcpp::ParameterValue(1014.13)},
  {"fx1", rclcpp::ParameterValue(1208.40)},
  {"fy1", rclcpp::ParameterValue(1208.40)},
  {"k11", rclcpp::ParameterValue(-0.14)},
  {"k21", rclcpp::ParameterValue(0.01)},
  {"k31", rclcpp::ParameterValue(0.0)},
  {"p11", rclcpp::ParameterValue(0.0)},
  {"p21", rclcpp::ParameterValue(0.0)},
  {"rx1", rclcpp::ParameterValue(0.0)},
  {"ry1", rclcpp::ParameterValue(0.0)},
  {"rz1", rclcpp::ParameterValue(0.0)},
  {"tx1", rclcpp::ParameterValue(0.13)},
  {"ty1", rclcpp::ParameterValue(0.0)},
  {"tz1", rclcpp::ParameterValue(0.0)},

  /* GEV Parameters */
  {"AnswerTimeout", rclcpp::ParameterValue(100)},
  {"CommandRetryCount", rclcpp::ParameterValue(50)},
  {"MaximumPendingResends", rclcpp::ParameterValue(0)},
  {"MaximumResendRequestRetryByPacket", rclcpp::ParameterValue(0)},
  {"MaximumResendGroupSize", rclcpp::ParameterValue(0)},
  {"ResendRequestTimeout", rclcpp::ParameterValue(100)},
  {"RequestTimeout", rclcpp::ParameterValue(10000)},
  {"ResetOnIdle", rclcpp::ParameterValue(2000)},
};

#endif // __BOTTLENOSE_PARAMETERS_HPP__