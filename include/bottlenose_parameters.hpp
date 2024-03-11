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

const parameter_t bottlenose_parameters[] = {
  {"frame_id", rclcpp::ParameterValue("camera")},
  {"format", rclcpp::ParameterValue("1920x1080")},
  {"mac_address", rclcpp::ParameterValue("00:00:00:00:00:00")},
  {"keep_partial", rclcpp::ParameterValue(false)},
  {"stereo", rclcpp::ParameterValue(false)},
  {"feature_points", rclcpp::ParameterValue("none")},
  {"ai_model", rclcpp::ParameterValue("")},
  {"sparse_point_cloud", rclcpp::ParameterValue(false)},
  /* Device parameters */
  {"fps", rclcpp::ParameterValue(10.0)},
  {"exposure", rclcpp::ParameterValue(20.0)},
  {"gain", rclcpp::ParameterValue(1.0)},
  {"gamma", rclcpp::ParameterValue(2.2)},
  {"dgainBlue", rclcpp::ParameterValue(1024)},
  {"dgainGB", rclcpp::ParameterValue(1024)},
  {"dgainGR", rclcpp::ParameterValue(1024)},
  {"dgainRed", rclcpp::ParameterValue(1024)},
  {"OffsetX", rclcpp::ParameterValue(108)},
  {"OffsetY", rclcpp::ParameterValue(440)},
  {"OffsetX1", rclcpp::ParameterValue(108)},
  {"OffsetY1", rclcpp::ParameterValue(440)},
  /* ISP parameters */
  {"wbBlue", rclcpp::ParameterValue(1.0)},
  {"wbGreen", rclcpp::ParameterValue(1.0)},
  {"wbRed", rclcpp::ParameterValue(1.0)},
  {"wbAuto", rclcpp::ParameterValue(false)},
  /* Color Profile Presets */
  {"CCMColorProfile", rclcpp::ParameterValue("IndoorWarmLightCurtainOpen")},
  /* Custom Color Profile, disabled by default */
  {"CCMCustom", rclcpp::ParameterValue("") },
  /* Reception timeout */
  {"Timeout", rclcpp::ParameterValue(5000)},
  /* Black level controls */
  {"blackBlue", rclcpp::ParameterValue(4200)},
  {"blackGB", rclcpp::ParameterValue(4200)},
  {"blackGR", rclcpp::ParameterValue(4200)},
  {"blackRed", rclcpp::ParameterValue(4200)},

  /* Auto exposure controls */
  {"autoExposureEnable", rclcpp::ParameterValue(false)},
  {"autoExposureLuminanceTarget", rclcpp::ParameterValue(0x4000)},

  /* GEV Parameters (Pleora defaults) */
  {"AnswerTimeout", rclcpp::ParameterValue(1000)},
  {"CommandRetryCount", rclcpp::ParameterValue(3)},
  {"MaximumPendingResends", rclcpp::ParameterValue(512)},
  {"MaximumResendRequestRetryByPacket", rclcpp::ParameterValue(3)},
  {"MaximumResendGroupSize", rclcpp::ParameterValue(15)},
  {"ResendRequestTimeout", rclcpp::ParameterValue(5000)},
  {"RequestTimeout", rclcpp::ParameterValue(1000)},
  {"ResetOnIdle", rclcpp::ParameterValue(200)},
  {"RequestMissingPackets", rclcpp::ParameterValue(true)},
  {"ResendDelay", rclcpp::ParameterValue(0)}, // us
  {"GevSCPSPacketSize", rclcpp::ParameterValue(0)}, // 0 = Auto
  {"GevSCPD", rclcpp::ParameterValue(0)}, // ns


  /* Calibration file parameters */
  {"camera_calibration_file", rclcpp::ParameterValue("")},
  {"left_camera_calibration_file", rclcpp::ParameterValue("")},
  {"right_camera_calibration_file", rclcpp::ParameterValue("")},

  /* Keypoint parameters */
  {"features_max",  rclcpp::ParameterValue(65534)},
  {"features_threshold",  rclcpp::ParameterValue(3)},
  {"features_nms",  rclcpp::ParameterValue(false)},
  {"gftt_detector", rclcpp::ParameterValue("harris")},
  {"features_quality", rclcpp::ParameterValue(500)},
  {"features_min_distance", rclcpp::ParameterValue(15)},
  {"features_harrisk", rclcpp::ParameterValue(0.0)},

  /* Pointcloud parameters */
  {"AKAZELength", rclcpp::ParameterValue(486)},
  {"AKAZEWindow", rclcpp::ParameterValue(20)},
  {"HAMATXOffset", rclcpp::ParameterValue(0)},
  {"HAMATYOffset", rclcpp::ParameterValue(0)},
  {"HAMATRect1X", rclcpp::ParameterValue(1900)},
  {"HAMATRect1Y", rclcpp::ParameterValue(2)},
  {"HAMATRect2X", rclcpp::ParameterValue(1900)},
  {"HAMATRect2Y", rclcpp::ParameterValue(2)},
  {"HAMATMinThreshold", rclcpp::ParameterValue(60)},
  {"HAMATRatioThreshold",rclcpp::ParameterValue(1023)},

  /* DNN parameters */
//  {"DNNTopK", rclcpp::ParameterValue(1)}, // Not needed for Bounding boxes
  {"DNNMaxDetections", rclcpp::ParameterValue(100)},
  {"DNNNonMaxSuppression", rclcpp::ParameterValue(0.45)},
  {"DNNConfidence", rclcpp::ParameterValue(0.2)},

  /* NTP support */
  {"ntpEnable", rclcpp::ParameterValue(false)}
};

#endif // __BOTTLENOSE_PARAMETERS_HPP__