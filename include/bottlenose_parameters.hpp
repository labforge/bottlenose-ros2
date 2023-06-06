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
  {"height", rclcpp::ParameterValue(480)},
  {"width", rclcpp::ParameterValue(640)},
  {"camera_id", rclcpp::ParameterValue(0)},
  {"mac_address", rclcpp::ParameterValue("00:00:00:00:00:00")},
  {"keep_partial", rclcpp::ParameterValue(false)},
};

#endif // __BOTTLENOSE_PARAMETERS_HPP__