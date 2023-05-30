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
#ifndef __BOTTLENOSE_CAMERA_DRIVER_HPP__
#define __BOTTLENOSE_CAMERA_DRIVER_HPP__

#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>

namespace bottlenose_camera_driver {
  class CameraDriver : public rclcpp::Node {
  public:
    explicit CameraDriver(const rclcpp::NodeOptions&);
    ~CameraDriver() {};
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;
    
    bool is_flipped;

    std::string frame_id_;
    int image_height_;
    int image_width_;
    double fps_;
    int camera_id;

    std::chrono::steady_clock::time_point last_frame_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame);
    void ImageCallback();
};
} // namespace bottlenose_camera_driver
#endif //__BOTTLENOSE_CAMERA_DRIVER_HPP__

