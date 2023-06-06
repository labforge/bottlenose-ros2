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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <PvBuffer.h>

namespace bottlenose_camera_driver {
  class CameraDriver : public rclcpp::Node {
  public:
    explicit CameraDriver(const rclcpp::NodeOptions&);
    ~CameraDriver();
  private:
    std::shared_ptr<sensor_msgs::msg::Image> convertFrameToMessage(PvBuffer *buffer);

    void management_thread();             ///< Management thread for interacting with GEV stack.
    void status_callback();               ///< ROS2 status callback and orchestration polled from a timer.

    std::atomic<bool> done;               ///< Flag for management thread to terminate.
    std::thread m_thread;                 ///< Management thread handle.
    bool m_terminate;                     ///< Flag to terminate management thread.

    std::string m_mac_address;            ///< Mac address of camera to connect to.
    std::shared_ptr<sensor_msgs::msg::Image> m_image_msg; ///< Image message to publish.

    std::list<PvBuffer *> m_buffers;      ///< List of buffers for GEV stack.

    rclcpp::TimerBase::SharedPtr m_timer; ///< Timer for status callback.
    std::thread m_management_thread;      ///< Management thread handle.

    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfo_manager;
    image_transport::CameraPublisher m_camera_pub; ///< Camera publisher.
};
} // namespace bottlenose_camera_driver
#endif //__BOTTLENOSE_CAMERA_DRIVER_HPP__

