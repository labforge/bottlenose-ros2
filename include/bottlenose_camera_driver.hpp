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

    /**
     * @brief Management thread for interacting with GEV stack. GEV stack, not ROS drives timing.
     * @param mac_address Mac address to connect to
     */
    void management_thread();
    void status_callback();       ///< ROS2 status callback and orchestration polled from a timer

    std::mutex m_mutex;           ///< Mutex for management thread <-> ROS interaction
    std::condition_variable m_cv; ///< Condition variable for management thread <-> ROS interaction
    std::thread m_thread;         ///< Management thread handle
    bool m_terminate;             ///< Flag to terminate management thread

    std::string m_mac_address;    ///< Mac address of camera to connect to

    std::list<PvBuffer *> m_buffers; ///< List of buffers for GEV stack

    rclcpp::TimerBase::SharedPtr timer_;
    std::thread m_aquisition_thread;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
};
} // namespace bottlenose_camera_driver
#endif //__BOTTLENOSE_CAMERA_DRIVER_HPP__

