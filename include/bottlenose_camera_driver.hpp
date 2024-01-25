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
        G. M. Tchamgoue <martin@labforge.ca> 
*/
#ifndef __BOTTLENOSE_CAMERA_DRIVER_HPP__
#define __BOTTLENOSE_CAMERA_DRIVER_HPP__

#include <cstdio>
#include <iostream>
#include <variant>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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
#include <PvDeviceGEV.h>
#include <PvStreamGEV.h>

#include "bottlenose_chunk_parser.hpp"

namespace bottlenose_camera_driver {
  class CameraDriver : public rclcpp::Node {
  public:
    explicit CameraDriver(const rclcpp::NodeOptions&);
    bool is_streaming();
    bool isCalibrated();
    ~CameraDriver() override;
  private:
    /**
     * Convert the frame to a ROS2 message.
     * @param buffer Buffer received on GEV interface
     * @return ROS2 message
     */
    std::shared_ptr<sensor_msgs::msg::Image> convertFrameToMessage(IPvImage *image, uint64_t timestamp);

    bool set_interval();                  ///< Set camera frame rate.
    bool set_format();                    ///< Set camera format.
    bool set_ccm_profile();               ///< Apply color profile
    bool set_ccm_custom();                ///< Apply custom color profile
    bool set_stereo();                    ///< Apply stereo setting if set
    bool set_auto_exposure();             ///< Set auto exposure
    bool update_runtime_parameters();     ///< Update runtime parameters from ROS2 parameters.
    bool connect();                       ///< Connect to camera.
    void disconnect();                    ///< Disconnect from camera.
    bool queue_buffers();                 ///< Queue buffers for GEV stack.
    void abort_buffers();                 ///< Abort buffers for GEV stack.
    void publish_features(std::vector<keypoints_t *> &features, uint64_t timestamp); ///< Publish keypoints
    void management_thread();             ///< Management thread for interacting with GEV stack.
    void status_callback();               ///< ROS2 status callback and orchestration polled from a timer.
    static bool is_ebus_loaded();         ///< Check if the eBusSDK Driver is loaded.
    bool set_chunk(std::string chunk, bool enable);    ///< Enable, Disable chunk data
    bool enable_ntp(bool enable);         ///< Enable NTP
    bool configure_feature_points();      ///< Configure feature points

    bool load_calibration(uint32_t sid, std::string cname); ///< load calibration data
    bool set_calibration();                 ///< set calibration on to camera
    uint32_t get_num_sensors();             ///< returns the number of sensors: 1=mono and 2=stereo    
    bool set_register(std::string, std::variant<int64_t, double, bool>); ///< set a register value on the camera
    bool set_enum_register(std::string, std::string); ///< set a register value on the camera
    bool m_calibrated;

    std::atomic<bool> done;               ///< Flag for management thread to terminate.
    std::thread m_thread;                 ///< Management thread handle.
    std::atomic<bool> m_terminate;        ///< Flag to terminate management thread.
    PvDeviceGEV *m_device;                ///< GEV device handle.
    PvStreamGEV *m_stream;                ///< GEV stream handle.

    std::string m_mac_address;            ///< Mac address of camera to connect to.
    /// Image message to publish for mono or left sensor
    std::shared_ptr<sensor_msgs::msg::Image> m_image_msg;
    /// Image message to publish for stereo right sensor
    std::shared_ptr<sensor_msgs::msg::Image> m_image_msg_1;

    /// Camera parameter cache
    std::map<std::string, std::variant<int64_t, double, std::string>> m_camera_parameter_cache;
    std::list<PvBuffer *> m_buffers;      ///< List of buffers for GEV stack.

    rclcpp::TimerBase::SharedPtr m_timer; ///< Timer for status callback.
    std::thread m_management_thread;      ///< Management thread handle.

    std::shared_ptr<camera_info_manager::CameraInfoManager> m_cinfo_manager[2];
    
    /// Camera publisher.
    image_transport::CameraPublisher m_image_color;
    image_transport::CameraPublisher m_image_color_1;

    // Keypoints publisher.

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_keypoints;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_keypoints_1;
};
} // namespace bottlenose_camera_driver
#endif //__BOTTLENOSE_CAMERA_DRIVER_HPP__

