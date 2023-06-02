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

@file bottlenose_camera_driver.cpp Implementation of Bottlenose Camera Driver
@author Thomas Reidemeister <thomas@labforge.ca>
*/

#include <chrono>
#include <memory>
#include <string>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <filesystem>

#include <iostream>
#include <list>

#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvSystem.h>
#include <PvBuffer.h>
#include <opencv2/opencv.hpp>

#include "bottlenose_camera_driver.hpp"
#include "bottlenose_parameters.hpp"

#define BUFFER_COUNT ( 16 )
#define BUFFER_SIZE ( 3840 * 2160 * 3 ) // 4K UHD, YUV422 + ~1 image plane to account for chunk info

using namespace std::chrono_literals;
using namespace std;
namespace bottlenose_camera_driver
{

CameraDriver::CameraDriver(const rclcpp::NodeOptions &node_options) : Node("bottlenose_camera_driver", node_options)
{
  // Allocate GEV buffers
  for ( size_t i = 0; i < BUFFER_COUNT; i++ )
  {
    // Create new buffer object
    PvBuffer *buffer = new PvBuffer;

    // Have the new buffer object allocate payload memory
    buffer->Alloc(BUFFER_SIZE );

    // Add to external list - used to eventually release the buffers
    m_buffers.push_back(buffer );
  }
  m_terminate = false;
  for(auto &parameter : bottlenose_parameters) {
    this->declare_parameter(parameter.name, parameter.default_value);
  }

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

  /* get ROS2 config parameter for camera calibration file */
  auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
  cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);

  cap.open(camera_id);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);

  last_frame_ = std::chrono::steady_clock::now();

  timer_ = this->create_wall_timer(1ms, std::bind(&CameraDriver::status_callback, this));
}

CameraDriver::~CameraDriver() {
  m_terminate = true;
  m_aquisition_thread.join();
  // Go through the buffer list
  auto iter = m_buffers.begin();
  while ( iter != m_buffers.end() )
  {
    delete *iter;
    iter++;
  }
  m_buffers.clear();
}

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::ConvertFrameToMessage(cv::Mat &frame)
{
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // Make sure output in the size the user wants even if it is not native
    if(frame.rows != image_width_ || frame.cols != image_height_){
        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
    }

    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
    ros_image.header = header_;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8";
    /* FIXME c++20 has std::endian */
    // ros_image.is_bigendian = (std::endian::native == std::endian::big);
    ros_image.is_bigendian = false;
    ros_image.step = frame.cols * frame.elemSize();
    size_t size = ros_image.step * frame.rows;
    ros_image.data.resize(size);

    if (frame.isContinuous())
    {
        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
    }
    else
    {
        // Copy by row by row
        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
        uchar *cv_data_ptr = frame.data;
        for (int i = 0; i < frame.rows; ++i)
        {
            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
            ros_data_ptr += ros_image.step;
            cv_data_ptr += frame.step;
        }
    }

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
}

void CameraDriver::status_callback()
{
    cap >> frame;

    auto now = std::chrono::steady_clock::now();

    if (!frame.empty() && (now - last_frame_) > 1ms)
    {
        last_frame_ = now;

        // Convert to a ROS2 image
        if (!is_flipped)
        {
            image_msg_ = ConvertFrameToMessage(frame);
        }
        else
        {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
            image_msg_ = ConvertFrameToMessage(frame);
        }

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
            new sensor_msgs::msg::CameraInfo(cinfo_manager_->getCameraInfo()));

        rclcpp::Time timestamp = this->get_clock()->now();

        image_msg_->header.stamp = timestamp;
        image_msg_->header.frame_id = frame_id_;

        camera_info_msg_->header.stamp = timestamp;
        camera_info_msg_->header.frame_id = frame_id_;

        camera_info_pub_.publish(image_msg_, camera_info_msg_);
    }
}

void CameraDriver::management_thread(const char*mac_address) {
  PvSystem sys = PvSystem();
  const PvDeviceInfo* pDevice;
  PvResult res = sys.FindDevice(mac_address, &pDevice);
  if(res.IsFailure()) {
    // FIXME: Log error
    return;
  }
  cerr << "ML_LOOP: Found device " << pDevice->GetDisplayID().GetAscii() << endl;
  PvDevice *device = PvDevice::CreateAndConnect( pDevice->GetConnectionID(), &res );
  if(res.IsFailure() || device == nullptr) {
    // FIXME: Log error
    return;
  }
  cerr << "ML_LOOP: Connected to device " << pDevice->GetDisplayID().GetAscii() << endl;
  PvGenInteger *intval = dynamic_cast<PvGenInteger *>( device->GetParameters()->Get("GevSCPSPacketSize"));
  assert(intval != nullptr);
  int64_t val;
  intval->GetValue(val);
  if(val < 8000) {
    cerr << "Warning: Configure your NICs MTU to be at least 8K to have reliable image transfer -> overriding to 8K" << endl;
    // FIXME: terminate
  }
  PvStream *stream = PvStream::CreateAndOpen( pDevice->GetConnectionID(), &res );
  if(res.IsFailure() || stream == nullptr) {
    // FIXME: Log error
    device->Disconnect();
    PvDevice::Free(device);
    return;
  }
  cerr << "ML_LOOP: Connected to stream " << pDevice->GetDisplayID().GetAscii() << endl;
  PvDeviceGEV* deviceGEV = static_cast<PvDeviceGEV *>( device );
  PvStreamGEV *streamGEV = static_cast<PvStreamGEV *>( stream );
  deviceGEV->NegotiatePacketSize();
  deviceGEV->SetStreamDestination( streamGEV->GetLocalIPAddress(), streamGEV->GetLocalPort() );

  // Stream tweaks, see https://supportcenter.pleora.com/s/article/Recommended-eBUS-Player-Settings-for-Wireless-Connection
  for (const char*param : {"ResetOnIdle",
                           "MaximumPendingResends",
                           "MaximumResendRequestRetryByPacket",
                           "MaximumResendGroupSize"}) {
    intval = static_cast<PvGenInteger *>( device->GetParameters()->Get(param));
    intval->SetValue(0);
  }
  intval = static_cast<PvGenInteger *>( device->GetParameters()->Get("ResendRequestTimeout"));
  intval->SetValue(200);
  intval = static_cast<PvGenInteger *>( device->GetParameters()->Get("RequestTimeout"));
  intval->SetValue(10000);

  // Queue buffers
  auto iter = m_buffers.begin();
  while (iter != m_buffers.end() )
  {
    // FIXME: check result
    stream->QueueBuffer( *iter );
    iter++;
  }

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  PvGenCommand *cmdStart = static_cast<PvGenCommand *>( device->GetParameters()->Get("AcquisitionStart") );
  PvGenCommand *cmdStop = static_cast<PvGenCommand *>( device->GetParameters()->Get("AcquisitionStop") );

  device->StreamEnable();
  cmdStart->Execute();

  while(!m_terminate) {
    PvBuffer *buffer = nullptr;
    PvResult operationResult;
    res = stream->RetrieveBuffer( &buffer, &operationResult);
    if(res.IsOK()) {
      if(operationResult.IsOK()) {
        switch ( buffer->GetPayloadType() ) {
          case PvPayloadTypeImage:
            cout << "  W: " << dec << buffer->GetImage()->GetWidth() << " H: " << buffer->GetImage()->GetHeight() << endl;
            break;

          case PvPayloadTypeChunkData:
            cout << " Chunk Data payload type" << " with " << buffer->GetChunkCount() << " chunks" << endl;
            break;

          case PvPayloadTypeRawData:
            cout << " Raw Data with " << buffer->GetRawData()->GetPayloadLength() << " bytes" << endl;
            break;

          case PvPayloadTypeMultiPart:
            cout << " Multi Part with " << buffer->GetMultiPartContainer()->GetPartCount() << " parts" << endl;
            break;

          default:
            cout << " Payload type not supported by this sample" << endl;
            break;
        }
      } else {
        cerr << "Aq op failed " << operationResult.GetCodeString().GetAscii() << endl;
      }
      stream->QueueBuffer( buffer );
    } else {
      cerr << "Buffer failed " << res.GetCodeString().GetAscii() << endl;
    }
  }
  cmdStop->Execute();
  device->StreamDisable();
  stream->AbortQueuedBuffers();
  while ( stream->GetQueuedBufferCount() > 0 )
  {
    PvBuffer *buffer = nullptr;
    PvResult operationResult;
    stream->RetrieveBuffer( &buffer, &operationResult );
  }
  stream->Close();
  PvStream::Free( stream );
  device->Disconnect();
  PvDevice::Free( device );
}

} // namespace bottlenose_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bottlenose_camera_driver::CameraDriver)
