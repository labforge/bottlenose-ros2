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


namespace bottlenose_camera_driver
{
  using namespace std::chrono_literals;
  using namespace std;
  using namespace cv;

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

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  m_camera_pub = image_transport::create_camera_publisher(this, "image_raw", custom_qos_profile);

  m_cinfo_manager = std::make_shared<camera_info_manager::CameraInfoManager>(this);

//  /* get ROS2 config parameter for camera calibration file */
//  auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
//  m_cinfo_manager->loadCameraInfo(camera_calibration_file_param_);

  m_timer = this->create_wall_timer(1ms, std::bind(&CameraDriver::status_callback, this));
}

CameraDriver::~CameraDriver() {
  m_terminate = true;
  m_management_thread.join();
  // Go through the buffer list
  auto iter = m_buffers.begin();
  while ( iter != m_buffers.end() )
  {
    delete *iter;
    iter++;
  }
  m_buffers.clear();
}

//std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::ConvertFrameToMessage(cv::Mat &frame)
//{
//    std_msgs::msg::Header header_;
//    sensor_msgs::msg::Image ros_image;
//
//    // Make sure output in the size the user wants even if it is not native
//    if(frame.rows != image_width_ || frame.cols != image_height_){
//        cv::resize(frame, frame, cv::Size(image_width_, image_height_));
//    }
//
//    /* To remove CV-bridge and boost-python3 dependencies, this is pretty much a copy of the toImageMsg method in cv_bridge. */
//    ros_image.header = header_;
//    ros_image.height = frame.rows;
//    ros_image.width = frame.cols;
//    ros_image.encoding = "bgr8";
//    /* FIXME c++20 has std::endian */
//    // ros_image.is_bigendian = (std::endian::native == std::endian::big);
//    ros_image.is_bigendian = false;
//    ros_image.step = frame.cols * frame.elemSize();
//    size_t size = ros_image.step * frame.rows;
//    ros_image.data.resize(size);
//
//    if (frame.isContinuous())
//    {
//        memcpy(reinterpret_cast<char *>(&ros_image.data[0]), frame.data, size);
//    }
//    else
//    {
//        // Copy by row by row
//        uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
//        uchar *cv_data_ptr = frame.data;
//        for (int i = 0; i < frame.rows; ++i)
//        {
//            memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
//            ros_data_ptr += ros_image.step;
//            cv_data_ptr += frame.step;
//        }
//    }
//
//    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
//    return msg_ptr_;
//}

void CameraDriver::status_callback() {
  if(m_terminate) {
    return;
  }
  if(m_management_thread.joinable()) {
    return;
  }
  auto mac_address = this->get_parameter("mac_address").as_string();
  if(mac_address == "00:00:00:00:00:00") {
    return;
  }
  m_mac_address = mac_address;
  m_management_thread = std::thread(&CameraDriver::management_thread, this);
}

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::convertFrameToMessage(PvBuffer *buffer) {
  if(buffer != nullptr) {
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // No image component, likely multipart
    // FIXME: Add points and multipart options
    PvImage *image = buffer->GetImage();
    if(image == nullptr)
      return nullptr;

    ros_image.header = header_;
    // FIXME: Right now only YUV422_8 encoding, convert to rgb8 to support ROS2 "legacy" tooling
    if( image->GetPixelType() == PvPixelYUV422_8) {
      Mat m(image->GetHeight(), image->GetWidth(), CV_8UC2, image->GetDataPointer());
      Mat res;
      cvtColor(m, res, COLOR_YUV2RGB_YUYV);
      ros_image.encoding = "rgb8";
      ros_image.height = image->GetHeight();
      ros_image.width = image->GetWidth();
      ros_image.is_bigendian = false;
      ros_image.step = image->GetWidth() * 3;
      size_t size = ros_image.step * image->GetHeight();
      ros_image.data.resize(size);
      memcpy(reinterpret_cast<char *>(&ros_image.data[0]), res.data, size);
    } else {
      return nullptr;
    }

    // Timestamp from epoch conversion
    uint64_t seconds = image->GetTimestamp() / 1e6; //useconds
    uint64_t nanoseconds = (image->GetTimestamp() - seconds * 1e6) * 1e3; //nanoseconds
    ros_image.header.stamp.nanosec = nanoseconds;
    ros_image.header.stamp.sec = seconds;
    ros_image.header.frame_id = this->get_parameter("frame_id").as_string();

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
  }
  return nullptr;
}



void CameraDriver::management_thread() {
  static size_t i = 0;
  PvSystem sys = PvSystem();
  const PvDeviceInfo* pDevice;
  PvResult res = sys.FindDevice(m_mac_address.c_str(), &pDevice);
  if(res.IsFailure()) {
    // FIXME: Log error
    cout << "E1 "<< endl;
    return;
  }
  PvDevice *device = PvDevice::CreateAndConnect( pDevice->GetConnectionID(), &res );
  if(res.IsFailure() || device == nullptr) {
    // FIXME: Log error
    return;
  }
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
    intval = static_cast<PvGenInteger *>( stream->GetParameters()->Get(param));
    intval->SetValue(0);
  }
//  intval = static_cast<PvGenInteger *>( stream->GetParameters()->Get("ResendRequestTimeout"));
//  intval->SetValue(200);
//  intval = static_cast<PvGenInteger *>( stream->GetParameters()->Get("RequestTimeout"));
//  intval->SetValue(10000);

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
            cout << i++ << "  W: " << dec << buffer->GetImage()->GetWidth() << " H: " << buffer->GetImage()->GetHeight() << endl;
            m_image_msg = convertFrameToMessage(buffer);

            if(m_image_msg != nullptr) {
              sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                  new sensor_msgs::msg::CameraInfo(m_cinfo_manager->getCameraInfo()));
              info_msg->header = m_image_msg->header;
              m_camera_pub.publish(m_image_msg, info_msg);
            }
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
