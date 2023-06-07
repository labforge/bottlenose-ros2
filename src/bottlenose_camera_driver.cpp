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

#include <list>


#include <PvDevice.h>
#include <PvStream.h>
#include <PvSystem.h>
#include <PvBuffer.h>
#include <opencv2/opencv.hpp>

#include "bottlenose_camera_driver.hpp"
#include "bottlenose_parameters.hpp"

#define BUFFER_COUNT ( 16 )
#define BUFFER_SIZE ( 3840 * 2160 * 3 ) // 4K UHD, YUV422 + ~1 image plane to account for chunk data


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

void CameraDriver::status_callback() {
  if(m_terminate) {
    return;
  }
  if(m_management_thread.joinable()) {
    if(!done) {
      RCLCPP_INFO_ONCE(get_logger(), "Bottlenose initialised");
      return;
    } else {
      m_management_thread.join();
    }
  }
  auto mac_address = this->get_parameter("mac_address").as_string();
  if(mac_address == "00:00:00:00:00:00") {
    RCLCPP_INFO_ONCE(get_logger(), "Bottlenose undefined please set mac_address");
    return;
  }
  m_mac_address = mac_address;
  done = false;
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

bool CameraDriver::set_interval() {
  PvGenFloat *interval = dynamic_cast<PvGenFloat *>( m_device->GetParameters()->Get("interval"));
  if(interval == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not configure interval");
    return false;
  }
  PvResult res = interval->SetValue(1000.0 / get_parameter("fps").as_double());
  if(res.IsFailure()) {
    RCLCPP_ERROR(get_logger(), "Could not configure interval to %f ms for %f fps",
                 1000.0 / get_parameter("fps").as_double(),
                 get_parameter("fps").as_double());
    return false;
  }
  return true;
}

bool CameraDriver::set_format() {
  PvGenInteger *heightParam = dynamic_cast<PvGenInteger *>( m_device->GetParameters()->Get("Height"));
  PvGenInteger *widthParam = dynamic_cast<PvGenInteger *>( m_device->GetParameters()->Get("Width"));
  if(heightParam == nullptr || widthParam == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not configure format");
    return false;
  }
  // Find the position of 'x' in the format specification string
  auto format = get_parameter("format").as_string();
  auto xPos = format.find('x');

  // Extract the width and height substrings
  std::string widthStr = format.substr(0, xPos);
  std::string heightStr = format.substr(xPos + 1);

  // Convert the width and height strings to integers
  int width = std::stoi(widthStr);
  int height = std::stoi(heightStr);

  // Print the width and height
  RCLCPP_DEBUG_STREAM(get_logger(), "Decoded format " << width << " x " << height);
  PvResult res = heightParam->SetValue(height);
  if(res.IsFailure()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure format to " << format);
    return false;
  }
  // Wait for parameter pass-through
  usleep(200*2000);
  // Confirm the format
  int64_t width_in;
  res = widthParam->GetValue(width_in);
  if(res.IsFailure()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure format to " << format);
    return false;
  }
  if(width_in != width) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure format to " << format << " actual format is " << width_in << " x " << height);
    return false;
  }

  return true;
}

bool CameraDriver::connect() {
  PvSystem sys = PvSystem();
  const PvDeviceInfo* pDevice;
  PvResult res = sys.FindDevice(m_mac_address.c_str(), &pDevice);
  if(res.IsFailure()) {
    RCLCPP_ERROR(get_logger(), "Failed to find device %s", m_mac_address.c_str());
    return false;
  }
  PvDevice *device = PvDevice::CreateAndConnect( pDevice->GetConnectionID(), &res );
  if(res.IsFailure() || device == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not connect to device %s", m_mac_address.c_str());
    return false;
  }
  PvGenInteger *intval = dynamic_cast<PvGenInteger *>( device->GetParameters()->Get("GevSCPSPacketSize"));
  assert(intval != nullptr);
  int64_t val;
  intval->GetValue(val);
  if(val < 8000) {
    RCLCPP_WARN(get_logger(), "Current MTU is %ld, please set to at least 8K for reliable image transfer", val);
  }
  PvStream *stream = PvStream::CreateAndOpen( pDevice->GetConnectionID(), &res );
  if(res.IsFailure() || stream == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not open device %s, cause %s", m_mac_address.c_str(), res.GetCodeString().GetAscii());
    disconnect();
    return false;
  }
  m_device = static_cast<PvDeviceGEV *>( device );
  m_stream = static_cast<PvStreamGEV *>( stream );
  if(m_device == nullptr || m_stream == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not initialize GEV stack");
    disconnect();
    return false;
  }
  res = m_device->NegotiatePacketSize();
  if(res.IsFailure()) {
    RCLCPP_ERROR(get_logger(), "Could not negotiate packet size");
    disconnect();
    return false;
  }
  res = m_device->SetStreamDestination( m_stream->GetLocalIPAddress(), m_stream->GetLocalPort() );
  if(res.IsFailure()) {
    RCLCPP_ERROR(get_logger(), "Could not set stream destination to localhost");
    disconnect();
    return false;
  }

  for (const char*param : {"AnswerTimeout",
                           "CommandRetryCount"}) {
    intval = static_cast<PvGenInteger *>( device->GetCommunicationParameters()->Get(param));
    res = intval->SetValue(get_parameter(param).as_int());
    if(res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not adjust communication parameters");
      disconnect();
      return false;
    }
  }

  // Stream tweaks, see https://supportcenter.pleora.com/s/article/Recommended-eBUS-Player-Settings-for-Wireless-Connection
  for (const char*param : {"ResetOnIdle",
                           "MaximumPendingResends",
                           "MaximumResendRequestRetryByPacket",
                           "MaximumResendGroupSize",
                           "ResendRequestTimeout",
                           "RequestTimeout"}) {
    intval = static_cast<PvGenInteger *>( stream->GetParameters()->Get("ResendRequestTimeout"));
    res = intval->SetValue(get_parameter(param).as_int());
    if (res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not adjust communication parameters");
      disconnect();
      return false;
    }
  }

  return true;
}

void CameraDriver::disconnect() {
  if(m_stream != nullptr) {
    m_stream->Close();
    PvStream::Free(m_stream);
  }
  if(m_device != nullptr) {
    m_device->Disconnect();
    PvDeviceGEV::Free(m_device);
  }
  m_device = nullptr;
  m_stream = nullptr;
}

bool CameraDriver::queue_buffers() {
  // Queue buffers
  auto iter = m_buffers.begin();
  while (iter != m_buffers.end() )
  {
    PvResult res = m_stream->QueueBuffer( *iter );
    if(res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not queue GEV buffers");
      return false;
    }
    iter++;
  }
  return true;
}

void CameraDriver::abort_buffers() {
  m_stream->AbortQueuedBuffers();
  while ( m_stream->GetQueuedBufferCount() > 0 )
  {
    PvBuffer *buffer = nullptr;
    PvResult operationResult;
    m_stream->RetrieveBuffer( &buffer, &operationResult );
  }
}


void CameraDriver::management_thread() {
  if(!connect()) {
    done = true;
    return;
  }
  if(!queue_buffers()) {
    disconnect();
    done = true;
    return;
  }
  // Apply one-time parameters
  if(!set_format()) {
    disconnect();
    done = true;
    return;
  }
  if(!set_interval()) {
    disconnect();
    done = true;
    return;
  }

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  PvGenCommand *cmdStart = static_cast<PvGenCommand *>( m_device->GetParameters()->Get("AcquisitionStart") );
  PvGenCommand *cmdStop = static_cast<PvGenCommand *>( m_device->GetParameters()->Get("AcquisitionStop") );
  m_device->StreamEnable();
  cmdStart->Execute();

  while(!m_terminate) {
    PvBuffer *buffer = nullptr;
    PvResult operationResult;
    PvResult res = m_stream->RetrieveBuffer( &buffer, &operationResult);
    if(res.IsOK()) {
      if(operationResult.IsOK() || (get_parameter("keep_partial").as_bool()
        && ((operationResult.GetCode() == PvResult::Code::TOO_MANY_RESENDS) ||
           (operationResult.GetCode() == PvResult::Code::RESENDS_FAILURE) ||
           (operationResult.GetCode() == PvResult::Code::TOO_MANY_CONSECUTIVE_RESENDS) ||
          (operationResult.GetCode() == PvResult::Code::MISSING_PACKETS) ||
          (operationResult.GetCode() == PvResult::Code::CORRUPTED_DATA)))) {
        switch ( buffer->GetPayloadType() ) {
          case PvPayloadTypeImage:
            m_image_msg = convertFrameToMessage(buffer);

            if(m_image_msg != nullptr) {
              RCLCPP_DEBUG(get_logger(), "Received Image %i x %i", buffer->GetImage()->GetWidth(), buffer->GetImage()->GetHeight());
              sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                  new sensor_msgs::msg::CameraInfo(m_cinfo_manager->getCameraInfo()));
              info_msg->header = m_image_msg->header;
              m_camera_pub.publish(m_image_msg, info_msg);
            }
            break;

          case PvPayloadTypeChunkData:
            RCLCPP_DEBUG_STREAM(get_logger(), "Chunk Data payload type" << " with " << buffer->GetChunkCount() << " chunks");
            break;

          case PvPayloadTypeRawData:
            RCLCPP_DEBUG_STREAM(get_logger(), "Raw Data with " << buffer->GetRawData()->GetPayloadLength() << " bytes");
            break;

          case PvPayloadTypeMultiPart:
            RCLCPP_DEBUG_STREAM(get_logger(), "Multi Part with " << buffer->GetMultiPartContainer()->GetPartCount() << " parts");
            break;

          default:
            RCLCPP_DEBUG(get_logger(), "Payload type not supported : 0x%08X", buffer->GetPayloadType());
            break;
        }
      } else {
        if(operationResult.GetCode() == PvResult::Code::TIMEOUT) {
          // This usually signifies that the GEV stack died, fall back to complete recovery
          RCLCPP_ERROR_STREAM(get_logger(),
                              "Acquisition operation failed with " << operationResult.GetCodeString().GetAscii()
                                                                   << " reconnecting");
          break;
        } else {
          // Notify retry
          RCLCPP_WARN_STREAM(get_logger(),
                             "Acquisition operation failed with " << operationResult.GetCodeString().GetAscii());
        }
      }
      res = m_stream->QueueBuffer( buffer );
      if(res.IsFailure()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not queue GEV buffers");
        break;
      }
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Buffer failed with " << res.GetCodeString().GetAscii());
      break;
    }
  }
  cmdStop->Execute();
  m_device->StreamDisable();
  abort_buffers();
  disconnect();
  done = true;
}

} // namespace bottlenose_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bottlenose_camera_driver::CameraDriver)
