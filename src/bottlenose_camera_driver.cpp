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
        G. M. Tchamgoue <martin@labforge.ca>  
*/

#include <unistd.h>
#include <chrono>
#include <memory>
#include <string>
#include <cassert>
#include <list>
#include <filesystem>

#include <PvDevice.h>
#include <PvStream.h>
#include <PvSystem.h>
#include <PvBuffer.h>
#include <opencv2/opencv.hpp>

#include "bottlenose_camera_driver.hpp"
#include "bottlenose_parameters.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#define BUFFER_COUNT ( 16 )
#define BUFFER_SIZE ( 3840 * 2160 * 3 ) // 4K UHD, YUV422 + ~1 image plane to account for chunk data
#define WAIT_PROPAGATE() usleep(250 * 1000)
#define LEFTCAM (0)
#define RIGHTCAM (1)

namespace bottlenose_camera_driver
{
  using namespace std::chrono_literals;
  using namespace std;
  using namespace cv;
  namespace fs = std::filesystem;

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
  m_image_color = image_transport::create_camera_publisher(this, "image_color", custom_qos_profile);
  m_image_color_1 = image_transport::create_camera_publisher(this, "image_color_1", custom_qos_profile);

  //m_cinfo_manager[0] = std::make_shared<camera_info_manager::CameraInfoManager>(this);
  //load_calibration(0, "camera");
  //m_cinfo_manager[1] = std::make_shared<camera_info_manager::CameraInfoManager>(this);

  if(!is_ebus_loaded()) {
    RCLCPP_ERROR(get_logger(), "The eBus Driver is not loaded, please reinstall the driver!");
  }

  m_timer = this->create_wall_timer(1ms, std::bind(&CameraDriver::status_callback, this));
}

CameraDriver::~CameraDriver() {
  m_terminate = true;
  if(m_management_thread.joinable()) {
    m_management_thread.join();
  }
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

std::shared_ptr<sensor_msgs::msg::Image> CameraDriver::convertFrameToMessage(IPvImage *image, uint64_t timestamp) {
  if(image != nullptr) {
    std_msgs::msg::Header header_;
    sensor_msgs::msg::Image ros_image;

    // No image component, likely multipart
    ros_image.header = header_;

    // Right now only YUV422_8 encoding, convert to rgb8 to support ROS2 "legacy" tooling
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

    // Timestamp from epoch conversion (Test this)
    uint64_t seconds = timestamp / 1e6; //useconds
    uint64_t nanoseconds = (timestamp - seconds * 1e6) * 1e3; //nanoseconds
    ros_image.header.stamp.nanosec = nanoseconds;
    ros_image.header.stamp.sec = seconds;
    ros_image.header.frame_id = this->get_parameter("frame_id").as_string();

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
  }
  return nullptr;
}

bool CameraDriver::update_runtime_parameters() {
    // All integer parameters
    for(auto param : {"dgainBlue",
                      "dgainGB",
                      "dgainGR",
                      "dgainRed",
                      "blackBlue",
                      "blackGB",
                      "blackGR",
                      "blackRed"}) {
        PvGenInteger *intval = static_cast<PvGenInteger *>( m_device->GetParameters()->Get(param));
        int64_t val = get_parameter(param).as_int();
        try {
            auto value = m_camera_parameter_cache.at(param);
            if(get<int64_t>(value) == val) {
                continue;
            }
        } catch(std::out_of_range &e) { }

        PvResult res = intval->SetValue(val);
        if (res.IsFailure()) {
            RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val << " cause " << res.GetDescription().GetAscii());
            return false;
        }
        // Cache
        m_camera_parameter_cache[param] = val;
        RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter " << param << " to " << val);
    }

    for(auto param : {"exposure",
            "gain",
            "gamma",
            "wbBlue",
            "wbGreen",
            "wbRed"}) {
        PvGenFloat *floatVal = static_cast<PvGenFloat *>( m_device->GetParameters()->Get(param));
        double val = get_parameter(param).as_double();
        try {
            auto value = m_camera_parameter_cache.at(param);
            if(get<double>(value) == val) {
                continue;
            }
        } catch(std::out_of_range &e) { }

        PvResult res = floatVal->SetValue(val);
        if (res.IsFailure()) {
            RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val);
            return false;
        }
        // Cache
        m_camera_parameter_cache[param] = val;
        RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter " << param << " to " << val);
    }
    return set_ccm_profile();
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
  RCLCPP_DEBUG_STREAM(get_logger(), "Configured interval to " << 1000.0 / get_parameter("fps").as_double()
    << " ms for " << get_parameter("fps").as_double() << " fps");
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
  WAIT_PROPAGATE();
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
  RCLCPP_DEBUG_STREAM(get_logger(), "Configured format to " << format);

  return true;
}

bool CameraDriver::set_ccm_profile() {
    auto ccm_profile_str = get_parameter("CCMColorProfile").as_string();
    try {
        auto value = m_camera_parameter_cache.at("CCMColorProfile");
        if(get<string>(value) == ccm_profile_str) {
            return true;
        }
    } catch(std::out_of_range &e) { }

    PvGenEnum *colorProfile = dynamic_cast<PvGenEnum *>( m_device->GetParameters()->Get("CCMColorProfile"));
    int64_t num_entries = 0;
    PvResult res = colorProfile->GetEntriesCount(num_entries);
    if(res.IsFailure()) {
        RCLCPP_ERROR(get_logger(), "Could not enumerate sensor color profiles");
        return false;
    }
    for(int64_t i = 0; i < num_entries; i++) {
        const PvGenEnumEntry *entry = nullptr;
        res = colorProfile->GetEntryByIndex(i, &entry);
        if(res.IsFailure()) {
            RCLCPP_ERROR(get_logger(), "Could not enumerate sensor color profiles");
            return false;
        }
        PvString str;
        res = entry->GetName(str);
        if(res.IsFailure()) {
            RCLCPP_ERROR(get_logger(), "Could not enumerate sensor color profiles");
            return false;
        }
        if(ccm_profile_str == str.GetAscii()) {
            res = colorProfile->SetValue(str);
            if(res.IsFailure()) {
                RCLCPP_ERROR(get_logger(), "Could not set sensor color profile");
                return false;
            }
            RCLCPP_DEBUG_STREAM(get_logger(), "Set sensor color profile to " << str.GetAscii());
            m_camera_parameter_cache["CCMColorProfile"] = ccm_profile_str;
            return true;
        }
    }
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid Color Profile: " << ccm_profile_str);
    return false;
}

bool CameraDriver::set_stereo() {
    bool enable_stereo = get_parameter("stereo").as_bool();
    PvGenBoolean *multipart = dynamic_cast<PvGenBoolean *>( m_device->GetParameters()->Get("GevSCCFGMultiPartEnabled"));
    if(multipart == nullptr) {
        RCLCPP_ERROR(get_logger(), "Could not configure stereo");
        return false;
    }

    PvResult res = multipart->SetValue(enable_stereo);
    if(!res.IsOK())
        RCLCPP_ERROR_STREAM(get_logger(), "Could not configure stereo, cause: " << res.GetDescription().GetAscii());
    else
        RCLCPP_DEBUG_STREAM(get_logger(), "Configured stereo to " << enable_stereo);

    return res.IsOK();
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
    RCLCPP_ERROR(get_logger(), "Could not connect to device %s\nCause: %s\nDescription: %s",
                 m_mac_address.c_str(), res.GetCodeString().GetAscii(), res.GetDescription().GetAscii());
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
    RCLCPP_DEBUG_STREAM(get_logger(), "Set " << param << " to " << get_parameter(param).as_int());
  }

  // Stream tweaks, see https://supportcenter.pleora.com/s/article/Recommended-eBUS-Player-Settings-for-Wireless-Connection
  for (const char*param : {"ResetOnIdle",
                           "MaximumPendingResends",
                           "MaximumResendRequestRetryByPacket",
                           "MaximumResendGroupSize",
                           "ResendRequestTimeout",
                           "RequestTimeout"}) {
    intval = static_cast<PvGenInteger *>( stream->GetParameters()->Get(param));
    res = intval->SetValue(get_parameter(param).as_int());
    if (res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not adjust communication parameters");
      disconnect();
      return false;
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "Set " << param << " to " << get_parameter(param).as_int());
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
  if(!set_stereo()) {
    disconnect();
    done = true;
    return;
  }
  // Fail hard on first runtime parameter update issues
  if(!update_runtime_parameters()) {
    disconnect();
    done = true;
    return;
  }

  if(!set_calibration()){
    RCLCPP_WARN_STREAM(get_logger(), "Camera not calibrated!");
  }else{
    RCLCPP_INFO(get_logger(), "Camera calibrated!");
  }

  int64_t timeout;
  try {
      timeout = get_parameter("Timeout").as_int();
  } catch(exception & e) {
    RCLCPP_ERROR(get_logger(), "Timeout parameter is not an integer");
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
    PvResult res = m_stream->RetrieveBuffer(&buffer, &operationResult, timeout);
    IPvImage *img0, *img1;
    if(res.IsOK()) {
      if(operationResult.IsOK() || (get_parameter("keep_partial").as_bool()
        && ((operationResult.GetCode() == PvResult::Code::TOO_MANY_RESENDS) ||
           (operationResult.GetCode() == PvResult::Code::RESENDS_FAILURE) ||
           (operationResult.GetCode() == PvResult::Code::TOO_MANY_CONSECUTIVE_RESENDS) ||
          (operationResult.GetCode() == PvResult::Code::MISSING_PACKETS) ||
          (operationResult.GetCode() == PvResult::Code::CORRUPTED_DATA)))) {
        switch ( buffer->GetPayloadType() ) {
          case PvPayloadTypeImage:
            img0 = buffer->GetImage();
            m_image_msg = convertFrameToMessage(img0, buffer->GetTimestamp());

            if(m_image_msg != nullptr) {
              RCLCPP_DEBUG(get_logger(), "Received Image %i x %i", buffer->GetImage()->GetWidth(), buffer->GetImage()->GetHeight());
              sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                  new sensor_msgs::msg::CameraInfo(m_cinfo_manager[0]->getCameraInfo()));
              info_msg->header = m_image_msg->header;
              m_image_color.publish(m_image_msg, info_msg);
            }
            break;

          case PvPayloadTypeChunkData:
            RCLCPP_DEBUG_STREAM(get_logger(), "Chunk Data payload type" << " with " << buffer->GetChunkCount() << " chunks");
            break;

          case PvPayloadTypeRawData:
            RCLCPP_DEBUG_STREAM(get_logger(), "Raw Data with " << buffer->GetRawData()->GetPayloadLength() << " bytes");
            break;

          case PvPayloadTypeMultiPart:
            img0 = buffer->GetMultiPartContainer()->GetPart(0)->GetImage();
            img1 = buffer->GetMultiPartContainer()->GetPart(1)->GetImage();
            m_image_msg = convertFrameToMessage(img0, buffer->GetTimestamp());
            m_image_msg_1 = convertFrameToMessage(img1, buffer->GetTimestamp());

            if(m_image_msg != nullptr) {
                RCLCPP_DEBUG(get_logger(), "Received left Image %i x %i", buffer->GetImage()->GetWidth(), buffer->GetImage()->GetHeight());
                sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                        new sensor_msgs::msg::CameraInfo(m_cinfo_manager[0]->getCameraInfo()));
                info_msg->header = m_image_msg->header;
                m_image_color.publish(m_image_msg, info_msg);
            }
            if(m_image_msg_1 != nullptr) {
                RCLCPP_DEBUG(get_logger(), "Received Right Image %i x %i", buffer->GetImage()->GetWidth(), buffer->GetImage()->GetHeight());
                sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                        new sensor_msgs::msg::CameraInfo(m_cinfo_manager[1]->getCameraInfo()));
                info_msg->header = m_image_msg_1->header;
                m_image_color_1.publish(m_image_msg_1, info_msg);
            }
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
        RCLCPP_ERROR(get_logger(), "Could not queue GEV buffers");
        break;
      }
    } else {
      RCLCPP_WARN_STREAM(get_logger(), "Buffer failed with " << res.GetCodeString().GetAscii());
      break;
    }
    // Even if parameter update fails, keep going to keep system alive after streaming is enabled
    if(!update_runtime_parameters()) {
        RCLCPP_WARN(get_logger(), "Runtime parameter update issue");
    }
  }
  cmdStop->Execute();
  m_device->StreamDisable();
  abort_buffers();
  disconnect();
  done = true;
}

bool CameraDriver::is_streaming() {
  return !done && m_device != nullptr && m_stream != nullptr && m_stream->IsOpen();
}

bool CameraDriver::is_ebus_loaded() {
  char buffer[128];
  std::string result = "";
  FILE* pipe = popen("/usr/sbin/lsmod", "r");
  if (!pipe)
    return false;

  try {
    while (fgets(buffer, sizeof buffer, pipe) != NULL) {
      result += buffer;
    }
  } catch (...) {
    pclose(pipe);
    return false;
  }

  return result.find("ebUniversalProForEthernet") != string::npos;
}

bool CameraDriver::load_calibration(uint32_t sid, std::string cname){
  std::string param = cname + "_calibration_file";
  std::string kfile_param;
  const std::string prefix("file://");

  if((sid != LEFTCAM) && (sid != RIGHTCAM)){
    return false;
  }
  if(cname.size() == 0){
    return false;
  }
      
  if(this->has_parameter(param)){
    kfile_param = this->get_parameter(param).as_string();        
  }
  else{
    std::string default_url = prefix + ament_index_cpp::get_package_share_directory(this->get_name()) + "/config/" + cname + ".yaml";
    kfile_param = this->declare_parameter(param, default_url);      
  }
  if(kfile_param.rfind(prefix, 0) != 0){
    fs::path absolutePath = fs::absolute(kfile_param);
    kfile_param = prefix + absolutePath.string();
  }
  
  if(!m_cinfo_manager[sid]){
    m_cinfo_manager[sid] = std::make_shared<camera_info_manager::CameraInfoManager>(this, cname, kfile_param);
  } else{
    m_cinfo_manager[sid]->setCameraName(cname);
    m_cinfo_manager[sid]->loadCameraInfo(kfile_param);
  }
  
  return m_cinfo_manager[sid]->isCalibrated();  
}

uint32_t CameraDriver::get_num_sensors(){
  PvGenParameter *lGenParameter = m_device->GetParameters()->Get("DeviceModelName");
  PvString sensor_model;
  PvResult res = dynamic_cast<PvGenString *>(lGenParameter)->GetValue(sensor_model);
  uint32_t num_sensors = 0;
  
  if((res.IsOK()) && (sensor_model.GetLength() > 0)){
      std::string model(sensor_model.GetAscii());                
      num_sensors = (std::toupper(model.back()) != 'M') + 1;        
  }           
  
  return num_sensors;
}

bool CameraDriver::set_register(std::string regname, std::variant<int64_t, double, bool> regvalue){
  PvGenParameter *lGenParameter = m_device->GetParameters()->Get(regname.c_str());
  PvGenType lGenType;
  PvResult res = lGenParameter->GetType(lGenType);
  if(!res.IsOK()) return false;

  switch (lGenType){
    case PvGenTypeInteger:
      res = dynamic_cast<PvGenInteger *>(lGenParameter)->SetValue(std::get<int64_t>(regvalue));
      break;
    case PvGenTypeFloat:
      res = dynamic_cast<PvGenFloat *>(lGenParameter)->SetValue(std::get<double>(regvalue));
      break;
    case PvGenTypeBoolean:
      res = dynamic_cast<PvGenBoolean *>(lGenParameter)->SetValue(std::get<bool>(regvalue));
      break;
    case PvGenTypeCommand:
      if(std::get<bool>(regvalue)){                
          res = dynamic_cast<PvGenCommand *>(lGenParameter)->Execute();
      }
      break;
    default:
      return false;
  }

  return res.IsOK();
}

static void decompose_projection(double *pm, double *tvec, double *rvec){
  if((pm == NULL) || (tvec == NULL) || (rvec == NULL)) return;
    
  cv::Mat P(3, 4, CV_64F, pm); //projection matrix
  cv::Mat K(3, 3, CV_64F); // intrinsic parameter matrix
  cv::Mat R(3, 3, CV_64F); // rotation matrix
  cv::Mat T(4, 1, CV_64F, tvec); // translation vector
  cv::Mat r(1, 3, CV_64F, rvec); // rotation vector

  cv::decomposeProjectionMatrix(P, K, R, T);  
  cv::Rodrigues(R, r);            
  
  for(uint32_t i = 0; i < 3; ++i){
    tvec[i] /= tvec[3];
  }                
}

static bool make_calibration_registers(uint32_t sid, sensor_msgs::msg::CameraInfo cam, 
                                       std::map<std::string, std::variant<int64_t, double, bool>> &kparams){

  double tvec[4] = {0.0};
  double rvec[3] = {0.0};

  if(cam.distortion_model != "plumb_bob"){    
    return false;
  }

  decompose_projection(cam.p.data(), tvec, rvec);
  std::string id = std::to_string(sid);

  kparams["fx" + id] = cam.k[0];
  kparams["fy" + id] = cam.k[4];
  kparams["cx" + id] = cam.k[2];
  kparams["cy" + id] = cam.k[5];
  
  kparams["k1" + id] = cam.d[0];
  kparams["k2" + id] = cam.d[1];
  kparams["k3" + id] = cam.d[4];  
  kparams["p1" + id] = cam.d[2];
  kparams["p2" + id] = cam.d[3];
              
  kparams["tx" + id] = tvec[0];
  kparams["ty" + id] = tvec[1];
  kparams["tz" + id] = tvec[2];          
  kparams["rx" + id] = rvec[0];
  kparams["ry" + id] = rvec[1];
  kparams["rz" + id] = rvec[2];
  
  kparams["kWidth"] = cam.width;
  kparams["kHeight"] = cam.height;

  return true;
}

bool CameraDriver::set_calibration(){
  uint32_t num_sensors = get_num_sensors();
  bool calibrated = false;
  std::map<std::string, std::variant<int64_t, double, bool>> kregisters;

  if(num_sensors == 1){
    calibrated = load_calibration(LEFTCAM, "camera");
  } else if(num_sensors == 2){
    calibrated = load_calibration(LEFTCAM, "left_camera");
    calibrated &= load_calibration(RIGHTCAM, "right_camera");
  } 

  if(calibrated){
    calibrated = false;
    for(uint32_t i = 0; i < num_sensors; ++i){
      if(!make_calibration_registers(i, m_cinfo_manager[i]->getCameraInfo(), kregisters)){
        RCLCPP_ERROR(get_logger(), "Only Plumb_bob calibration model supported!");
        return calibrated;
      }      
    }

    for(auto &kreg:kregisters){
      if(!set_register(kreg.first, kreg.second)){
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to set camera register [" << kreg.first << "]");
        return calibrated;
      }      
    }

    if(set_register("saveCalibrationData", true)){            
      calibrated = set_register("Undistortion", true);
      if(num_sensors == 2) calibrated &= set_register("Rectification", true);
      if(!calibrated){
        RCLCPP_ERROR(get_logger(), "Failed to trigger Undistortion/Rectification mode on camera.");
      }
    } else{
      RCLCPP_ERROR(get_logger(), "Failed to trigger calibration mode on camera.");
    }
  }
  
  return calibrated;
}

} // namespace bottlenose_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bottlenose_camera_driver::CameraDriver)
