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
#include <utility>
#include <string>
#include <cassert>
#include <list>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <arpa/inet.h>  // For inet_ntoa
#include <netinet/in.h> // For struct in_addr

#include <PvDevice.h>
#include <PvStream.h>
#include <PvSystem.h>
#include <PvBuffer.h>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

#include "bottlenose_camera_driver.hpp"
#include "bottlenose_parameters.hpp"
#include "bottlenose_chunk_parser.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/stat.h>

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

  static bool parseMatrix(const std::string &str, float matrix[3][3]) {
    std::istringstream iss(str);
    char ch;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (!(iss >> matrix[i][j])) {
          return false;  // Failed to read a float
        }
        if (j < 2) {  // Expecting a comma after first two elements of each row
          if (!(iss >> ch) || ch != ',') {
            return false;
          }
        }
      }
      if (i < 2) {  // Expecting a semicolon after first two rows
        if (!(iss >> ch) || ch != ';') {
          return false;
        }
      }
    }

    return true;  // Successfully parsed the matrix
  }

  static pair<uint64, uint64> convertTimestamp(uint64_t in) {
    uint64_t seconds = in / 1e3; // milliseconds
    uint64_t nanoseconds = (in - seconds * 1e3) * 1e6; // nanoseconds
    return std::make_pair(seconds, nanoseconds);
  }

  /**
   * @brief Callback for reading data from curl.
   * @param ptr Buffer to write to.
   * @param size Size of each element to read.
   * @param nmemb Number of elements to read.
   * @param stream Opened file stream.
   * @return IO return code.
   */
  static size_t curlReadCallback(void *ptr, size_t size, size_t nmemb, FILE *stream) {
    size_t retcode = fread(ptr, size, nmemb, stream);
    return retcode;
  }

  /**
   * @brief Convert integer coded IP address to string.
   * @param ip IP address.
   * @return String representing the IP address.
   */
  string ipv4ToString(uint32_t ip) {
    struct in_addr ip_addr;
    ip_addr.s_addr = htonl(ip); // Ensure network byte order
    return std::string(inet_ntoa(ip_addr));
  }

CameraDriver::CameraDriver(const rclcpp::NodeOptions &node_options)
  : Node("bottlenose_camera_driver", node_options),
  m_calibrated(false),
  m_terminate(false)
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

  for(auto &parameter : bottlenose_parameters) {
    this->declare_parameter(parameter.name, parameter.default_value);
  }

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  m_image_color = image_transport::create_camera_publisher(this, "image_color", custom_qos_profile);
  m_image_color_1 = image_transport::create_camera_publisher(this, "image_color_1", custom_qos_profile);

  m_keypoints = create_publisher<visualization_msgs::msg::ImageMarker>("features", 10);
  m_keypoints_1 = create_publisher<visualization_msgs::msg::ImageMarker>("features_1", 10);

  m_detections = create_publisher<vision_msgs::msg::Detection2DArray>("detections", 10);

  m_pointcloud = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

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
    auto ts = convertTimestamp(timestamp);
    ros_image.header.stamp.nanosec = ts.second;
    ros_image.header.stamp.sec = ts.first;
//    RCLCPP_DEBUG(get_logger(), "Decoded timestamp %9ld %09ld", seconds, nanoseconds);
    ros_image.header.frame_id = this->get_parameter("frame_id").as_string();

    auto msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>(ros_image);
    return msg_ptr_;
  }
  return nullptr;
}

bool CameraDriver::update_runtime_parameters() {
  // All integer parameters
  for (auto param: {"dgainBlue",
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
      if (get<int64_t>(value) == val) {
        continue;
      }
    } catch (std::out_of_range &e) {}

    PvResult res = intval->SetValue(val);
    if (res.IsFailure()) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val << " cause "
                                                                  << res.GetDescription().GetAscii());
      return false;
    }
    // Cache
    m_camera_parameter_cache[param] = val;
    RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter " << param << " to " << val);
  }
  PvGenFloat *floatVal = static_cast<PvGenFloat *>( m_device->GetParameters()->Get("gamma"));
  double val = get_parameter("gamma").as_double();
  bool dirty = true;
  try {
    auto value = m_camera_parameter_cache.at("gamma");
    if (get<double>(value) == val) {
      dirty = false;
    }
  } catch (std::out_of_range &e) {}
  if(dirty) {
    PvResult res = floatVal->SetValue(val);
    if (res.IsFailure()) {
      RCLCPP_WARN_STREAM(get_logger(),
                         "Could not set parameter gamma to " << val << " cause " << res.GetDescription().GetAscii());
      return false;
    }
    // Cache
    m_camera_parameter_cache["gamma"] = val;
    RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter gamma to " << val);
  }

  bool enable_awb = get_parameter("wbAuto").as_bool();
  PvGenBoolean *gev_awb = dynamic_cast<PvGenBoolean *>( m_device->GetParameters()->Get("wbAuto"));
  if(gev_awb == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not enable auto white balance ... are you running the latest firmware?");
    return false;
  }
  try {
    auto value = m_camera_parameter_cache.at("wbAuto");
    if (get<int64_t>(value) != (int64_t)(enable_awb)) {
      PvResult res = gev_awb->SetValue(enable_awb);
      if(!res.IsOK()) {
        RCLCPP_ERROR_STREAM(get_logger(), "Could not configure auto white balance, cause: " << res.GetDescription().GetAscii());
        return false;
      }
      RCLCPP_DEBUG(get_logger(), "Set auto white balance to %d", enable_awb);
    }
  } catch (std::out_of_range &e) {
    // Undefined, set as well
    PvResult res = gev_awb->SetValue(enable_awb);
    if(!res.IsOK()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not configure auto white balance, cause: " << res.GetDescription().GetAscii());
      return false;
    }
    RCLCPP_DEBUG(get_logger(), "Set auto white balance to %d", enable_awb);
  }
  // propagate cache
  m_camera_parameter_cache["wbAuto"] = (int64_t)(enable_awb);

  if(!enable_awb) {
    for (auto param: {"wbBlue", "wbGreen", "wbRed"}) {
      floatVal = static_cast<PvGenFloat *>( m_device->GetParameters()->Get(param));
      double val = get_parameter(param).as_double();
      try {
        auto value = m_camera_parameter_cache.at(param);
        if (get<double>(value) == val) {
          continue;
        }
      } catch (std::out_of_range &e) {}

      PvResult res = floatVal->SetValue(val);
      if (res.IsFailure()) {
        RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val << " cause " << res.GetDescription().GetAscii());
        return false;
      }
      // Cache
      m_camera_parameter_cache[param] = val;
      RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter " << param << " to " << val);
    }
  }

  // Only if auto exposure is not enabled
  bool enable_aexp = get_parameter("autoExposureEnable").as_bool();
  if(!enable_aexp) {
    for (auto param: {"exposure",
                      "gain"}) {
      PvGenFloat *floatVal = dynamic_cast<PvGenFloat *>( m_device->GetParameters()->Get(param));
      if(floatVal == nullptr) {
        RCLCPP_ERROR(get_logger(), "Could not configure parameter %s, please check you are running the latest firmware", param);
        return false;
      }
      double val = get_parameter(param).as_double();
      try {
        auto value = m_camera_parameter_cache.at(param);
        if (get<double>(value) == val) {
          continue;
        }
      } catch (std::out_of_range &e) {}

      PvResult res = floatVal->SetValue(val);
      if (res.IsFailure()) {
        RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val);
        return false;
      }
      // Cache
      m_camera_parameter_cache[param] = val;
      RCLCPP_DEBUG_STREAM(get_logger(), "Set parameter " << param << " to " << val);
    }
  }
  if(!set_ccm_profile()) {
    return false;
  }
  return set_ccm_custom();
}

bool CameraDriver::is_stereo() {
  return get_num_sensors() == 2;
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

  // Apply Offsets
  for (auto param: {"OffsetX", "OffsetY"}) {
    PvGenInteger *intval = static_cast<PvGenInteger *>( m_device->GetParameters()->Get(param));
    int val = get_parameter(param).as_int();
    PvResult res = intval->SetValue(val);
    if (res.IsFailure()) {
      RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val << " cause "
                                                                  << res.GetDescription().GetAscii());
      return false;
    }
  }
  if(is_stereo()) {
    for (auto param: {"OffsetX1", "OffsetY1"}) {
      PvGenInteger *intval = static_cast<PvGenInteger *>( m_device->GetParameters()->Get(param));
      int val = get_parameter(param).as_int();
      PvResult res = intval->SetValue(val);
      if (res.IsFailure()) {
        RCLCPP_WARN_STREAM(get_logger(), "Could not set parameter " << param << " to " << val << " cause "
                                                                    << res.GetDescription().GetAscii());
        return false;
      }
    }
  }
  m_width = width;
  m_height = height;


  RCLCPP_DEBUG_STREAM(get_logger(), "Offsets applied " << format);

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

bool CameraDriver::set_ccm_custom() {
  auto ccm_custom_str = get_parameter("CCMCustom").as_string();
  if(!ccm_custom_str.empty()) {
    try {
      auto value = m_camera_parameter_cache.at("CCMCustom");
      if(get<string>(value) == ccm_custom_str) {
        return true;
      }
    } catch(std::out_of_range &e) { }

    // Apply custom color profile
    float matrix[3][3];
    if(!parseMatrix(ccm_custom_str, matrix)) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid custom color profile: " << ccm_custom_str);
      return false;
    }
    // Apply values
    for(size_t row = 0; row < 3; row++) {
      for(size_t col = 0; col < 3; col++) {
        std::string param = "CCMValue" + std::to_string(row) + std::to_string(col);
        if(!set_register(param, matrix[row][col])) {
          RCLCPP_ERROR_STREAM(get_logger(), "Could not configure custom color profile");
          return false;
        }
      }
    }
    PvGenCommand * cmd = dynamic_cast<PvGenCommand *>( m_device->GetParameters()->Get("SetCustomProfile"));
    PvResult res = cmd->Execute();
    if(res.IsFailure()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not apply custom color profile");
      return false;
    }
    // Wait for parameter pass-through
    WAIT_PROPAGATE();
    // Check the return code
    PvGenString *strStatus = dynamic_cast<PvGenString*>( m_device->GetParameters()->Get("CCM0Status"));
    if(!strStatus) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not apply custom color profile");
      return false;
    }
    PvString statusVal;
    res = strStatus->GetValue(statusVal);
    if(res.IsFailure()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not apply custom color profile");
      return false;
    }
    if(string("Custom CCM profile set") != statusVal.GetAscii()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not apply custom color profile");
      return false;
    }
    //"Custom CCM profile set"

    // Apply to cache
    m_camera_parameter_cache["CCMCustom"] = ccm_custom_str;
    RCLCPP_DEBUG_STREAM(get_logger(), "Applied custom color profile: " << ccm_custom_str);
  }
  return true;
}

bool CameraDriver::set_stereo() {
    // Ignore for bottlenose mono
    if(!is_stereo()) {
        return true;
    }
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

bool CameraDriver::set_auto_exposure() {
  bool enable_aexp = get_parameter("autoExposureEnable").as_bool();
  RCLCPP_DEBUG_STREAM(get_logger(), "Auto exposure set to " << enable_aexp);
  // Apply parameter to GEV
  PvGenBoolean *gev_aexp = dynamic_cast<PvGenBoolean *>( m_device->GetParameters()->Get("autoExposureEnable"));
  if(gev_aexp == nullptr) {
    RCLCPP_ERROR(get_logger(), "Could not enable auto exposure ... are you running the latest firmware?");
    return false;
  }
  PvResult res = gev_aexp->SetValue(enable_aexp);
  if(!res.IsOK()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure auto exposure, cause: " << res.GetDescription().GetAscii());
    return false;
  }
  if(enable_aexp) {
    PvGenInteger *gev_aexp_target = dynamic_cast<PvGenInteger *>( m_device->GetParameters()->Get("autoExposureLuminanceTarget"));
    if(gev_aexp_target == nullptr) {
      RCLCPP_ERROR(get_logger(), "Unable to register luminance target");
      return false;
    }
    res = gev_aexp_target->SetValue(get_parameter("autoExposureLuminanceTarget").as_int());
    if(!res.IsOK()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not configure luminance target, cause: " << res.GetDescription().GetAscii());
      return false;
    }
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
  m_device = dynamic_cast<PvDeviceGEV *>( device );
  m_stream = dynamic_cast<PvStreamGEV *>( stream );
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

  // Stream settings
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
  // Wipe parameter cache as we cannot control what was set before reconnect
  m_camera_parameter_cache.clear();
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

void CameraDriver::publish_features(const std::vector<keypoints_t> &features, const uint64_t &timestamp) {
  auto ts = convertTimestamp(timestamp);

  for(auto &feature_list : features) {
    visualization_msgs::msg::ImageMarker marker;
    marker.type = visualization_msgs::msg::ImageMarker::POINTS;
    marker.scale = 3;
    marker.filled = 0;
    marker.action = visualization_msgs::msg::ImageMarker::ADD;
    std_msgs::msg::ColorRGBA outline_color;
    outline_color.r = 1.0;
    outline_color.g = 0.0;
    outline_color.b = 0.0;
    outline_color.a = 1.0;
    marker.header.frame_id = this->get_parameter("frame_id").as_string();
    marker.header.stamp.sec = ts.first;
    marker.header.stamp.nanosec = ts.second;
    marker.ns = "features";
    for(size_t i = 0; i < feature_list.count; i++) {
      auto pt = geometry_msgs::msg::Point();
      pt.x = feature_list.points[i].x;
      pt.y = feature_list.points[i].y;
      pt.z = 0;
//      RCLCPP_DEBUG(get_logger(), "(%f, %f)", pt.x, pt.y);
      marker.points.push_back(pt);
      marker.outline_colors.push_back(outline_color);
    }
    if(feature_list.fid == 0 || feature_list.fid == 2) {
      m_keypoints->publish(marker);
      RCLCPP_DEBUG(get_logger(), "Published %zu keypoints stream %i fid %i", marker.points.size(), 0, feature_list.fid);
    } else if(feature_list.fid == 1 || feature_list.fid == 3) {
      m_keypoints_1->publish(marker);
      RCLCPP_DEBUG(get_logger(), "Published %zu keypoints stream %i fid %i", marker.points.size(), 1, feature_list.fid);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid feature list id %d", feature_list.fid);
    }
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
  if(!set_auto_exposure()) {
    disconnect();
    done = true;
    return;
  }
  if(!enable_ntp(get_parameter("ntpEnable").as_bool())) {
    disconnect();
    done = true;
    return;
  }
  // Enable chunk data for meta information to have reliable timestamping at source
  if(!set_chunk("FrameInformation", true)) {
    disconnect();
    done = true;
    return;
  }
  // Configure feature point
  if(!configure_feature_points()) {
    disconnect();
    done = true;
    return;
  }
  if(!configure_point_cloud()) {
    disconnect();
    done = true;
    return;
  }
  // Configure ai model
  if(!configure_ai_model()) {
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

  // Buffer for keypoint chunk decoding
  vector<keypoints_t> feature_points;

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
        info_t info = {};
        bboxes_t bboxes = {};
        uint64_t timestamp = 0;
        pointcloud_t point_cloud = {};
        switch ( buffer->GetPayloadType() ) {
          case PvPayloadTypeImage:
            img0 = buffer->GetImage();
            // Timestamp handling
            timestamp = buffer->GetTimestamp(); // Fallback timestamp, use Pleora local time
            if(chunkDecodeMetaInformation(buffer, &info)) {
              RCLCPP_DEBUG_STREAM(get_logger(), "Bottlenose time: " << ms_to_date_string(info.real_time));
              timestamp = info.real_time;
            } else {
              RCLCPP_WARN(get_logger(), "Could not decode meta information");
            }
            if(chunkDecodeKeypoints(buffer, feature_points)) {
              this->publish_features(feature_points, timestamp);
            }
            if(chunkDecodeBoundingBoxes(buffer, bboxes)) {
              this->publish_bboxes(bboxes, timestamp);
            }
            if(chunkDecodePointCloud(buffer, point_cloud)) {
              this->publish_pointcloud(point_cloud, timestamp);
            }
            m_image_msg = convertFrameToMessage(img0, timestamp);

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
            // Timestamp handling
            timestamp = buffer->GetTimestamp(); // Fallback timestamp, use Pleora local time
            if(chunkDecodeMetaInformation(buffer, &info)) {
              RCLCPP_DEBUG_STREAM(get_logger(), "Bottlenose time: " << ms_to_date_string(info.real_time));
              timestamp = info.real_time;
            } else {
              RCLCPP_WARN(get_logger(), "Could not decode meta information");
            }
            if(chunkDecodeKeypoints(buffer, feature_points)) {
              this->publish_features(feature_points, timestamp);
            }
            if(chunkDecodeBoundingBoxes(buffer, bboxes)) {
              this->publish_bboxes(bboxes, timestamp);
            }
            if(chunkDecodePointCloud(buffer, point_cloud)) {
              this->publish_pointcloud(point_cloud, timestamp);
            }
            m_image_msg = convertFrameToMessage(img0, timestamp);
            m_image_msg_1 = convertFrameToMessage(img1, timestamp);

            if(m_image_msg != nullptr) {
                RCLCPP_DEBUG(get_logger(), "Received left Image %i x %i", img0->GetWidth(), img0->GetHeight());
                sensor_msgs::msg::CameraInfo::SharedPtr info_msg(
                        new sensor_msgs::msg::CameraInfo(m_cinfo_manager[0]->getCameraInfo()));
                info_msg->header = m_image_msg->header;
                m_image_color.publish(m_image_msg, info_msg);
            }
            if(m_image_msg_1 != nullptr) {
                RCLCPP_DEBUG(get_logger(), "Received Right Image %i x %i", img1->GetWidth(), img1->GetHeight());
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
          m_stream->QueueBuffer( buffer );
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

void CameraDriver::publish_bboxes(const bboxes_t &bboxes, const uint64_t &timestamp) {
  auto ts = convertTimestamp(timestamp);
  vision_msgs::msg::Detection2DArray msg;

  for(size_t i = 0; i < bboxes.count; i++) {
    auto height = bboxes.box[i].bottom - bboxes.box[i].top;
    auto width = bboxes.box[i].right - bboxes.box[i].left;

    // Encode the bounding box
    vision_msgs::msg::BoundingBox2D bbox;
    vision_msgs::msg::ObjectHypothesisWithPose hyp;

    bbox.size_x = width;
    bbox.size_y = height;

#if ROS_VERSION_MAJOR == 2 && ROS_VERSION_MINOR == 0
    bbox.center.x = bboxes.box[i].left + width / 2.0;
    bbox.center.y = bboxes.box[i].top + height / 2.0;

    // Encode the class
    hyp.score = bboxes.box[i].score;
    hyp.id = bboxes.box[i].label;
#else // Humble
    bbox.center.position.x = bboxes.box[i].left + width / 2.0;
    bbox.center.position.y = bboxes.box[i].top + height / 2.0;

    // Encode the class
    hyp.hypothesis.class_id = bboxes.box[i].label;
    hyp.hypothesis.score = bboxes.box[i].score;
#endif

    vision_msgs::msg::Detection2D det;
    det.bbox = bbox;
    det.results.push_back(hyp);

    msg.detections.push_back(det);
  }
  msg.header.frame_id = this->get_parameter("frame_id").as_string();
  msg.header.stamp.sec = ts.first;
  msg.header.stamp.nanosec = ts.second;
  m_detections->publish(msg);
  RCLCPP_DEBUG(get_logger(), "Published %u bounding boxes stream %i fid %i", bboxes.count, 0, bboxes.fid);
}

void CameraDriver::publish_pointcloud(const pointcloud_t &pointcloud, const uint64_t &timestamp) {
  auto ts = convertTimestamp(timestamp);

  sensor_msgs::msg::PointField x_field;
  x_field.name = "x";
  x_field.offset = 0; // Assuming 'x' is the first field, so offset is 0
  x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  x_field.count = 1; // Typically, each coordinate is just one element


  sensor_msgs::msg::PointField y_field;
  y_field.name = "y";
  y_field.offset = 4; // Assuming 'x' is 4 bytes, 'y' starts after 'x'
  y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  y_field.count = 1;

  sensor_msgs::msg::PointField z_field;
  z_field.name = "z";
  z_field.offset = 8; // Assuming 'x' and 'y' are 4 bytes each, 'z' starts after 'y'
  z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  z_field.count = 1;

  sensor_msgs::msg::PointCloud2 msg;
  // Define fields
  msg.fields.push_back(x_field);
  msg.fields.push_back(y_field);
  msg.fields.push_back(z_field);
  // Set other necessary fields
  msg.height = 1;  // If your cloud is unordered, height is 1
  msg.width = pointcloud.count;  // Number of points in the cloud

  // Point step is the size of a point in bytes
  msg.point_step = sizeof(float) * 3;
  msg.row_step = msg.point_step * msg.width; // Row step is point step times the width
  msg.is_dense = false;  // If there are no invalid (NaN, Inf) points, set to true

  msg.data.resize(msg.row_step * msg.height);

  for(size_t i = 0; i < pointcloud.count; i++) {
    // Copy data into cloud_msg.data
    std::memcpy(&msg.data[i * msg.point_step + x_field.offset], &pointcloud.points[i].x, sizeof(float));
    std::memcpy(&msg.data[i * msg.point_step + y_field.offset], &pointcloud.points[i].y, sizeof(float));
    std::memcpy(&msg.data[i * msg.point_step + z_field.offset], &pointcloud.points[i].z, sizeof(float));
//    std::memcpy(&msg.data[i * msg.point_step + intensity_field.offset], &intensity, sizeof(float));
  }
  msg.header.frame_id = this->get_parameter("frame_id").as_string();
  msg.header.stamp.sec = ts.first;
  msg.header.stamp.nanosec = ts.second;
  m_pointcloud->publish(msg);
  RCLCPP_DEBUG(get_logger(), "Decoded point cloud with %u points", pointcloud.count);
}

bool CameraDriver::is_streaming() {
  return !done && m_device != nullptr && m_stream != nullptr && m_stream->IsOpen();
}

bool CameraDriver::is_ebus_loaded() {
  char buffer[128];
  std::string result;
  FILE* pipe = popen("/usr/sbin/lsmod", "r");
  if (!pipe)
    return false;

  try {
    while (fgets(buffer, sizeof buffer, pipe) != nullptr) {
      result += buffer;
    }
  } catch (...) {
    pclose(pipe);
    return false;
  }

  return result.find("ebUniversalProForEthernet") != string::npos;
}

bool CameraDriver::set_chunk(std::string chunk, bool enable) {
  // Enable chunk mode for any chunk
  if(!set_register("ChunkModeActive", true)) {
    RCLCPP_ERROR(get_logger(), "Could not enable basic chunk output");
    return false;
  }

  // Select the appropriate enumerator for chunk
  if(!set_enum_register("ChunkSelector", chunk)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not select chunk: " << chunk);
    return false;
  }

  // Set the selected chunk as enabled or disabled
  if(!set_register("ChunkEnable", enable)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not enable chunk: " << chunk);
    return false;
  }

  return true;
}

bool CameraDriver::enable_ntp(bool enable) {
  if(!set_register("ntpEnable", enable)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure NTP to be " << enable);
    return false;
  }
  return true;
}

bool CameraDriver::configure_feature_points() {
  bool enabled = false;
  if(get_parameter("feature_points").as_string() == "none") {
    // Disable feature_points
    RCLCPP_DEBUG(get_logger(), "Disabling feature points");
    return set_chunk("FeaturePoints", false);
  } else if(get_parameter("feature_points").as_string() == "gftt") {
    RCLCPP_DEBUG(get_logger(), "Enabling gftt features");
    if(!set_enum_register("KPCornerType", "GFTT")) {
      RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
      return false;
    }
    string detector_type = get_parameter("gftt_detector").as_string();
    if(detector_type == "harris") {
      if(!set_enum_register("KPDetector", "Harris")) {
        RCLCPP_ERROR(get_logger(), "Could not configure gftt_detector");
        return false;
      }
    } else if(detector_type == "eigen") {
      if(!set_enum_register("KPDetector", "Min-Eigen")) {
        RCLCPP_ERROR(get_logger(), "Could not configure gftt_detector");
        return false;
      }
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid setting for \'gftt_detector\' = "
        << detector_type << "valid choices are {'harris', 'eigen'}");
      return false;
    }
    if(!set_register("KPQualityLevel", get_parameter("features_quality").as_int())) {
      RCLCPP_ERROR(get_logger(), "Could not configure features_quality");
      return false;
    }
    if(!set_register("KPMinimunDistance", get_parameter("features_min_distance").as_int())) {
      RCLCPP_ERROR(get_logger(), "Could not configure features_min_distance");
      return false;
    }
    if(!set_register("KPHarrisParamK", get_parameter("features_harrisk").as_double())) {
      RCLCPP_ERROR(get_logger(), "Could not configure features_harrisk");
      return false;
    }
    enabled = true;
  } else if(get_parameter("feature_points").as_string() == "fast9") {
    RCLCPP_DEBUG(get_logger(), "Enabling fast9 features");
    if(!set_enum_register("KPCornerType", "Fast9n")) {
      RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
      return false;
    }
    if(!set_register("KPThreshold", get_parameter("features_threshold").as_int())) {
      RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
      return false;
    }
    enabled = true;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid setting for \'feature_points\' = "
      << get_parameter("feature_points").as_string() << "valid choices are {'none', 'gftt', 'fast9'}");
    return false;
  }
  if(enabled) {
    if(!set_register("KPMaxNumber", get_parameter("features_max").as_int())) {
      RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
      return false;
    }
    if(!set_chunk("FeaturePoints", true)) {
      RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
      return false;
    }
  }
  return true;
}

bool CameraDriver::configure_point_cloud() {
  if(!is_stereo()) {
    return true;
  }

  bool enabled = get_parameter("sparse_point_cloud").as_bool();
  if(!set_chunk("SparsePointCloud", enabled)) {
    RCLCPP_ERROR(get_logger(), "Could not configure point cloud");
    return false;
  }
  if(enabled) {
    int akazeValue = get_parameter("AKAZELength").as_int();
    string value = std::to_string(akazeValue) + "-Bits";
    if(!set_enum_register("AKAZELength", value)) {
      RCLCPP_ERROR(get_logger(), "Could not configure AKAZELength");
      return false;
    }
    akazeValue = get_parameter("AKAZEWindow").as_int();
    value = std::to_string(akazeValue) + "x" + std::to_string(akazeValue) + "-Window";
    if(!set_enum_register("AKAZEWindow", value)) {
      RCLCPP_ERROR(get_logger(), "Could not configure AKAZEWindow");
      return false;
    }

    for(auto value : {"HAMATXOffset",
                        "HAMATYOffset",
                        "HAMATRect1X",
                        "HAMATRect1Y",
                        "HAMATRect2X",
                        "HAMATRect2Y",
                        "HAMATMinThreshold",
                        "HAMATRatioThreshold"}) {
      if(!set_register(value, get_parameter(value).as_int())) {
        RCLCPP_ERROR(get_logger(), "Could not configure %s", value);
        return false;
      }
    }
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Sparse point cloud configured to " << enabled);

  return true;
}

bool CameraDriver::configure_ai_model() {
  bool enabled = false;
  if(get_parameter("ai_model").as_string().empty()) {
    RCLCPP_DEBUG(get_logger(), "AI model disabled");
  } else {
    // Figure out network configuration
    PvGenInteger *address = static_cast<PvGenInteger *>( m_device->GetParameters()->Get("GevCurrentIPAddress"));
    PvGenBoolean *enable = static_cast<PvGenBoolean *>( m_device->GetParameters()->Get("EnableWeightsUpdate"));
    PvGenString *model = static_cast<PvGenString *>( m_device->GetParameters()->Get("DNNStatus"));
    if (enable == nullptr || address == nullptr || model == nullptr) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model, please update your firmware");
      return false;
    }
    int64_t intIpAddress;
    PvResult res = address->GetValue(intIpAddress);
    if (res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not enumerate network address");
      return false;
    }
    string ipAddress = ipv4ToString(intIpAddress);
    bool status;
    res = enable->GetValue(status);
    if (res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not determine model status");
      return false;
    }
    // Reset transfer
    if (status) {
      res = enable->SetValue(false);
      if (res.IsFailure()) {
        RCLCPP_ERROR(get_logger(), "Could not reset model transfer");
        return false;
      }
    }
    res = enable->SetValue(true);
    if (res.IsFailure()) {
      RCLCPP_ERROR(get_logger(), "Could not initiate model transfer");
      return false;
    }
    size_t trials = 10;
    PvString modelStatus;
    while (trials-- > 0) {
      res = model->GetValue(modelStatus);
      string modelStatusStr = modelStatus.GetAscii();
      if (res.IsFailure()) {
        RCLCPP_ERROR(get_logger(), "Could not determine model status");
        return false;
      }
      if (modelStatusStr.find("FTP running") != std::string::npos) {
        break;
      }
      WAIT_PROPAGATE();
    }
    if(trials == 0) {
      RCLCPP_ERROR(get_logger(), "Could not file transfer");
      return false;
    }
    filesystem::path fsPath(get_parameter("ai_model").as_string());
    string basename = fsPath.filename().string();
    string target = string("ftp://anonymous:@") + ipAddress + "/" + basename;
    if(!ftp_upload(target, get_parameter("ai_model").as_string())) {
      return false;
    }
    trials = 10;
    while (trials-- > 0) {
      res = model->GetValue(modelStatus);
      string modelStatusStr = modelStatus.GetAscii();
      if (res.IsFailure()) {
        RCLCPP_ERROR(get_logger(), "Could not determine model status");
        return false;
      }
      if (modelStatusStr.find("Loaded") != std::string::npos) {
        break;
      }
      usleep(100000);
    }
    if(trials == 0) {
      RCLCPP_ERROR(get_logger(), "Could not initialize model");
      return false;
    } else {
      RCLCPP_DEBUG(get_logger(), "Model loaded, last status %s", modelStatus.GetAscii());
    }

    // Disable debugging
    if(!set_register("DNNDrawOnStream", false)) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNDrawOnStream");
      return false;
    }
    // Enable label output
    if(!set_register("DNNOutputLabels", true)) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNOutputLabels");
      return false;
    }
//    // Set DNN parameters : invalid for bboxes
//    if(!set_register("DNNTopK", get_parameter("DNNTopK").as_int())) {
//      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNTopK");
//      return false;
//    }
    if(!set_register("DNNMaxDetections", get_parameter("DNNMaxDetections").as_int())) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNMaxDetections");
      return false;
    }
    if(!set_register("DNNNonMaxSuppression", get_parameter("DNNNonMaxSuppression").as_double())) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNNonMaxSuppression");
      return false;
    }
    if(!set_register("DNNConfidence", get_parameter("DNNConfidence").as_double())) {
      RCLCPP_ERROR(get_logger(), "Could not configure AI model: DNNConfidence");
      return false;
    }
    enabled = true;
  }

  // Enable or disable model based on outcome of previous check
  if(!set_register("DNNEnable", enabled)) {
    RCLCPP_ERROR(get_logger(), "Could not configure DNNEnable");
    return false;
  }
  if(!set_chunk("BoundingBoxes", enabled)) {
    RCLCPP_ERROR(get_logger(), "Could not configure feature_points");
    return false;
  }
  return true;
}

bool CameraDriver::load_calibration(uint32_t sid, std::string cname){
  std::string param = cname + "_calibration_file";
  std::string kfile_param;
  const std::string prefix("file://");

  if((sid != LEFTCAM) && (sid != RIGHTCAM)){
    return false;
  }
  if(cname.empty()){
    return false;
  }
      
  if(this->get_parameter(param).as_string().length() > 0) {
    kfile_param = this->get_parameter(param).as_string();        
  }
  else{
    std::string default_url = prefix + ament_index_cpp::get_package_share_directory(this->get_name()) + "/config/" + cname + ".yaml";
    this->set_parameter(rclcpp::Parameter(param, default_url));
    kfile_param = this->get_parameter(param).as_string();
//    kfile_param = this->declare_parameter(param, default_url);
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
  if(lGenParameter == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure parameter " << regname << "please check you are running the latest firmware");
    return false;
  }
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

bool CameraDriver::set_enum_register(std::string regname, std::string value) {
  PvGenParameter *lGenParameter = m_device->GetParameters()->Get(regname.c_str());
  if(lGenParameter == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not configure parameter " << regname << "please check you are running the latest firmware");
    return false;
  }

  PvGenType lGenType;
  PvResult res = lGenParameter->GetType(lGenType);
  if(!res.IsOK())
    return false;

  if(lGenType == PvGenTypeEnum){
    PvString lValue = value.c_str();
    PvGenEnum *lGenEnum = dynamic_cast<PvGenEnum *>( lGenParameter );
    res = lGenEnum->SetValue(lValue);
    if(!res.IsOK()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not set enum register: " << regname << " to " << value
        << " cause: " << res.GetDescription().GetAscii());
      int64_t count;
      res = lGenEnum->GetEntriesCount(count);
      if(res) {
        for(int64_t i = 0; i < count; i++) {
          const PvGenEnumEntry *entry;
          res = lGenEnum->GetEntryByIndex(i, &entry);
          if(res) {
            PvString name;
            res = entry->GetName(name);
            if(res) {
              RCLCPP_ERROR_STREAM(get_logger(), "Valid entry [" << i << "] = " << name.GetAscii());
            }
          }
        }
      }
      return false;
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Register " << regname << " is not an enum");
    return false;
  }

  return res.IsOK();
}

// const char *ftp_url = "ftp://example.com/path/to/your/file"; // Replace with your FTP URL
// const char *file_path = "/path/to/local/file"; // Replace with the path to the local file
bool CameraDriver::ftp_upload(const std::string &ftp_url, const std::string &file_path) {
  CURL *curl;
  CURLcode res;
  FILE *hd_src;
  struct stat file_info{};
  curl_off_t fsize;

  // Get the file size
  if(stat(file_path.c_str(), &file_info)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not retrieve file information: " << file_path);
    return false;
  }
  fsize = (curl_off_t)file_info.st_size;

  // Open the file to upload
  hd_src = fopen(file_path.c_str(), "rb");

  curl_global_init(CURL_GLOBAL_ALL);
  curl = curl_easy_init();
  if(curl) {
    // Set up the FTP upload
    curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    curl_easy_setopt(curl, CURLOPT_URL, ftp_url.c_str());
    curl_easy_setopt(curl, CURLOPT_READDATA, hd_src);
    curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, fsize);
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, curlReadCallback);
    curl_easy_setopt(curl, CURLOPT_FTPPORT, "-"); // Use "-" to enable active mode, PASV not supported by camera

    // Perform the upload
    res = curl_easy_perform(curl);

    // Check for errors
    if(res != CURLE_OK) {
      RCLCPP_ERROR_STREAM(get_logger(), "curl_easy_perform() failed: " << curl_easy_strerror(res));
      return false;
    }

    // Clean up
    curl_easy_cleanup(curl);
  }
  fclose(hd_src);

  curl_global_cleanup();
  return true;
}

static void decompose_projection(double *pm, double *tvec, double *rvec){
  if((pm == nullptr) || (tvec == nullptr) || (rvec == nullptr)) return;
    
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
  
  kparams["kWidth"] = (int64_t)cam.width;
  kparams["kHeight"] = (int64_t)cam.height;

  return true;
}

static bool is_calib_valid(sensor_msgs::msg::CameraInfo cam){
  if((cam.width == 0) || (cam.height == 0)){
    return false;
  }

  if(((int32_t)cam.k[0]) == 0) return false;
  if(((int32_t)cam.k[2]) == 0) return false; 
  if(((int32_t)cam.k[4]) == 0) return false;  
  if(((int32_t)cam.k[5]) == 0) return false;

  return true;
}

bool CameraDriver::set_calibration(){
  uint32_t num_sensors = get_num_sensors();
  m_calibrated = false;
  std::map<std::string, std::variant<int64_t, double, bool>> kregisters;

  if(num_sensors == 1){
    m_calibrated = load_calibration(LEFTCAM, "camera");
  } else if(num_sensors == 2){
    m_calibrated = load_calibration(LEFTCAM, "left_camera");
    m_calibrated &= load_calibration(RIGHTCAM, "right_camera");
  } 

  if(m_calibrated){
    m_calibrated = false;
    for(uint32_t i = 0; i < num_sensors; ++i){
      camera_info_manager::CameraInfo info = m_cinfo_manager[i]->getCameraInfo();
      if(!is_calib_valid(info)){
        RCLCPP_ERROR(get_logger(), "Invalid calibration!");
        return m_calibrated;
      }
      if((m_width != (int)info.width) || (m_height != (int)info.height)){
        RCLCPP_ERROR(get_logger(), "Camera resolution mismatch cam_width = %i cam_height = %i "
                                   "vs. calib_with = %i calib_height = %i for camera %i!", m_width, m_height,
                                   info.width, info.height, i);
        return m_calibrated;
      }
      if(!make_calibration_registers(i, info, kregisters)){
        RCLCPP_ERROR(get_logger(), "Only Plumb_bob calibration model supported!");
        return m_calibrated;
      }
    }

    for(auto &kreg:kregisters){
      if(!set_register(kreg.first, kreg.second)){
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to set camera register [" << kreg.first << "]");
        return m_calibrated;
      }      
    }


    if(set_register("saveCalibrationData", true)){
      m_calibrated = set_register("Undistortion", true);
      if(!m_calibrated){
        RCLCPP_ERROR(get_logger(), "Failed to trigger Undistortion mode on camera.");
        return false;
      }
      RCLCPP_DEBUG(get_logger(), "Calibration committed");
      if(num_sensors == 2) {
        if(!get_parameter("sparse_point_cloud").as_bool()) { // Special handling, if we have 3d points decoded, we cannot rectify the images
          if(!set_register("Rectification", true)) {
            RCLCPP_ERROR(get_logger(), "Failed to trigger Rectification mode on camera.");
            return m_calibrated;
          }
          RCLCPP_DEBUG(get_logger(), "Rectification enabled");
        } else {
          if(!set_register("Rectification", false)) {
            RCLCPP_ERROR(get_logger(), "Failed to disable Rectification mode on camera.");
            return m_calibrated;
          }
          RCLCPP_DEBUG(get_logger(), "Rectification disabled");
        }
      }
    } else{
      RCLCPP_ERROR(get_logger(), "Failed to trigger calibration mode on camera.");
      return false;
    }
  }
  
  return m_calibrated;
}

bool CameraDriver::isCalibrated(){
  return m_calibrated;
}

} // namespace bottlenose_camera_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(bottlenose_camera_driver::CameraDriver)
