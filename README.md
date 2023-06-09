# ROS2 Camera Driver for Bottlenose Cameras (Preliminary)

[![ContinousIntegration](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml)

This driver currently supports color-image streaming from Bottlenose Mono and Stereo.

## Requirements 
  * ROS2 Foxy or newer, tested with [ROS2 Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html) on Ubuntu 22.04
  * eBUS SDK 6.3, please see the releases in our [SDK Demos](https://github.com/labforge/sdk-demos/releases)
  * Bottlenose Mono or Stereo Camera, at [firmware](https://github.com/labforge/bottlenose/releases/) v0.1.100 or newer

## Building and Installing

 * Set up your [ros2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
 * Clone this repository into the workspace
 * Build and install with ```colcon```
```
# Build and install all workspace nodes
colcon build --symlink-install
# Resource the local configuration
source install/local_setup.bash
```

## Usage
 * Start the driver node directly or via launch file
```
# Source the Genican environment variables from your eBUS installation or add them to .bashrc
source /opt/pleora/ebus_sdk/Ubuntu-<your version>/bin/set_puregev_env.sh
ros2 run bottlenose_camera_driver bottlenose_camera_driver_node --ros-args -p mac_address:="8C:1F:64:D0:E0:0C" --log-level bottlenose_camera_driver:=debug
```
 * In a separate terminal, view the camera stream
```
ros2 run image_view image_view --ros-args --remap /image:=/image_raw
```

### Available parameters

| Parameter                               | Description                                                                   | Default                   | Run-time adjustable |
|-----------------------------------------|-------------------------------------------------------------------------------|---------------------------|---------------------|
| ```mac_address```                       | The MAC address of Bottlenose                                                 | ```00:00:00:00:00:00```   | :x:                 |
| ```frame_id```                          | The frame_id embedded in image headers                                        | ```camera```              | :heavy_check_mark:  |
| ```keep_partial```                      | Keep partial images (i.e. corrupted in transmission)                          | ```false```               | :heavy_check_mark:  |
| ```camera_calibration_file```           | Camera Calibration File (*)                                                   | ```config/camera.yaml```  | :x:                 |
| ```fps```                               | Target frames per second (**)                                                 | ```20```                  | :x:                 |
| ***Image Sensor(s) Controls***          | (***)                                                                         |                           |                     |
| ```exposure```                          | Exposure time in milliseconds                                                 | ```20```                  | :heavy_check_mark:  |
| ```gain```                              | Analog gain                                                                   | ```1.0```                 | :heavy_check_mark:  |
| ```dGainBlue```                         | Digital gain for blue pixels in Bayer array                                   | ```1024```                | :heavy_check_mark:  |
| ```dgainGB```                           | Digital gain for green-blue pixels in Bayer array                             | ```1024```                | :heavy_check_mark:  |
| ```dgainGR```                           | Digital gain for green-red pixels in Bayer array                              | ```1024```                | :heavy_check_mark:  |
| ```dGainRed```                          | Digital gain for red pixels in Bayer array                                    | ```1024```                | :heavy_check_mark:  |
| **Image Processing Controls**           |                                                                               |                           |                     |
| ```gamma```                             | Gamma correction of the image                                                 | ```2.2```                 | :heavy_check_mark:  |
| ```blackBlue```                         | Black level for blue pixels in Bayer array                                    | ```255```                 | :heavy_check_mark:  |
| ```blackGB```                           | Black level for green-blue pixels in Bayer array                              | ```255```                 | :heavy_check_mark:  |
| ```blackGR```                           | Black level for green-red pixels in Bayer array                               | ```255```                 | :heavy_check_mark:  |
| ```blackRed```                          | Black level for red pixels in Bayer array                                     | ```255```                 | :heavy_check_mark:  |
| ```blackGainBlue```                     | Black gain for blue pixels in Bayer array                                     | ```0.9375```              | :heavy_check_mark:  |
| ```blackGainGB```                       | Black gain for green-blue pixels in Bayer array                               | ```0.9375```              | :heavy_check_mark:  |
| ```blackGainGR```                       | Black gain for green-red pixels in Bayer array                                | ```0.9375```              | :heavy_check_mark:  |
| ```blackGainRed```                      | Black gain for red pixels in Bayer array                                      | ```0.9375```              | :heavy_check_mark:  |
| ```brightness```                        | Brightness of the image                                                       | ```-13107```              | :heavy_check_mark:  |
| ```linearContrast```                    | Linear contrast of the image                                                  | ```136```                 | :heavy_check_mark:  |
| ```wbBlue```                            | White balance for blue component                                              | ```1.0```                 | :heavy_check_mark:  |
| ```wbGreen```                           | White balance for green component                                             | ```1.0```                 | :heavy_check_mark:  |
| ```wbRed```                             | White balance for red component                                               | ```1.0```                 | :heavy_check_mark:  |
| **Color Correction Controls**           |                                                                               |                           |                     |
| ```custom_ccm```                        | Enable custom color correction matrix                                         | ```false```               | :x:                 |
| ```CCMValue00```                        | Color correction matrix value at row 0, column 0 (only if custom_ccm=true)    | ```1.0```                 | :x:                 |
| ```CCMValue01```                        | Color correction matrix value at row 0, column 1 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue02```                        | Color correction matrix value at row 0, column 2 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue10```                        | Color correction matrix value at row 1, column 0 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue11```                        | Color correction matrix value at row 1, column 1 (only if custom_ccm=true)    | ```1.0```                 | :x:                 |
| ```CCMValue12```                        | Color correction matrix value at row 1, column 2 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue20```                        | Color correction matrix value at row 2, column 0 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue21```                        | Color correction matrix value at row 2, column 1 (only if custom_ccm=true)    | ```0.0```                 | :x:                 |
| ```CCMValue22```                        | Color correction matrix value at row 2, column 2 (only if custom_ccm=true)    | ```1.0```                 | :x:                 |
| **GigE Vision Stream Parameters**       |                                                                               |                           |                     |
| ```AnswerTimeout```                     | Time the GigE Vision Device can take for command response.                    | ```100```                 | :x:                 |
| ```CommandRetryCount```                 | Command attempts before it is considered as failed                            | ```50```                  | :x:                 |
| ```MaximumPendingResends```             | Maximum number of packets in a block that can be missing                      | ```0```                   | :x:                 |
| ```MaximumResendRequestRetryByPacket``` | The maximum number of times a resend request can be issued.                   | ```0```                   | :x:                 |
| ```MaximumResendGroupSize```            | Maximum number of packets to resend at once                                   | ```0```                   | :x:                 |
| ```ResendRequestTimeout```              | Timeout for resend requests in (us)                                           | ```100```                 | :x:                 |
| ```RequestTimeout```                    | Maximum time that the data receiver waits for all the packets of a block (ms) | ```10000```               | :x:                 |
| ```ResetOnIdle```                       | Time without packets before resetting itself                                  | ```2000```                | :x:                 |

(*) Note: Required parameter. Even if the image stream is desired to be raw, the calibration dictates the resolution of 
the image. 

(**) Note: effective limitations are imposed by available bandwidth for the chosen configuration. If the bandwidth is
exceeded the camera will drop frames. 

(**) Note: For stereo cameras the controls are applied to both sensors simultaneously.

 * Required parameters:
   * mac_address: The MAC address of the camera to connect to

### Published Topics
```
bottlenose_camera_driver
 |
 +-- camera_info          : Camera calibration data
 +-- image_color          : Color image stream of Bottlenose camera (unrectified, if rectification is disabled)
 +-- image_color_1        : Color image stream of Bottlenose camera (unrectified, if rectification is disabled), 
 |                          stereo only
 +-- image_rect_color     : Rectified color image stream of Bottlenose camera (if rectification is enabled)
 +-- image_rect_color_1   : Rectified color image stream of Bottlenose camera (if rectification is enabled), 
 |                          stereo only
 +-- depth
     +-- image_rect_color : Rectified disparity image stream of Bottlenose camera (if depth is enabled)
```
In the current release the topics ```image_color```, ```image_rect_color``` and ```depth/image_rect_color``` 
are exclusive. See the below for the available configurations:

 * ```mode=0``` ***Color streaming***
   * ```rectify=0``` -> ```image_color{_1}``` (*default*)
   * ```rectify=1``` -> ```image_rect_color{_1}```
   * ***Stereo only***
     * Set sensor to ```0``` or ```1``` to switch between left or right sensor
   
 * ```mode=1``` ***Stereo streaming***
   * ```rectify=0``` -> ```image_color``` and ```image_color_1```
   * ```rectify=1``` -> ```image_rect_color``` and ```image_rect_color_1```

 * ```mode=3``` ***Disparity streaming***
   * ```rectify=1``` -> ```depth/image_rect_color``` 
   * Note ```rectify=0``` is not supported for depth streaming

## References
 * Bottlenose [Getting Started](https://docs.labforge.ca/docs/getting-started)
 * Bottlenose [Calibration Guide](https://docs.labforge.ca/docs/3d-modules) 
 * Bottlenose [SDK Demos](https://github.com/labforge/sdk-demos)

