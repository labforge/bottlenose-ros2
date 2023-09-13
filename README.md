# ROS2 Camera Driver for Bottlenose Cameras (Preliminary)

[![ContinousIntegration](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml)

This driver currently supports color-image streaming from Bottlenose Mono and Stereo.

## Requirements 
  * ROS2 Foxy or newer:
    * tested with [ROS2 Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html) on Ubuntu 22.04
    * tested with [ROS2 Foxy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html) on Ubuntu 20.04
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
# replace <MAC> with the MAC address of your camera (see product label or output of eBusPlayer connection dialog)
ros2 run bottlenose_camera_driver bottlenose_camera_driver_node --ros-args -p mac_address:="<MAC>" --log-level bottlenose_camera_driver:=debug
```
   * The camera is running in mono mode. To run in stereo mode, add the parameter ```-p stereo:=true``` (*Bottlenose Stereo models only)
   * Any other parameters can be set as well, see below for a list of available parameters ```-p <parameter_name>:=<value>```, e.g. ```-p exposure:=10.0```
 * In a separate terminal, view the camera stream
```
# View the left or mono stream
ros2 run image_view image_view --ros-args --remap /image:=/camera_image_color
# View the right or stereo stream (not supported for Mono models, stereo:=true must be set, see above)
ros2 run image_view image_view --ros-args --remap /image:=/camera_image_color_1
```
 * To perform a launch file, use the provided example launch file (please set ```mac_address``` to the MAC address of 
   your camera in the launch file)
   * ```ros2 launch bottlenose-ros2/launch/bottlenose_camera_node.launch.py``` 

### Available parameters

| Parameter                               | Description                                                                   | Default                          | Run-time adjustable  |
|-----------------------------------------|-------------------------------------------------------------------------------|----------------------------------|----------------------|
| ```mac_address```                       | The MAC address of Bottlenose                                                 | ```00:00:00:00:00:00```          | :x:                  |
| ```frame_id```                          | The frame_id embedded in image headers                                        | ```camera```                     | :heavy_check_mark:   |
| ```keep_partial```                      | Keep partial images (i.e. corrupted in transmission)                          | ```false```                      | :heavy_check_mark:   |
| ```format```                            | Format of the camera (*)                                                      | ```1920x1080```                  | :x:                  |
| ```fps```                               | Target frames per second (*)                                                  | ```10.0```                       | :x:                  |
| ***Image Sensor(s) Controls***          | (**)                                                                          |                                  |                      |
| ```exposure```                          | Exposure time in milliseconds                                                 | ```20.0```                       | :heavy_check_mark:   |
| ```gain```                              | Analog gain                                                                   | ```1.0```                        | :heavy_check_mark:   |
| ```dGainBlue```                         | Digital gain for blue pixels in Bayer array                                   | ```1024```                       | :heavy_check_mark:   |
| ```dgainGB```                           | Digital gain for green-blue pixels in Bayer array                             | ```1024```                       | :heavy_check_mark:   |
| ```dgainGR```                           | Digital gain for green-red pixels in Bayer array                              | ```1024```                       | :heavy_check_mark:   |
| ```dGainRed```                          | Digital gain for red pixels in Bayer array                                    | ```1024```                       | :heavy_check_mark:   |
| **Image Processing Controls**           |                                                                               |                                  |                      |
| ```gamma```                             | Gamma correction of the image                                                 | ```2.2```                        | :heavy_check_mark:   |
| ```wbBlue```                            | White balance for blue component                                              | ```1.0```                        | :heavy_check_mark:   |
| ```wbGreen```                           | White balance for green component                                             | ```1.0```                        | :heavy_check_mark:   |
| ```wbRed```                             | White balance for red component                                               | ```1.0```                        | :heavy_check_mark:   |
| ```blackBlue```                         | Black level for blue pixels in Bayer array                                    | ```4200```                       | :heavy_check_mark:  |
| ```blackGB```                           | Black level for green-blue pixels in Bayer array                              | ```4200```                       | :heavy_check_mark:  |
| ```blackGR```                           | Black level for green-red pixels in Bayer array                               | ```4200```                       | :heavy_check_mark:  |
| ```blackRed```                          | Black level for red pixels in Bayer array                                     | ```4200```                       | :heavy_check_mark:  |
| **Color Correction Controls**           |                                                                               |                                  |                      |
| ```CCMColorProfile```                   | Color correction profile preset                                               | ```IndoorWarmLightCurtainOpen``` | :x:         |
| **GigE Vision Stream Parameters**       |                                                                               |                                  |                      |
| ```AnswerTimeout```                     | Time the GigE Vision Device can take for command response.                    | ```100```                        | :x:                  |
| ```CommandRetryCount```                 | Command attempts before it is considered as failed                            | ```50```                         | :x:                  |
| ```MaximumPendingResends```             | Maximum number of packets in a block that can be missing                      | ```0```                          | :x:                  |
| ```MaximumResendRequestRetryByPacket``` | The maximum number of times a resend request can be issued.                   | ```0```                          | :x:                  |
| ```MaximumResendGroupSize```            | Maximum number of packets to resend at once                                   | ```0```                          | :x:                  |
| ```ResendRequestTimeout```              | Timeout for resend requests in (us)                                           | ```100```                        | :x:                  |
| ```RequestTimeout```                    | Maximum time that the data receiver waits for all the packets of a block (ms) | ```10000```                      | :x:                  |
| ```ResetOnIdle```                       | Time without packets before resetting itself                                  | ```2000```                       | :x:                  |
| ```Timeout```                           | Buffer reception timeout in (ms)                                              | ```5000```                       | :x:                  |
 
(*) Note: effective limitations are imposed by available bandwidth for the chosen configuration. If the bandwidth is
exceeded the camera will drop frames. 

(**) Note: For stereo cameras the controls are applied to both sensors simultaneously.

 * Required parameters:
   * mac_address: The MAC address of the camera to connect to

### Published Topics
```
bottlenose_camera_driver
 |
 +-- camera_image_color      : Color image stream of Bottlenose camera (in case of Stereo of the left sensor)
 +-- camera_image_color_1    : Color image stream of Bottlenose camera (in case of Stereo of the right sensor, not supported for Mono models)
```

### Common Mistakes

#### Mac address not configured correctly
 * No images will be sent by the driver, the driver will attempt to reconnect indefinitely, see this log line
```
[ERROR] [1694642295.072355485] [bottlenose_camera_driver]: Failed to find device 8C:1F:64:D0:E0:0F
[ERROR] [1694642298.090279520] [bottlenose_camera_driver]: Failed to find device 8C:1F:64:D0:E0:0F
```
 * ***Workaround:*** Please set the appropriate MAC address in the launch file, please see product labelling

#### Missing MAC address configuration in the launch file
```
[INFO] [1694642348.548832114] [bottlenose_camera_driver]: Bottlenose undefined please set mac_address
```
 * ***Workaround:*** Please set the appropriate MAC address in the launch file, please see product labelling

#### eBusDriver not properly installed
```
[ERROR] [1694642437.179197654] [bottlenose_camera_driver]: The eBus Driver is not loaded, please reinstall the driver!
``` 
* The driver will attempt to stream from the sensor using the standard Linux network stack with degraded performance
* ***Workaround***:
    * Please make sure you **do not** install Linux with ***Secure Boot*** or ***UEFI*** enabled.
    * Please reinstall the ***eBus SDK*** Debian package (see above) such that the kernel driver is reinstalled


## References
 * Bottlenose [Getting Started](https://docs.labforge.ca/docs/getting-started)
 * Bottlenose [SDK Demos](https://github.com/labforge/sdk-demos)

