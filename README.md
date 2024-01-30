# ROS2 Camera Driver for Bottlenose Cameras

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
# Source the Genicam environment variables from your eBUS installation or add them to `.bashrc`
source /opt/pleora/ebus_sdk/Ubuntu-<your version>/bin/set_puregev_env.sh

# start the ros2 node. 
# replace <MAC> with the MAC address of your camera (see product label or output of eBusPlayer connection dialog)
ros2 run bottlenose_camera_driver bottlenose_camera_driver_node --ros-args -p mac_address:="<MAC>" -p stereo:=true

```
   * To run in Mono mode, set the parameter ```-p stereo:=false```   


  * Alternatively, a launch file can be used. 
  ```
  # Launch file method. Before, running, please, edit the Python script with your corresponding MAC address.
  ros2 launch bottlenose-ros2/launch/bottlenose_camera_node.launch.py
  ``` 


### Default Parameters

| Parameter                                | Description                                                                      | Default                          | Run-time adjustable |
|------------------------------------------|----------------------------------------------------------------------------------|----------------------------------|---------------------|
| ```mac_address```                        | The MAC address of Bottlenose                                                    | ```00:00:00:00:00:00```          |                     |
| ```frame_id```                           | The frame_id embedded in image headers                                           | ```camera```                     | :white_check_mark:  |
| ```keep_partial```                       | Keep partial images (i.e. corrupted in transmission)                             | ```false```                      | :white_check_mark:  |
| ```format```                             | Format of the camera (*)                                                         | ```1920x1080```                  |                     |
| ```fps```                                | Target frames per second (*)                                                     | ```10.0```                       |                     |
| ```ntpEnable```                          | Enable NTP UDP broadcast receptions on Bottlenose                                | ```false```                      |                     |
| ```stereo```                             | Enable stereo mode (Bottlenose Stereo only)                                      | ```false```                      |                     |
| ```feature_points```                     | Configure feature point type {```none```, ```gftt```, ```fast9```)               | ```none```                       |                     |
| ```ai_model```                           | Configure the path to the AI model file (enables Bottlenose on-board AI)         | ```""```                         |                     |
| ***Image Sensor(s) Controls***           | (**)                                                                             |                                  |                     |
| ```exposure```                           | Exposure time in milliseconds                                                    | ```20.0```                       | :white_check_mark:  |
| ```gain```                               | Analog gain                                                                      | ```1.0```                        | :white_check_mark:  |
| ```dGainBlue```                          | Digital gain for blue pixels in Bayer array                                      | ```1024```                       | :white_check_mark:  |
| ```dgainGB```                            | Digital gain for green-blue pixels in Bayer array                                | ```1024```                       | :white_check_mark:  |
| ```dgainGR```                            | Digital gain for green-red pixels in Bayer array                                 | ```1024```                       | :white_check_mark:  |
| ```dGainRed```                           | Digital gain for red pixels in Bayer array                                       | ```1024```                       | :white_check_mark:  |
| **Image Processing Controls**            |                                                                                  |                                  |                     |
| ```gamma```                              | Gamma correction of the image                                                    | ```2.2```                        | :white_check_mark:  |
| ```wbBlue```                             | White balance for blue component                                                 | ```1.0```                        | :white_check_mark:  |
| ```wbGreen```                            | White balance for green component                                                | ```1.0```                        | :white_check_mark:  |
| ```wbRed```                              | White balance for red component                                                  | ```1.0```                        | :white_check_mark:  |
| ```wbAuto```                             | Auto white balance control                                                       | ```true```                       | :white_check_mark:  |
| ```blackBlue```                          | Black level for blue pixels in Bayer array                                       | ```4200```                       | :white_check_mark:  |
| ```blackGB```                            | Black level for green-blue pixels in Bayer array                                 | ```4200```                       | :white_check_mark:  |
| ```blackGR```                            | Black level for green-red pixels in Bayer array                                  | ```4200```                       | :white_check_mark:  |
| ```blackRed```                           | Black level for red pixels in Bayer array                                        | ```4200```                       | :white_check_mark:  |
| **Color Correction Controls**            |                                                                                  |                                  |                     |
| ```CCMColorProfile```                    | Color correction profile preset                                                  | ```IndoorWarmLightCurtainOpen``` |                     |
| ```CCMCustom```                          | Custom color correction matrix (only valid if ```CCMColorProfile```=```Custom``` | ```1,0,0;0,1,0;0,0,1```          |                     |
| **Auto Exposure and Auto Gain Controls** |                                                                                  |                                  |                     |
| ```autoExposureEnable```                 | Enable auto exposure and auto gain control                                       | ```false```                      |                     |
| ```autoExposureLuminanceTarget```        | Luminance target for exposure control ```[0, 65535]```                           | ```16384```                      |                     |
| **Feature Point Controls**               |                                                                                  |                                  |                     |
| ```features_max```                       | Maximum number of features to detect                                             | ```1000```                       |                     |
| ```features_threshold```                 | Threshold for feature detection ```[0,255]``` (***fast9 only***)                 | ```100```                        |                     |
| ```features_nms```                       | Use Non-maximum suppression (***fast9 only***)                                   | ```false```                      |                     |
| **AI Model Controls**                    |                                                                                  |                                  |                     |
| ```DNNMaxDetections```                   | Maximum number of detections                                                     | ```100```                        |                     |
| ```DNNNonMaxSuppression```               | Set the non-maximum suppression value for bounding boxes.                        | ```0.45```                       |                     |
| ```DNNConfidence```                      | Set confidence threshold of the detector.                                        | ```0.2```                        |                     |
| **GigE Vision Stream Parameters**        |                                                                                  |                                  |                     |
| ```AnswerTimeout```                      | Time the GigE Vision Device can take for command response.                       | ```1000```                       |                     |
| ```CommandRetryCount```                  | Command attempts before it is considered as failed                               | ```3```                          |                     |
| ```MaximumPendingResends```              | Maximum number of packets in a block that can be missing                         | ```512```                        |                     |
| ```MaximumResendRequestRetryByPacket```  | The maximum number of times a resend request can be issued.                      | ```3```                          |                     |
| ```MaximumResendGroupSize```             | Maximum number of packets to resend at once                                      | ```15```                         |                     |
| ```ResendRequestTimeout```               | Timeout for resend requests in (us)                                              | ```5000```                       |                     |
| ```RequestTimeout```                     | Maximum time that the data receiver waits for all the packets of a block (ms)    | ```1000```                       |                     |
| ```ResetOnIdle```                        | Time without packets before resetting itself                                     | ```200```                        |                     |
| ```Timeout```                            | Buffer reception timeout in (ms)                                                 | ```5000```                       |                     |


(*) Choose a reasonable resolution and frame rate for your network, otherwise the camera may drop frames. 
(**) For stereo versions, the parameters are applied to both cameras simultaneously.

The above parameters can be changed via a flag as `-p <parameter_name>:=<value>`, e.g. `-p exposure:=10.0` 

```
# To view the left or mono stream
ros2 run image_view image_view --ros-args --remap /image:=/image_color

# To view the right stream (not supported for Mono models, stereo:=true must be set, see above)
ros2 run image_view image_view --ros-args --remap /image:=/image_color_1
```

### Published Topics
```
bottlenose_camera_driver
 |
 +-- image_color      : Color image stream of Bottlenose camera (in case of Stereo of the left sensor)
 +-- image_color_1    : Color image stream of Bottlenose camera (in case of Stereo of the right sensor, not supported for Mono models)
 +-- detections       : Detections2D array of detected features in left (image_color) or mono sensor
 +-- features         : ImageMarker2D array of detected features in left (image_color) or mono sensor
 +-- features_1       : ImageMarker2D array of detected features in right (image_color_1) sensor (Bottlenose Stereo only)
```

### Featurepoint example
```bash
ros2 run bottlenose_camera_driver bottlenose_camera_driver_node --ros-args \
    -p mac_address:="<MAC>" \ 
    -p stereo:=true \
    -p feature_points:=fast9 \
    -p features_threshold:=10
```
 * Enable stereo processing and set the feature point type to ```fast9``` with a confidence threshold of ```10```
 * You can use [Foxglove studio](https://foxglove.dev/) to visualize the makers in the image

### AI example
```bash
ros2 run bottlenose_camera_driver bottlenose_camera_driver_node --ros-args \
  -p mac_address:="<MAC>" \ 
  -p ai_model:=<absolute_path>yolov3_1_416_416_3.tar \
  -p DNNConfidence:=0.01 
```
* Enable AI detections on Bottlenose using the yolov3 model with a confidence threshold of ```0.01``` 
  (see [our models](https://github.com/labforge/models)).
* You can use [Detection Visualizer](https://github.com/labforge/ros2_detection_visualizer) to annotate the images with 
  the detections and subscribe to the annotated images in [Foxglove studio](https://foxglove.dev/) 

### Save topics in a bag
```
# The following can be used to collect a stereo bag with the default parameters above.
ros2 record image_color image_color_1
```

### Visualize message from a topic
```
# This command allows you to visualize the content of the incoming messages from the `image_color` topic.
ros2 topic echo /image_color
```
The following is the output of the above command:
```yaml
header:
   stamp:
      sec: 5867905
      nanosec: 748000000
   frame_id: camera
height: 1080
width: 1920
encoding: rgb8
is_bigendian: 0
step: 5760
data:
- 20
- 45
- 30
- 21
- 47
- 31
- 24
- 47
- 30
- '...'
```

The timestamp of the data is composed of two parts: the second and the nanosecond component. The full timestamp can be decoded as `timestamp = stamp[sec] + stamp[nanosec]*1e-9`. The timestamp of the message originates from the camera at the time the image is taken.


### Common Mistakes

#### Mac address not configured correctly
No images will be sent by the driver, the driver will attempt to reconnect indefinitely, see this log line
```
[ERROR] [1694642295.072355485] [bottlenose_camera_driver]: Failed to find device 8C:1F:64:D0:E0:0F
[ERROR] [1694642298.090279520] [bottlenose_camera_driver]: Failed to find device 8C:1F:64:D0:E0:0F
```
Workaround: Please set the appropriate MAC address in the launch file, please see product labelling

#### Missing MAC address configuration in the launch file
```
[INFO] [1694642348.548832114] [bottlenose_camera_driver]: Bottlenose undefined please set mac_address
```
Workaround: Please set the appropriate MAC address in the launch file, please see product labelling

#### eBusDriver not properly installed or an automated kernel update removed it
```
[ERROR] [1694642437.179197654] [bottlenose_camera_driver]: The eBus Driver is not loaded, please reinstall the driver!
``` 
The driver will attempt to stream from the sensor using the standard Linux network stack with degraded performance

Workaround:
  * Please make sure you **do not** install Linux with ***Secure Boot*** or ***UEFI*** enabled.
  * Please reinstall the ***eBus SDK*** Debian package (see above) such that the kernel driver is reinstalled after every kernel update


## References
 * Bottlenose [Getting Started](https://docs.labforge.ca/docs/getting-started)
 * Bottlenose [SDK Demos](https://github.com/labforge/sdk-demos)

