 # ROS2 Camera Driver for Bottlenose Cameras (Preliminary)

[![ContinousIntegration](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/labforge/bottlenose-ros2/actions/workflows/ci.yml)

This driver currently supports color-image streaming from Bottlenose Mono and Stereo.

 * Requirements 
  * ROS2 Foxy or newer, tested with [ROS2 Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html) on Ubuntu 22.04
  * eBUS SDK 6.3, please see the releases in our [SDK Demos](https://github.com/labforge/sdk-demos/releases)
  * Bottlenose Mono or Stereo Camera, at [firmware](https://github.com/labforge/bottlenose/releases/) v0.1.100 or newer

# Building and Installing

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

| Parameter                               | Description                                                                   | Default                 | Run-time adjustable |
|-----------------------------------------|-------------------------------------------------------------------------------|-------------------------|---------------------|
| ```mac_address```                       | The MAC address of Bottlenose                                                 | ```00:00:00:00:00:00``` | :x:                 |
| ```frame_id```                          | The frame_id embedded in image headers                                        | ```camera```            | :heavy_check_mark:  |
| ```keep_partial```                      | Keep partial images (i.e. corrupted in transmission)                          | ```false```             | :heavy_check_mark:  |
| ```format```                            | Format of the camera (*)                                                      | ```1920x1440```         | :x:                 |
| ```fps```                               | Target frames per second (*)                                                  | ```20```                | :x:                 |
| **GigE Vision Stream Parameters**       |                                                                               |                         |                     |
| ```AnswerTimeout```                     | Time the GigE Vision Device can take for command response.                    | ```100```               | :x:                 |
| ```CommandRetryCount```                 | Command attempts before it is considered as failed                            | ```50```                | :x:                 |
| ```MaximumPendingResends```             | Maximum number of packets in a block that can be missing                      | ```0```                 | :x:                 |
| ```MaximumResendRequestRetryByPacket``` | The maximum number of times a resend request can be issued.                   | ```0```                 | :x:                 |
| ```MaximumResendGroupSize```            | Maximum number of packets to resend at once                                   | ```0```                 | :x:                 |
| ```ResendRequestTimeout```              | Timeout for resend requests in (us)                                           | ```100```               | :x:                 |
| ```RequestTimeout```                    | Maximum time that the data receiver waits for all the packets of a block (ms) | ```10000```             | :x:                 |
| ```ResetOnIdle```                       | Time without packets before resetting itself                                  | ```2000```              | :x:                 |
 (*) Note: effective limitations are imposed by available bandwidth for the chosen configuration. If the bandwidth is
exceeded the camera will drop frames. 

 * Required parameters:
   * mac_address: The MAC address of the camera to connect to

### Published Topics
```
bottlenose_camera_driver
 |
 +-- camera_image_raw      : Color image stream of Bottlenose camera
```

## References
 * Bottlenose [Getting Started](https://docs.labforge.ca/docs/getting-started)
 * Bottlenose [SDK Demos](https://github.com/labforge/sdk-demos)

