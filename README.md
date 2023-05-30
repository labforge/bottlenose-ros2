 # ROS2 Camera Driver for Bottlenose Cameras

 * Requirements 
  * ROS2 Foxy or newer, tested with [ROS2 Humble](https://docs.ros.org/en/foxy/Releases/Release-Humble-Hawksbill.html) on Ubuntu 22.04
  * eBUS SDK 6.3, please see the releases in our [SDK Demos](https://github.com/labforge/sdk-demos/releases)
  * Bottlenose Mono or Stereo Camera, at [firmware](https://github.com/labforge/bottlenose/releases/) v0.1.100 or newer


# Building and Installing

 * Setup your ros2 workspace
 * Clone this repository into the workspace
 * Build and install with colcon
```
# Build and install all workspace nodes
colcon build --symlink-install
# Resource the local configuration
source install/local_setup.bash
```

## Usage

```
 ros2 run bottlenose_camera_driver bottlenose_camera_driver_node
```

Available parameters:
 * FIXME

## References
 * Bottlenose [Getting Started](https://docs.labforge.ca/docs/getting-started)
 * Bottlenose [SDK Demos](https://github.com/labforge/sdk-demos)

