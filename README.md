# lighthouse_ros
Basic library to import Valve's Lighthouse tracking system into ROS

This library uses the OpenVR C++ API to grab tracking data from Valve's SteamVR runtime. This data is then piped into ROS via an Odometry message. The API supplies both and linear and angular positions and velocities. Each device's odometry data is published to a separate topic. These topics can be renamed via parameters. Additionally, specific devices (denoted by a serial number), can be assigned to specific topics.

The CMake file currently does not automatically locate your OpenVR SDK location. Instead, it assumes it is placed in a libraries directory in your home folder. If it is installed elsewhere, change the path in the CMake file.
