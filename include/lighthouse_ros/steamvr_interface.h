#ifndef _STEAMVR_INTERFACE_H_
#define _STEAMVR_INTERFACE_H_

#include <openvr.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class SteamVRInterface
{
public:
  SteamVRInterface();
  ~SteamVRInterface();

  bool connectToSteamVR();
  void shutdown();

  void update();
  void recalibrate();

  void run();

private:
  vr::TrackedDevicePose_t vr_device_poses[vr::k_unMaxTrackedDeviceCount];
  vr::IVRSystem *steamVR; /// pointer to the steamVR connection object.

  int hz; /// Desired publish rate for device poses.
  std::string device_serials[16]; /// Stores serial numbers for each device.
  int device_index[16]; /// Stores the index of each device.
  ros::Publisher publishers[16]; /// Stores the pose publishers.

  ros::NodeHandle n;

  void createMsg(uint device_index, nav_msgs::Odometry *msg);

  void publish(int device_index, int pub_index);
};

#endif
