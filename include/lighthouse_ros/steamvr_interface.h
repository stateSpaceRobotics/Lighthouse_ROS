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

    int hz;                         /// Desired publish rate for device poses.
    std::string device_serials[16]; /// Stores serial numbers for each device.
    int device_index[16];           /// Stores the index of each device.
    ros::Publisher publishers[16];  /// Stores the pose publishers.
    double covariance[36] = {
        0.0000349162103240595, -0.0000018202960310455, -0.0000339898160507969,
        -0.0000081126791170800, 0.0000001353045808767, 0.0000032202291901186,
        -0.0000018202960310455, 0.0000011910722363973, 0.0000020423436706964,
        0.0000010961526869235, -0.0000000333091396801, -0.0000001408541892558,
        -0.0000339898160507969, 0.0000020423436706964, 0.0000341312090595451,
        0.0000060715616751347, -0.0000000237628610568, -0.0000029217229365340,
        -0.0000081126791170800, 0.0000010961526869235, 0.0000060715616751347,
        0.0000165832615351042, -0.0000004759697840205, -0.0000024486872043021,
        0.0000001353045808767, -0.0000000333091396801, -0.0000000237628610568,
        -0.0000004759697840205, 0.0000003366392930324, -0.0000000030521109214,
        0.0000032202291901186, -0.0000001408541892558, -0.0000029217229365340,
        -0.0000024486872043021, -0.0000000030521109214, 0.0000007445433570531};

    ros::NodeHandle n;

    void createMsg(uint device_index, nav_msgs::Odometry *msg);

    void publish(int device_index, int pub_index);
};

#endif
