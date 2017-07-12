#include "steamvr_interface.h"

/** \brief Brief description.
 *         Brief description cont.
 *
 *  Detailed description.
 */

/** \brief Constructor method
 *         Initializes connection to ROS and loads parameters.
 *
 *  The node is created and connected to ROS. It then loads all necessary
 *  config settings and objects in perperation for connecting to the SteamVR
 *  runtime. Published topics and services are also named and initialized.
 */
SteamVRInterface::SteamVRInterface()
{


  // get parameters
  // get update Hz
  n.param("update_rate",hz,20)
  // get device serial numbers and names
  std::string device_names[16];
  std::string device_name_string = "deviceii"
  std::string device_serial_string = "deviceii_serial";
  int i;
  for (i=0;i<16;i++){
    // configure parameter strings for parameter server query
    device_serial_string[6] = '0' + ((i+1) / 10);
    device_name_string[6] = device_serial_string[6];
    device_serial_string[7] = '0' + ((i+1) % 10);
    device_name_string[7] = device_serial_string[7];
    // set name default
    device_names[i] = device_name_string;
    //
    if (getparam(device_name_string, device_names[i])) ROS_DEBUG("Device %i name: %s",i,device_names[i]);
    if ~(getparam(device_serial_string, device_serials[i])) break;
  }
  ROS_INFO("Total loaded device strings: %i",i);


  // initialize topics
  // initialize pose publishers
  for (i=0;i<16,i++){
    publishers[0] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(device_names[i],2);
  }
}
