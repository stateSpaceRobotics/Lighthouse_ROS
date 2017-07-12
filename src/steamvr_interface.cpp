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
    if !(getparam(device_serial_string, device_serials[i])) break;
  }
  ROS_INFO("Total loaded device strings: %i",i);


  // initialize topics
  // initialize pose publishers
  for (i=0;i<16,i++){
    publishers[0] = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(device_names[i],2);
    ROS_DEBUG("Topic: %s",device_names[i]);
  }
}

/** \brief Connects to SteamVR
 *         Connects to SteamVR and initializes needed variables
 *
 *  Connects to SteamVR as a scene application. This maintains full
 *  functionality of the openVR api. This is allows access to the tracking api.
 */
SteamVRInterface::connectToSteamVR(){
  // start with no errors encountered
  vr::EVRInitError err = vr::VRInitError_None;

  // connect to steamvr as a scene to get access to tracking
  steamVR = vr::VR_INIT(&err,vr::VRApplication_Scene);

  // error encountered
  if (err != vr::VRInitError_None) {
    steamVR = NULL;
    ROS_ERROR("Failed to connect to SteamVR");
    return false;
  }

  ROS_INFO("Connected to SteamVR");

  return true;
}

/** \brief Publishes current poses of devices
 *         Polls and publishes poses of tracked devices.
 *
 *  Detailed description.
 */
 SteamVRInterface::update(){
   if (steamVR){
     // Query all device poses
     steamVR->GetDeviceToAbsoluteTrackingPose(
       vr::TrackingUniverseRawAndUncalibrated,
       0,
       vr_device_poses,vr::k_unMaxTrackedDeviceCount
     );

     for(int i = 0;i<16;i++){
       // check for no device/lighthouse
       if ((vr::TrackedDeviceClass_Invalid == vr::GetTrackedDeviceClass(i))
           || (vr::TrackedDeviceClass_TrackingReference == vr::GetTrackedDeviceClass(i))
          ){
           continue; // skip lighthouses
       }

       // Get device serial number
       char serialNumber[vr::k_unMaxPropertyStringSize];
       steamVR->GetStringTrackedDeviceProperty(
         i,
         vr::Prop_SerialNumber_String,
         sb,
         vr::k_unMaxPropertyStringSize
       );
       std::string serial(sreialNumber);
       for(int j=0;j<16;j++){
         // Check if at proper index
         if (serial == device_serials[j]){
           // Build pose and publishes
           publish(i,j);
           break;
         // check if new object added
         } else if (!device_serials[j]){
           device_names[j] = serial;
           publish(i,j);
           break;
         }
       }

     }
   }
 }
