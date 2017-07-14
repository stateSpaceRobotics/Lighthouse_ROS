#include "lighthouse_ros/steamvr_interface.h"
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
SteamVRInterface::SteamVRInterface() :
  n()
{


  // get parameters
  // get update Hz
  n.param("update_rate",hz,20);
  // get device serial numbers and names
  std::string device_names[16];
  std::string device_name_string = "deviceii";
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
    n.param(device_name_string, device_names[i]);
    //ROS_DEBUG("Device %i name: %s",i,device_names[i].c_str(), device_names[i]);
    n.getParam(device_serial_string, device_serials[i]);
  }
  ROS_INFO("Total loaded device strings: %i",i);


  // initialize topics
  // initialize pose publishers
  for (i=0;i<16;i++){
    publishers[0] = n.advertise<nav_msgs::Odometry>(device_names[i],2);
    ROS_DEBUG("Topic: %s",device_names[i].c_str());
  }
}

/** \brief Connects to SteamVR
 *         Connects to SteamVR and initializes needed variables
 *
 *  Connects to SteamVR as a scene application. This maintains full
 *  functionality of the openVR api. This is allows access to the tracking api.
 */
bool SteamVRInterface::connectToSteamVR(){
  // start with no errors encountered
  vr::EVRInitError err = vr::VRInitError_None;

  // connect to steamvr as a scene to get access to tracking
  steamVR = vr::VR_Init(&err,vr::VRApplication_Background);

  // error encountered
  //if (err != vr::VRInitError_None) {
  //  steamVR = NULL;
    ROS_ERROR("Failed to connect to SteamVR: ERROR %i",err);
   // return false;
  //}
  ROS_INFO("%i",steamVR);
  ROS_INFO("Connected to SteamVR");

  return true;
}

/** \brief Runs main ROS loop
 *         Publishes device odometry at preset rate.
 *
 *  Detailed description.
 */
void SteamVRInterface::run(){
  ros::Rate rate(hz);
  while(ros::ok()){
    update();
    rate.sleep();
  }
}

/** \brief Severs the SteamVR link
 *         Safely shuts down the SteamVR link.
 *
 *  Detailed description.
 */
void SteamVRInterface::shutdown(){
  ROS_INFO("Shutting down SteamVRInterface.");
  if (steamVR){
  vr::VR_Shutdown();
  steamVR = NULL;
  }
}

/** \brief Updates current poses of devices
 *         Polls and publishes poses of tracked devices.
 *
 *  Detailed description.
 */
 void SteamVRInterface::update(){
   if (steamVR){
     // Query all device poses
     steamVR->GetDeviceToAbsoluteTrackingPose(
       vr::TrackingUniverseRawAndUncalibrated,
       0,
       vr_device_poses,vr::k_unMaxTrackedDeviceCount
     );

     for(int i = 0;i<16;i++){
       // check for no device/lighthouse
       if ((vr::TrackedDeviceClass_Invalid == steamVR->GetTrackedDeviceClass(i))
           || (vr::TrackedDeviceClass_TrackingReference == steamVR->GetTrackedDeviceClass(i))
           || (vr::TrackedDeviceClass_HMD == steamVR->GetTrackedDeviceClass(i))
          ){
           continue; // skip lighthouses
       }

       // Get device serial number
       char serialNumber[vr::k_unMaxPropertyStringSize];
       steamVR->GetStringTrackedDeviceProperty(
         i,
         vr::Prop_SerialNumber_String,
         serialNumber,
         vr::k_unMaxPropertyStringSize
       );
       std::string serial(serialNumber);
       for(int j=0;j<16;j++){
         // Check if at proper index
         if (serial == device_serials[j]){
           // Build pose and publishes
           publish(i,j);
           break;
         // check if new object added
         } else if (device_serials[j].empty()){
           device_serials[j] = serial;
           publish(i,j);
           break;
         }
       }

     }
   }
 }


 /** \brief Publishes current pose of specified device to specified topic.
  *         Grabs current pose and publishes it out to topic.
  *
  *  Detailed description.
  */
void SteamVRInterface::publish(int device_index, int pub_index){
    if (vr_device_poses[device_index].bPoseIsValid){
      nav_msgs::Odometry odometry_msg;
      createMsg(device_index, &odometry_msg);
      odometry_msg.header.stamp = ros::Time::now();
      odometry_msg.header.frame_id = "odom";
      odometry_msg.child_frame_id = publishers[pub_index].getTopic();
      odometry_msg.child_frame_id.erase(0,1);
      publishers[pub_index].publish(odometry_msg);
    } else ROS_ERROR("Invalid pose for device: %s", "name");
  }

  /** \brief Publishes current poses of devices
   *         Polls and publishes poses of tracked devices.
   *
   *  Detailed description.
   */
void   SteamVRInterface::createMsg(uint device_index, nav_msgs::Odometry *msg){
     // quaternion
     vr::HmdMatrix34_t mat = vr_device_poses[device_index].mDeviceToAbsoluteTracking;
     msg->pose.pose.orientation.w = sqrt(fmax(0, 1+mat.m[0][0]+mat.m[1][1]+mat.m[2][2]))/2;
     msg->pose.pose.orientation.x = sqrt(fmax(0, 1+mat.m[0][0]-mat.m[1][1]-mat.m[2][2]))/2;
     msg->pose.pose.orientation.y = sqrt(fmax(0, 1-mat.m[0][0]+mat.m[1][1]-mat.m[2][2]))/2;
     msg->pose.pose.orientation.z = sqrt(fmax(0, 1-mat.m[0][0]-mat.m[1][1]+mat.m[2][2]))/2;
     msg->pose.pose.orientation.x = copysign(msg->pose.pose.orientation.x, mat.m[2][1]-mat.m[1][2]);
     msg->pose.pose.orientation.y = copysign(msg->pose.pose.orientation.y, mat.m[0][2]-mat.m[2][0]);
     msg->pose.pose.orientation.z = copysign(msg->pose.pose.orientation.z, mat.m[1][0]-mat.m[0][1]);

     // coordinates
     msg->pose.pose.position.x = mat.m[0][3];
     msg->pose.pose.position.y = mat.m[1][3];
     msg->pose.pose.position.z = mat.m[2][3];

     // covariance
     msg->pose.covariance[0] = -1;

     // linear velocity
     msg->twist.twist.linear.x = vr_device_poses[device_index].vVelocity.v[0];
     msg->twist.twist.linear.y = vr_device_poses[device_index].vVelocity.v[1];
     msg->twist.twist.linear.z = vr_device_poses[device_index].vVelocity.v[2];

     // angular angular velocity
     msg->twist.twist.angular.x = vr_device_poses[device_index].vAngularVelocity.v[0];
     msg->twist.twist.angular.y = vr_device_poses[device_index].vAngularVelocity.v[1];
     msg->twist.twist.angular.z = vr_device_poses[device_index].vAngularVelocity.v[2];

     // covariance
     msg->twist.covariance[0] = -1;

   }

/** \brief Deconstructor method
 *         
 *
 *  Detailed description.
 */
SteamVRInterface::~SteamVRInterface(){
  ROS_INFO("Shutting down...");
  // ROS shutting down, so shutdown SteamVR link
  shutdown();
  return;
}

// Main
int main(int argc, char** argv){
  ros::init(argc, argv, "steamvr_interface");

  SteamVRInterface node;

  // If it can connect to SteamVR
  if (node.connectToSteamVR()) node.run();

  return 0;
}
