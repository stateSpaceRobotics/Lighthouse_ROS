# lighthouse_ros Installation Guide

## Steam

To begin, first install Steam if you do not already have it:
```bash
sudo apt-get install steam
```
If you run into any issues with launching Steam, try this fix:

#### Removing old libraries that Steam tries to bundle: https://askubuntu.com/questions/762219/steam-wont-start-on-ubuntu-16-04

## SteamVR

Next is to install SteamVR, follow the instructions on their website. You only need to be concerned with parts 4.B. and 4.C. (4.A. if you haven’t installed Steam yet. You can attempt to install the downloaded version through Steam's website if you wish, but the easiest way is through CLI with apt-get.)

#### SteamVR website: https://support.steampowered.com/kb_article.php?ref=2001-UXCM-4439

Make sure, as the instructions on SteamVR's website state, that you are using the Beta version of SteamVR.

Before we can launch SteamVR for ROS purposes (i.e. only using the trackers instead of the Head-Mounted Display (HMD) or the controllers), we must do a little housekeeping:

1. Launch the Steam application.
2. Select the LIBRARY tab at the top and find SteamVR under TOOLS on the left sidebar.
3. Right click SteamVR and select Properties
4. Select the LOCAL FILES tab and then select BROWSE LOCAL FILES
5. In the file navigator, go to resources > settings
6. Open up default.vrsettings with your favorite editor, and leave the editor open
7. Navigate to your catkin workspace directory and go to src > lighthouse_ros
8. Take the default.vrsettings file that is located in lighthouse_ros, and copy and paste it over the default.vrsettings file that was originally opened

Overwriting the first steamvr.settings with the new settings file allows you to run SteamVR without an HMD and still work with the Vive trackers.

Restart Steam and SteamVR.

Verify SteamVR works by running the program and connecting a base station and tracker. For questions on how to connect the Vive tracker, look here: https://dl.vive.com/Tracker/FAQ/Tracker_FAQ_V1.6.pdf

__Note__: Make sure that your tracker is at least several feet away from the base station, else it will likely not pick up tracking.

## openvr

You will also need to install openvr to build lighthouse_ros. Create a folder named “libraries” in your home folder, and clone openvr to your libraries folder like so:

```bash
mkdir ~/libraries
cd ~/libraries
git clone https://github.com/ValveSoftware/openvr.git
```

## Test Run
1. Kick off roscore.

```bash
roscore
```

2. lighthouse_ros should be kicked off with Steam runtime. You can do so with this command:

```bash
~/.steam/ubuntu12_32/steam-runtime/run.sh <path_to_catkin_workspace>/devel/lib/lighthouse_ros/steamvr_interface_node
```

3. At this point, if you have a base station and tracker set up correctly, you should be able to start looking at pose data. You can verify that a single tracker works with the following command: 
```bash
rostopic echo /device01
```
