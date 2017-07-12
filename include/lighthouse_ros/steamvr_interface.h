#ifndef _STEAMVR_INTERFACE_H_
#define _STEAMVR_INTERFACE_H_

#include <openvr.h>

#include <ros.h>

class SteamVRInterface
{
public:
  SteamVRInterface();
  ~SteamVRInterface();

  bool init();
  void shutdown();

  void update();
  void recalibrate();

private:
  std::string device_serials[16];
  int device_index[16];
}
