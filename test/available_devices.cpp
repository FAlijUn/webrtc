extern "C" {
#include <fcntl.h>  // for open()
}

#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>  // for access

#include <cmath>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>

#include "linux/videodev2.h"

namespace fs = std::filesystem;

inline int xioctl(int fd, uint64_t request, void * arg)
{
  int r = 0;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}
  

int main(){  
  // Initialize vector of device strings to fill in
  std::map<std::string, v4l2_capability> v4l2_devices;

  // Get a list of all v4l2 devices from /sys/class/video4linux
  const std::string v4l2_symlinks_dir = "/sys/class/video4linux/";
  for (const auto & device_symlink : fs::directory_iterator(v4l2_symlinks_dir)) {
    if (fs::is_symlink(device_symlink)) {
      // device_str is the full path to the device in /sys/devices/*
      std::string device_str = fs::canonical(
        v4l2_symlinks_dir + fs::read_symlink(
          device_symlink).generic_string());
      // get the proper device name (e.g. /dev/*)
      std::ifstream uevent_file(device_str + "/uevent");
      std::string line;
      std::string device_name;
      while (std::getline(uevent_file, line)) {
        auto dev_name_index = line.find("DEVNAME=");
        if (dev_name_index != std::string::npos) {
          device_name = "/dev/" + line.substr(dev_name_index + 8, line.size());
          break;
        }
      }

      int fd;
      // Try and open device to test access
      if ((fd = open(device_name.c_str(), O_RDONLY)) == -1) {
        std::cerr << "Cannot open device: `" << device_name << "`, ";
        std::cerr << "double-check read / write permissions for device" << std::endl;
      } else {
        struct v4l2_capability device_capabilities = {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &device_capabilities) == -1) {
          std::cerr << "Could not retrieve device capabilities: `" << device_name;
          std::cerr << "`" << std::endl;
        } else {
          v4l2_devices[device_name] = device_capabilities;
        }
      }
    } else {
      // device doesn't exist, continue
      continue;
    }
  }

  for(const auto& device : v4l2_devices){
    std::cout << "Device: " << device.first << std::endl;
    std::cout << "Driver: " << device.second.driver << std::endl;
    std::cout << "Card: " << device.second.card << std::endl;
    std::cout << "Bus Info: " << device.second.bus_info << std::endl;
    std::cout << "Version: " << device.second.version << std::endl;
    std::cout << "Capabilities: " << device.second.capabilities << std::endl;
    std::cout << std::endl;
  }
  return 0;
}