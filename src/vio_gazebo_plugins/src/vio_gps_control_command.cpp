#include <chrono>
#include <cerrno>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

int main(int argc, char **argv)
{
  if (argc != 2 || (std::string(argv[1]) != "on" && std::string(argv[1]) != "off")) {
    std::cerr << "usage: vio_gps_control_command on|off\n";
    return 2;
  }

  constexpr const char *sentinel = "/tmp/project_f_vio_gps_off";
  constexpr const char *sentinel_tmp = "/tmp/project_f_vio_gps_off.tmp";
  if (std::string(argv[1]) == "off") {
    {
      std::ofstream stream(sentinel_tmp, std::ios::out | std::ios::trunc);
      if (!stream) {
        std::cerr << "failed to create GPS dropout sentinel temporary file\n";
        return 1;
      }
      stream << "off\n";
      stream.close();
      if (!stream) {
        std::remove(sentinel_tmp);
        std::cerr << "failed to write GPS dropout sentinel temporary file\n";
        return 1;
      }
    }
    if (std::rename(sentinel_tmp, sentinel) != 0) {
      std::remove(sentinel_tmp);
      std::cerr << "failed to atomically install GPS dropout sentinel\n";
      return 1;
    }
  } else if (std::remove(sentinel) != 0 && errno != ENOENT) {
    std::cerr << "failed to remove GPS dropout sentinel\n";
    return 1;
  }
  return 0;
}
