# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pherro/PX4-Autopilot/gz_ws/src/asv_sim/asv_sim_gazebo_plugins"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-build"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/tmp"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src"
  "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/asv_sim2/test/FAKE_INSTALL-prefix/src/FAKE_INSTALL-stamp${cfgdir}") # cfgdir has leading slash
endif()
