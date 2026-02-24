# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-build"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/tmp"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src"
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
