# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitclone-lastrun.txt" AND EXISTS "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitinfo.txt" AND
  "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitclone-lastrun.txt" IS_NEWER_THAN "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/dstahlke/gnuplot-iostream.git" "gnuplot-iostream-src"
    WORKING_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/dstahlke/gnuplot-iostream.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"
          checkout "d674bdf23b93c76d491f03246d2e6f72bf5739ce" --
  WORKING_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'd674bdf23b93c76d491f03246d2e6f72bf5739ce'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitinfo.txt" "/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/pherro/PX4-Autopilot/gz_ws/build/gz-waves1/_deps/gnuplot-iostream-subbuild/gnuplot-iostream-populate-prefix/src/gnuplot-iostream-populate-stamp/gnuplot-iostream-populate-gitclone-lastrun.txt'")
endif()
